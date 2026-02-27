//! [`AgentLoop`] – The Definitive OODA Orchestrator.
//!
//! Implements the infinite Observe–Orient–Decide–Act–Gatekeep cycle that keeps
//! the robot alive.  Each tick:
//!
//! 1. **Observe** – query [`SensorFusion`] for the latest [`FusedState`] and
//!    check the [`Octree`] for collision data.
//! 2. **Orient** – format the state into a strict system prompt and retrieve
//!    relevant memories from the [`EpisodicStore`].
//! 3. **Decide** – call [`LlmDriver::complete`].  The returned JSON is hashed
//!    and checked against [`LoopGuard`] to ensure the agent isn't stuck in a
//!    repetitive hallucination loop.
//! 4. **Gatekeep** – the parsed [`HardwareIntent`] is checked by
//!    [`CapabilityManager`] (permission) and [`StateVerifier`] (physical
//!    invariants) via [`KernelGate`].
//! 5. **Act** – the approved intent is published to the [`EventBus`].
//!
//! # Human-in-the-Loop (HITL)
//!
//! When the LLM outputs an [`HardwareIntent::AskHuman`] intent the loop
//! automatically pauses.  Subsequent calls to [`AgentLoop::tick`] return
//! [`MechError::LlmInferenceFailed`] until a human operator supplies an answer
//! via [`AgentLoop::submit_human_response`].  The answer is then injected into
//! the LLM context window as a [`Role::User`] message and the OODA cycle
//! resumes normally.
//!
//! # Manual Override (Safety Interlock)
//!
//! Calling [`AgentLoop::handle_manual_override`] arms a 10-second AI
//! suspension.  While the suspension is active [`tick`] returns early without
//! invoking the LLM, and the [`ManualOverrideInterlock`] rule registered on the
//! [`StateVerifier`] rejects any AI-sourced `Drive` commands that may have been
//! in flight.  The suspension is cleared automatically once 10 seconds have
//! elapsed since the last call to `handle_manual_override`.
//!
//! # Example
//!
//! ```rust,no_run
//! use mechos_runtime::agent_loop::{AgentLoop, AgentLoopConfig};
//!
//! // Build the loop with sensible defaults and run one tick.
//! let config = AgentLoopConfig::default();
//! let mut agent = AgentLoop::new(config).expect("failed to initialise agent loop");
//! // agent.tick() drives one full OODA cycle.
//! ```

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::{Duration, Instant};

use mechos_kernel::{CapabilityManager, KernelGate, ManualOverrideInterlock, StateVerifier};
use mechos_memory::episodic::EpisodicStore;
use mechos_middleware::EventBus;
use mechos_perception::fusion::{FusedState, ImuData, OdometryData, SensorFusion};
use mechos_perception::octree::{Aabb, Octree, Point3};
use mechos_types::{Capability, Event, EventPayload, HardwareIntent, MechError};
use tokio::sync::broadcast;
use tracing::{debug, info, warn};
use uuid::Uuid;

use crate::llm_driver::{ChatMessage, LlmDriver, Role};
use crate::loop_guard::LoopGuard;

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

/// How long the AI is suspended after the most recent manual-override command.
const OVERRIDE_SUSPENSION_DURATION: Duration = Duration::from_secs(10);

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration bundle for [`AgentLoop`].
pub struct AgentLoopConfig {
    /// Base URL of the Ollama / OpenAI-compatible model server.
    pub llm_base_url: String,
    /// Model name to use for inference.
    pub llm_model: String,
    /// Number of consecutive identical LLM outputs that trigger a loop fault.
    pub loop_guard_threshold: usize,
    /// Capability grants to issue to the `"agent"` identity at startup.
    pub capabilities: Vec<Capability>,
    /// Optional path to the persistent episodic memory database.
    /// If `None`, an in-memory store is used (data lost on exit).
    pub persistence_path: Option<String>,
}

impl Default for AgentLoopConfig {
    fn default() -> Self {
        Self {
            llm_base_url: "http://localhost:11434".to_string(),
            llm_model: "llama3".to_string(),
            loop_guard_threshold: 3,
            capabilities: vec![
                Capability::HardwareInvoke("end_effector".to_string()),
                Capability::HardwareInvoke("drive_base".to_string()),
                Capability::HardwareInvoke("hitl".to_string()),
            ],
            persistence_path: None,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// AgentLoop
// ─────────────────────────────────────────────────────────────────────────────

/// The OODA orchestrator.
///
/// Owns all subsystem handles needed to run one full Observe–Orient–Decide–
/// Act–Gatekeep cycle.  Call [`AgentLoop::tick`] from an event loop or async
/// task to advance the agent by one step.
pub struct AgentLoop {
    llm: LlmDriver,
    fusion: SensorFusion,
    octree: Octree,
    memory: EpisodicStore,
    bus: EventBus,
    gate: KernelGate,
    loop_guard: LoopGuard,
    // ── HITL state ────────────────────────────────────────────────────────────
    /// `true` after the LLM has issued an `AskHuman` intent and before the
    /// human operator's response has been consumed.
    waiting_for_human: bool,
    /// The human operator's answer, ready to be injected into the next tick.
    pending_human_response: Option<String>,
    // ── Manual override state ─────────────────────────────────────────────────
    /// Shared flag that is `true` while the dashboard manual-override joystick
    /// is held.  Also registered in the [`StateVerifier`] as a
    /// [`ManualOverrideInterlock`] so AI `Drive` commands are automatically
    /// rejected while the human has control.
    override_active: Arc<AtomicBool>,
    /// Wall-clock time of the most recent manual-override drive command.
    override_last_seen: Option<Instant>,
    // ── Cockpit pause/resume state ────────────────────────────────────────────
    /// `true` when the Cockpit operator has explicitly paused the autonomous
    /// OODA cycle via the mode-toggle button.  Independent of the joystick
    /// override interlock.
    paused: bool,
    /// Non-blocking bus subscriber used to pick up human responses and
    /// dashboard-override events that arrive between ticks.
    bus_rx: broadcast::Receiver<Event>,
}

impl AgentLoop {
    /// Construct a new [`AgentLoop`] from the supplied configuration.
    ///
    /// # Errors
    ///
    /// Returns [`MechError::Serialization`] if the in-memory episodic store
    /// cannot be initialised (e.g. SQLite is unavailable).
    pub fn new(config: AgentLoopConfig) -> Result<Self, MechError> {
        let llm = LlmDriver::new(&config.llm_base_url, &config.llm_model);

        // Sensor fusion with a strong IMU weight.
        let fusion = SensorFusion::new(0.98);

        // Default world bounds: 20 m cube centred at origin, max 8 points per node.
        let world_bounds = Aabb::new(
            Point3::new(-10.0, -10.0, -10.0),
            Point3::new(10.0, 10.0, 10.0),
        );
        let octree = Octree::new(world_bounds, 8);

        // Open persistent store if configured, otherwise fallback to in-memory.
        let memory = if let Some(path) = &config.persistence_path {
            info!("opening persistent episodic store at: {}", path);
            EpisodicStore::open(path).map_err(|e| {
                MechError::Serialization(format!("failed to open episodic store at {path}: {e}"))
            })?
        } else {
            warn!("no persistence_path configured; using in-memory episodic store (data will be lost)");
            EpisodicStore::open_in_memory().map_err(|e| {
                MechError::Serialization(format!("failed to open in-memory episodic store: {e}"))
            })?
        };

        let bus = EventBus::default();

        // Subscribe to the bus for HITL responses and override events.
        let bus_rx = bus.subscribe();

        // Shared override flag – registered in the StateVerifier so AI Drive
        // commands are rejected whenever the human has the joystick.
        let override_active = Arc::new(AtomicBool::new(false));

        // Capability manager: grant the agent identity all configured caps.
        let mut caps = CapabilityManager::new();
        for cap in config.capabilities {
            caps.grant("agent", cap);
        }
        let mut verifier = StateVerifier::new();
        verifier.add_rule(Box::new(ManualOverrideInterlock::new(Arc::clone(
            &override_active,
        ))));
        let gate = KernelGate::new(caps, verifier);

        let loop_guard = LoopGuard::new(config.loop_guard_threshold);

        Ok(Self {
            llm,
            fusion,
            octree,
            memory,
            bus,
            gate,
            loop_guard,
            waiting_for_human: false,
            pending_human_response: None,
            override_active,
            override_last_seen: None,
            paused: false,
            bus_rx,
        })
    }

    // -------------------------------------------------------------------------
    // Subsystem accessors (for testing / external wiring)
    // -------------------------------------------------------------------------

    /// Return a clone of the [`EventBus`] so callers can subscribe to intents.
    pub fn bus(&self) -> EventBus {
        self.bus.clone()
    }

    /// Provide a fresh odometry sample to the sensor fusion engine.
    pub fn update_odometry(&mut self, data: OdometryData) {
        self.fusion.update_odometry(data);
    }

    /// Provide a fresh IMU sample to the sensor fusion engine.
    pub fn update_imu(&mut self, data: ImuData) {
        self.fusion.update_imu(data);
    }

    /// Insert a known obstacle point into the collision octree.
    pub fn add_obstacle(&mut self, p: Point3) {
        self.octree.insert(p);
    }

    // -------------------------------------------------------------------------
    // HITL API
    // -------------------------------------------------------------------------

    /// Inject a human operator's response into the OODA loop.
    ///
    /// Call this when the dashboard WebSocket sends a reply to an earlier
    /// [`HardwareIntent::AskHuman`] prompt.  The response is stored and
    /// consumed by the next [`tick`][Self::tick] call: it is prepended to the
    /// LLM context window as a [`Role::User`] message and the OODA cycle
    /// resumes normally.
    pub fn submit_human_response(&mut self, response: impl Into<String>) {
        self.pending_human_response = Some(response.into());
        self.waiting_for_human = false;
    }

    /// `true` if the loop is currently paused waiting for a human response.
    pub fn is_waiting_for_human(&self) -> bool {
        self.waiting_for_human
    }

    // -------------------------------------------------------------------------
    // Manual override API
    // -------------------------------------------------------------------------

    /// Route a dashboard manual-override drive command directly onto the bus,
    /// bypassing the AI gate, and arm the 10-second AI suspension.
    ///
    /// Call this every time the dashboard sends a Twist command tagged
    /// `source: "dashboard_override"`.  The AI suspension is lifted
    /// automatically once [`tick`][Self::tick] runs and finds that more than
    /// [`OVERRIDE_SUSPENSION_DURATION`] has elapsed since the last call.
    pub fn handle_manual_override(&mut self, linear_velocity: f32, angular_velocity: f32) {
        // Arm the interlock so AI Drive commands are rejected.
        self.override_active.store(true, Ordering::Release);
        self.override_last_seen = Some(Instant::now());

        // Publish the override command with a distinct source tag so downstream
        // adapters can route it directly to the HAL.
        let event = Self::build_override_event(linear_velocity, angular_velocity);
        // Best-effort publish – no subscribers is not an error.
        let _ = self.bus.publish(event);
    }

    /// `true` if the AI is currently suspended due to a manual override.
    pub fn is_override_active(&self) -> bool {
        self.override_active.load(Ordering::Acquire)
    }

    // -------------------------------------------------------------------------
    // Cockpit pause/resume API
    // -------------------------------------------------------------------------

    /// Pause or resume the autonomous OODA cycle via the Cockpit mode-toggle.
    ///
    /// When `paused` is `true` subsequent calls to [`tick`][Self::tick] return
    /// early without invoking the LLM.  This is independent of the 10-second
    /// joystick manual-override interlock.
    pub fn set_paused(&mut self, paused: bool) {
        self.paused = paused;
    }

    /// `true` if the Cockpit operator has explicitly paused the autonomous loop.
    pub fn is_paused(&self) -> bool {
        self.paused
    }

    // -------------------------------------------------------------------------
    // OODA tick
    // -------------------------------------------------------------------------

    /// Execute one full OODA cycle.
    ///
    /// # Errors
    ///
    /// Returns `Err` if:
    /// - The Cockpit operator has paused the loop via the mode-toggle.
    /// - A manual override is active (AI suspended for up to 10 s).
    /// - The loop is waiting for a human response to an `AskHuman` intent.
    /// - The LLM response cannot be parsed as a [`HardwareIntent`].
    /// - The [`KernelGate`] rejects the intent.
    /// - The [`LoopGuard`] detects a repetitive hallucination loop.
    pub async fn tick(&mut self, dt: f32) -> Result<HardwareIntent, MechError> {
        // ── Drain pending bus events ───────────────────────────────────────────
        // Pick up any human responses or override notifications that arrived
        // between ticks without blocking.
        self.drain_bus_events();

        // ── Cockpit pause guard ────────────────────────────────────────────────
        if self.paused {
            return Err(MechError::HardwareFault {
                component: "agent_loop".to_string(),
                details: "agent loop paused by operator".to_string(),
            });
        }

        // ── Manual override guard ──────────────────────────────────────────────
        if self.override_active.load(Ordering::Acquire) {
            if let Some(last) = self.override_last_seen {
                if last.elapsed() >= OVERRIDE_SUSPENSION_DURATION {
                    // 10-second window has expired: lift the AI suspension.
                    self.override_active.store(false, Ordering::Release);
                    self.override_last_seen = None;
                } else {
                    return Err(MechError::HardwareFault {
                        component: "agent_loop".to_string(),
                        details: "manual override active; AI suspended".to_string(),
                    });
                }
            }
        }

        // ── HITL: waiting for human response ───────────────────────────────────
        // If the last LLM turn produced an AskHuman intent and no response has
        // arrived yet, pause the loop.
        let extra_user_message: Option<ChatMessage> = if self.waiting_for_human {
            match self.pending_human_response.take() {
                Some(response) => {
                    self.waiting_for_human = false;
                    Some(ChatMessage {
                        role: Role::User,
                        content: response,
                    })
                }
                None => {
                    return Err(MechError::LlmInferenceFailed(
                        "AgentLoop paused: waiting for human response via dashboard".to_string(),
                    ));
                }
            }
        } else {
            None
        };

        // ── 1. Observe ────────────────────────────────────────────────────────
        let state: FusedState = self.fusion.fused_state(dt);

        // Probe a small AABB in front of the robot for collision detection.
        let probe = Aabb::new(
            Point3::new(state.position_x - 0.5, state.position_y - 0.5, -0.5),
            Point3::new(state.position_x + 0.5, state.position_y + 0.5, 0.5),
        );
        let path_clear = !self.octree.query_aabb(&probe);

        // ── 2. Orient ─────────────────────────────────────────────────────────
        // Retrieve the most recent episodic memories as context.
        let memories = self.memory.all_entries().unwrap_or_default();
        let memory_entries: Vec<String> = memories
            .iter()
            .rev()
            .take(3)
            .map(|e| format!("- [{}] {}", e.timestamp.format("%H:%M:%S"), e.summary))
            .collect();
        let memory_context = if memory_entries.is_empty() {
            "(none)".to_string()
        } else {
            memory_entries.join("\n")
        };

        let system_prompt = format!(
            "You are the cognitive brain of a physical robot.\n\
             Output ONLY a single valid JSON object matching the HardwareIntent schema.\n\
             ## System State\n\
             Position: x={:.3}, y={:.3}\n\
             Heading:  {:.3} rad\n\
             Velocity: vx={:.3}, vy={:.3}\n\
             Path: {}\n\
             ## Recent Memories\n{}\n",
            state.position_x,
            state.position_y,
            state.heading_rad,
            state.velocity_x,
            state.velocity_y,
            if path_clear { "CLEAR" } else { "BLOCKED" },
            memory_context,
        );

        let mut messages = vec![
            ChatMessage {
                role: Role::System,
                content: system_prompt,
            },
            ChatMessage {
                role: Role::User,
                content: "What is your next action? Reply with a single HardwareIntent JSON object.".to_string(),
            },
        ];
        // If a human response was pending, inject it as the next user turn so
        // the LLM has the operator's answer in its context window.
        if let Some(human_msg) = extra_user_message {
            messages.push(human_msg);
        }

        // ── 3. Decide ─────────────────────────────────────────────────────────
        let raw = self.llm.complete(&messages).await.map_err(|e| {
            MechError::LlmInferenceFailed(e.to_string())
        })?;

        // Hash the raw response and check for repetitive loops.
        let hash = Self::hash_str(&raw);
        if self.loop_guard.record(&hash.to_string()) {
            warn!("LoopGuard: repetitive LLM output detected; human intervention required");
            return Err(MechError::LlmInferenceFailed(
                "LoopGuard: repetitive LLM output detected; human intervention required"
                    .to_string(),
            ));
        }

        // Parse the JSON response into a HardwareIntent.
        let intent: HardwareIntent =
            serde_json::from_str(&raw).map_err(|e| {
                MechError::LlmInferenceFailed(format!("JSON parse error: {e}"))
            })?;

        debug!(intent = ?intent, "LLM decided intent");

        // ── 4. Gatekeep ───────────────────────────────────────────────────────
        self.gate.authorize_and_verify("agent", &intent)?;

        // ── 5. Act ────────────────────────────────────────────────────────────
        info!(intent = ?intent, "dispatching approved intent");
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: chrono::Utc::now(),
            source: "mechos-runtime::agent_loop".to_string(),
            payload: EventPayload::AgentThought(
                serde_json::to_string(&intent)
                    .unwrap_or_else(|_| "(serialisation error)".to_string()),
            ),
        };
        // Best-effort publish – no subscribers is not an error.
        let _ = self.bus.publish(event);

        // ── 6. HITL bookkeeping ───────────────────────────────────────────────
        // If the LLM asked for human guidance, park the loop until a response
        // arrives via `submit_human_response` or a bus `HumanResponse` event.
        if matches!(intent, HardwareIntent::AskHuman { .. }) {
            self.waiting_for_human = true;
        }

        Ok(intent)
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /// Non-blocking drain of pending bus events.
    ///
    /// Processes every event that is already waiting in the broadcast buffer:
    ///
    /// * [`EventPayload::HumanResponse`] – stores the response so the next
    ///   tick can inject it into the LLM context.
    /// * `source: "mechos-middleware::dashboard_override"` – extracts the
    ///   Twist velocities and arms the manual-override interlock.
    /// * [`EventPayload::AgentModeToggle`] – sets or clears the Cockpit
    ///   pause flag.
    fn drain_bus_events(&mut self) {
        loop {
            match self.bus_rx.try_recv() {
                Ok(event) => {
                    match &event.payload {
                        EventPayload::HumanResponse(response) => {
                            self.pending_human_response = Some(response.clone());
                            self.waiting_for_human = false;
                        }
                        EventPayload::AgentModeToggle { paused } => {
                            self.paused = *paused;
                        }
                        // Ingest LidarScan into the Octree
                        EventPayload::LidarScan {
                            ranges,
                            angle_min_rad,
                            angle_increment_rad,
                        } => {
                            for (i, r) in ranges.iter().enumerate() {
                                // Ignore invalid or out-of-range returns
                                if *r < 0.1 || *r > 20.0 || !r.is_finite() {
                                    continue;
                                }
                                let angle = angle_min_rad + (i as f32 * angle_increment_rad);
                                let x = r * angle.cos();
                                let y = r * angle.sin();
                                // We assume a 2D scan lies on the z=0 plane relative to the sensor frame.
                                // In a real TF tree this would be transformed to the map frame.
                                self.octree.insert(Point3::new(x, y, 0.0));
                            }
                        }
                        EventPayload::AgentThought(json_str)
                            if event.source
                                == "mechos-middleware::dashboard_override" =>
                        {
                            // Extract Twist velocities from the rosbridge JSON.
                            if let Ok(json) =
                                serde_json::from_str::<serde_json::Value>(json_str)
                            {
                                let linear_opt = json["msg"]["linear"]["x"].as_f64();
                                let angular_opt = json["msg"]["angular"]["z"].as_f64();
                                if linear_opt.is_none() || angular_opt.is_none() {
                                    warn!(
                                        "dashboard_override: missing linear.x or angular.z in Twist frame"
                                    );
                                }
                                let linear = linear_opt.unwrap_or(0.0) as f32;
                                let angular = angular_opt.unwrap_or(0.0) as f32;
                                self.override_active.store(true, Ordering::Release);
                                self.override_last_seen = Some(Instant::now());
                                // Re-publish the manual override command with the
                                // kernel source tag so downstream adapters can
                                // route it to the HAL.
                                let fwd = Self::build_override_event(linear, angular);
                                let _ = self.bus.publish(fwd);
                            }
                        }
                        _ => {}
                    }
                }
                Err(broadcast::error::TryRecvError::Empty) => break,
                Err(broadcast::error::TryRecvError::Lagged(_)) => continue,
                Err(broadcast::error::TryRecvError::Closed) => break,
            }
        }
    }

    /// Build an [`Event`] that carries a manual-override Twist command with
    /// the `"mechos-kernel::manual_override"` source tag.
    fn build_override_event(linear_velocity: f32, angular_velocity: f32) -> Event {
        let frame = serde_json::json!({
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear":  { "x": linear_velocity, "y": 0.0, "z": 0.0 },
                "angular": { "x": 0.0, "y": 0.0, "z": angular_velocity }
            }
        })
        .to_string();
        Event {
            id: Uuid::new_v4(),
            timestamp: chrono::Utc::now(),
            source: "mechos-kernel::manual_override".to_string(),
            payload: EventPayload::AgentThought(frame),
        }
    }

    fn hash_str(s: &str) -> u64 {
        let mut h = DefaultHasher::new();
        s.hash(&mut h);
        h.finish()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_agent() -> AgentLoop {
        AgentLoop::new(AgentLoopConfig::default()).expect("AgentLoop::new should not fail in tests")
    }

    #[test]
    fn agent_loop_constructs_without_panic() {
        let _agent = default_agent();
    }

    #[test]
    fn bus_clone_is_accessible() {
        let agent = default_agent();
        let _bus = agent.bus();
    }

    #[test]
    fn update_odometry_does_not_panic() {
        let mut agent = default_agent();
        agent.update_odometry(OdometryData {
            position_x: 1.0,
            position_y: 2.0,
            heading_rad: 0.5,
            velocity_x: 0.1,
            velocity_y: 0.0,
        });
    }

    #[test]
    fn add_obstacle_does_not_panic() {
        let mut agent = default_agent();
        agent.add_obstacle(Point3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn hash_str_is_deterministic() {
        let h1 = AgentLoop::hash_str("hello");
        let h2 = AgentLoop::hash_str("hello");
        assert_eq!(h1, h2);
    }

    #[test]
    fn hash_str_differs_for_different_inputs() {
        let h1 = AgentLoop::hash_str("action_a");
        let h2 = AgentLoop::hash_str("action_b");
        assert_ne!(h1, h2);
    }

    #[tokio::test]
    async fn tick_returns_llm_error_when_server_unavailable() {
        // No live LLM server – tick must return LlmInferenceFailed, not panic.
        let mut agent = default_agent();
        let result = agent.tick(0.1).await;
        assert!(matches!(result, Err(MechError::LlmInferenceFailed(_))));
    }

    // ── HITL tests ────────────────────────────────────────────────────────────

    #[test]
    fn initial_state_not_waiting_for_human() {
        let agent = default_agent();
        assert!(!agent.is_waiting_for_human());
    }

    #[test]
    fn submit_human_response_clears_waiting_state() {
        let mut agent = default_agent();
        agent.waiting_for_human = true;
        agent.submit_human_response("Yes, proceed");
        assert!(!agent.is_waiting_for_human());
        assert_eq!(agent.pending_human_response.as_deref(), Some("Yes, proceed"));
    }

    #[tokio::test]
    async fn tick_pauses_when_waiting_for_human_with_no_response() {
        let mut agent = default_agent();
        agent.waiting_for_human = true;
        // No pending response – tick must pause.
        let result = agent.tick(0.1).await;
        assert!(
            matches!(&result, Err(MechError::LlmInferenceFailed(msg)) if msg.contains("waiting for human")),
            "expected waiting-for-human pause, got: {result:?}"
        );
        // Still waiting after the pause.
        assert!(agent.is_waiting_for_human());
    }

    #[tokio::test]
    async fn tick_resumes_when_human_response_is_available() {
        // inject a pending response; tick should proceed to LLM (which will
        // fail because no server is running, but not with the "waiting" error).
        let mut agent = default_agent();
        agent.waiting_for_human = true;
        agent.pending_human_response = Some("Yes, push it".to_string());
        let result = agent.tick(0.1).await;
        // The "waiting" state should have been cleared.
        assert!(!agent.is_waiting_for_human());
        // With no LLM, we expect LlmInferenceFailed (not the waiting error).
        assert!(
            matches!(&result, Err(MechError::LlmInferenceFailed(msg)) if !msg.contains("waiting for human")),
            "expected LLM error after resume, got: {result:?}"
        );
    }

    // ── Manual override tests ─────────────────────────────────────────────────

    #[test]
    fn initial_state_override_not_active() {
        let agent = default_agent();
        assert!(!agent.is_override_active());
    }

    #[test]
    fn handle_manual_override_arms_interlock() {
        let mut agent = default_agent();
        agent.handle_manual_override(0.5, -0.2);
        assert!(agent.is_override_active());
    }

    #[tokio::test]
    async fn tick_returns_hardware_fault_when_override_active() {
        let mut agent = default_agent();
        agent.handle_manual_override(0.5, 0.0);
        let result = agent.tick(0.1).await;
        assert!(
            matches!(
                &result,
                Err(MechError::HardwareFault { component, .. })
                    if component == "agent_loop"
            ),
            "expected override HardwareFault, got: {result:?}"
        );
    }

    #[tokio::test]
    async fn override_lifts_after_suspension_duration_elapses() {
        let mut agent = default_agent();
        agent.handle_manual_override(0.5, 0.0);
        // Backdating the last-seen timestamp simulates the 10-s window expiring.
        agent.override_last_seen =
            Some(Instant::now() - OVERRIDE_SUSPENSION_DURATION - Duration::from_millis(1));
        let result = agent.tick(0.1).await;
        // Override should be cleared.
        assert!(!agent.is_override_active());
        // tick should proceed to the LLM (which is unavailable, so LlmInferenceFailed).
        assert!(matches!(result, Err(MechError::LlmInferenceFailed(_))));
    }

    #[test]
    fn handle_manual_override_publishes_kernel_event_to_bus() {
        let mut agent = default_agent();
        let mut rx = agent.bus().subscribe();
        agent.handle_manual_override(1.0, -0.5);
        let event = rx.try_recv().expect("event should be published");
        assert_eq!(event.source, "mechos-kernel::manual_override");
        if let EventPayload::AgentThought(json_str) = event.payload {
            assert!(json_str.contains("/cmd_vel"));
        } else {
            panic!("expected AgentThought");
        }
    }

    #[test]
    fn drain_bus_events_picks_up_human_response() {
        let mut agent = default_agent();
        // Publish a HumanResponse event directly onto the agent's bus.
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: chrono::Utc::now(),
            source: "mechos-middleware::dashboard/human_response".to_string(),
            payload: EventPayload::HumanResponse("Yes, go ahead".to_string()),
        };
        let _ = agent.bus.publish(event);
        agent.drain_bus_events();
        assert_eq!(
            agent.pending_human_response.as_deref(),
            Some("Yes, go ahead")
        );
    }

    #[test]
    fn drain_bus_events_picks_up_dashboard_override() {
        let mut agent = default_agent();
        let override_json = r#"{"op":"publish","topic":"/cmd_vel","msg":{"linear":{"x":0.8,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.3}},"source":"dashboard_override"}"#;
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: chrono::Utc::now(),
            source: "mechos-middleware::dashboard_override".to_string(),
            payload: EventPayload::AgentThought(override_json.to_string()),
        };
        let _ = agent.bus.publish(event);
        agent.drain_bus_events();
        assert!(agent.is_override_active());
    }

    // ── Cockpit pause/resume tests ────────────────────────────────────────────

    #[test]
    fn initial_state_not_paused() {
        let agent = default_agent();
        assert!(!agent.is_paused());
    }

    #[test]
    fn set_paused_true_pauses_loop() {
        let mut agent = default_agent();
        agent.set_paused(true);
        assert!(agent.is_paused());
    }

    #[test]
    fn set_paused_false_resumes_loop() {
        let mut agent = default_agent();
        agent.set_paused(true);
        agent.set_paused(false);
        assert!(!agent.is_paused());
    }

    #[tokio::test]
    async fn tick_returns_hardware_fault_when_paused() {
        let mut agent = default_agent();
        agent.set_paused(true);
        let result = agent.tick(0.1).await;
        assert!(
            matches!(
                &result,
                Err(MechError::HardwareFault { component, details })
                    if component == "agent_loop" && details.contains("paused")
            ),
            "expected paused HardwareFault, got: {result:?}"
        );
    }

    #[test]
    fn drain_bus_events_picks_up_agent_mode_toggle_pause() {
        let mut agent = default_agent();
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: chrono::Utc::now(),
            source: "mechos-cockpit::server".to_string(),
            payload: EventPayload::AgentModeToggle { paused: true },
        };
        let _ = agent.bus.publish(event);
        agent.drain_bus_events();
        assert!(agent.is_paused());
    }

    #[test]
    fn drain_bus_events_picks_up_agent_mode_toggle_resume() {
        let mut agent = default_agent();
        agent.set_paused(true);
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: chrono::Utc::now(),
            source: "mechos-cockpit::server".to_string(),
            payload: EventPayload::AgentModeToggle { paused: false },
        };
        let _ = agent.bus.publish(event);
        agent.drain_bus_events();
        assert!(!agent.is_paused());
    }
}
