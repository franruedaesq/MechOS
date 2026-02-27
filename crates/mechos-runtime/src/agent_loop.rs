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
//! # Example
//!
//! ```rust,no_run
//! use mechos_runtime::agent_loop::{AgentLoop, AgentLoopConfig};
//!
//! // Build the loop with sensible defaults and run one tick.
//! let config = AgentLoopConfig::default();
//! let mut agent = AgentLoop::new(config);
//! // agent.tick() drives one full OODA cycle.
//! ```

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use mechos_kernel::{CapabilityManager, KernelGate, StateVerifier};
use mechos_memory::episodic::EpisodicStore;
use mechos_middleware::EventBus;
use mechos_perception::fusion::{FusedState, ImuData, OdometryData, SensorFusion};
use mechos_perception::octree::{Aabb, Octree, Point3};
use mechos_types::{Capability, Event, EventPayload, HardwareIntent, MechError};
use uuid::Uuid;

use crate::llm_driver::{ChatMessage, LlmDriver, Role};
use crate::loop_guard::LoopGuard;

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
}

impl AgentLoop {
    /// Construct a new [`AgentLoop`] from the supplied configuration.
    pub fn new(config: AgentLoopConfig) -> Self {
        let llm = LlmDriver::new(&config.llm_base_url, &config.llm_model);

        // Sensor fusion with a strong IMU weight.
        let fusion = SensorFusion::new(0.98);

        // Default world bounds: 20 m cube centred at origin, max 8 points per node.
        let world_bounds = Aabb::new(
            Point3::new(-10.0, -10.0, -10.0),
            Point3::new(10.0, 10.0, 10.0),
        );
        let octree = Octree::new(world_bounds, 8);

        // In-memory episodic store (no persistence path configured).
        let memory = EpisodicStore::open_in_memory()
            .expect("failed to open in-memory episodic store");

        let bus = EventBus::default();

        // Capability manager: grant the agent identity all configured caps.
        let mut caps = CapabilityManager::new();
        for cap in config.capabilities {
            caps.grant("agent", cap);
        }
        let verifier = StateVerifier::new();
        let gate = KernelGate::new(caps, verifier);

        let loop_guard = LoopGuard::new(config.loop_guard_threshold);

        Self {
            llm,
            fusion,
            octree,
            memory,
            bus,
            gate,
            loop_guard,
        }
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
    // OODA tick
    // -------------------------------------------------------------------------

    /// Execute one full OODA cycle.
    ///
    /// # Errors
    ///
    /// Returns `Err` if:
    /// - The LLM response cannot be parsed as a [`HardwareIntent`].
    /// - The [`KernelGate`] rejects the intent.
    /// - The [`LoopGuard`] detects a repetitive hallucination loop.
    pub fn tick(&mut self, dt: f32) -> Result<HardwareIntent, MechError> {
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

        let messages = vec![
            ChatMessage {
                role: Role::System,
                content: system_prompt,
            },
            ChatMessage {
                role: Role::User,
                content: "What is your next action? Reply with a single HardwareIntent JSON object.".to_string(),
            },
        ];

        // ── 3. Decide ─────────────────────────────────────────────────────────
        let raw = self.llm.complete(&messages).map_err(|e| {
            MechError::LlmInferenceFailed(e.to_string())
        })?;

        // Hash the raw response and check for repetitive loops.
        let hash = Self::hash_str(&raw);
        if self.loop_guard.record(&hash.to_string()) {
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

        // ── 4. Gatekeep ───────────────────────────────────────────────────────
        self.gate.authorize_and_verify("agent", &intent)?;

        // ── 5. Act ────────────────────────────────────────────────────────────
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

        Ok(intent)
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

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
        AgentLoop::new(AgentLoopConfig::default())
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

    #[test]
    fn tick_returns_llm_error_when_server_unavailable() {
        // No live LLM server – tick must return LlmInferenceFailed, not panic.
        let mut agent = default_agent();
        let result = agent.tick(0.1);
        assert!(matches!(result, Err(MechError::LlmInferenceFailed(_))));
    }
}
