//! Dashboard simulation adapter.
//!
//! [`DashboardSimAdapter`] bridges between the MechOS internal [`EventBus`]
//! and the React / Three.js simulation UI via a `rosbridge_server`-compatible
//! WebSocket:
//!
//! * **Outbound (Simulated Movement)** – a [`HardwareIntent::Drive`] is
//!   translated into a `geometry_msgs/msg/Twist` JSON payload and pushed over
//!   the WebSocket to the dashboard's `rosbridge_server`.
//!
//! * **Inbound (Simulated LiDAR)** – `/sim_scan` messages from the dashboard
//!   (packed `sensor_msgs/msg/LaserScan` arrays produced by virtual raycasts)
//!   are parsed and fed into the [`EventBus`] as [`EventPayload::Telemetry`].

use async_trait::async_trait;
use futures_util::stream::{self, BoxStream};
use mechos_types::{Event, EventPayload, HardwareIntent, MechError, TelemetryData};
use serde_json::json;
use std::sync::Arc;
use uuid::Uuid;
use chrono::Utc;

use crate::adapter::MechAdapter;
use crate::bus::EventBus;

/// Adapter that communicates with the React / Three.js simulation dashboard
/// over a `rosbridge_server`-compatible WebSocket.
pub struct DashboardSimAdapter {
    bus: Arc<EventBus>,
    /// `ws://host:port` of the dashboard's rosbridge endpoint.
    rosbridge_url: String,
}

impl DashboardSimAdapter {
    /// Create a new [`DashboardSimAdapter`].
    ///
    /// `rosbridge_url` should be the WebSocket URL of the dashboard's
    /// `rosbridge_server` (e.g. `"ws://localhost:9090"`).
    pub fn new(bus: Arc<EventBus>, rosbridge_url: impl Into<String>) -> Self {
        Self {
            bus,
            rosbridge_url: rosbridge_url.into(),
        }
    }

    /// Return the rosbridge URL this adapter is configured to use.
    pub fn rosbridge_url(&self) -> &str {
        &self.rosbridge_url
    }

    /// Ingest a `sensor_msgs/msg/LaserScan` message received from the
    /// dashboard's `/sim_scan` topic and publish it as a
    /// [`EventPayload::Telemetry`] event on the internal bus.
    ///
    /// `_ranges` contains virtual distances (metres) produced by the
    /// dashboard's Three.js raycasts.  Full range processing (Octree collision
    /// map updates) is pending future integration; only the odometry fields
    /// are used to build the telemetry snapshot.
    pub fn ingest_sim_scan(
        &self,
        _ranges: &[f32],
        position_x: f32,
        position_y: f32,
        heading_rad: f32,
        battery_percent: u8,
    ) -> Result<usize, MechError> {
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: "mechos-middleware::dashboard/sim_scan".to_string(),
            payload: EventPayload::Telemetry(TelemetryData {
                position_x,
                position_y,
                heading_rad,
                battery_percent,
            }),
        };
        self.bus.publish(event)
    }

    /// Ingest a human operator's response to an [`HardwareIntent::AskHuman`]
    /// prompt that was previously pushed to the dashboard.
    ///
    /// Call this when the dashboard WebSocket sends a message on the
    /// `/hitl/human_response` topic.  The response is published as a
    /// [`EventPayload::HumanResponse`] event so that the [`AgentLoop`] can
    /// inject it into the LLM context window and resume the OODA cycle.
    ///
    /// [`AgentLoop`]: mechos_runtime::AgentLoop
    pub fn ingest_human_response(
        &self,
        response: impl Into<String>,
    ) -> Result<usize, MechError> {
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: "mechos-middleware::dashboard/human_response".to_string(),
            payload: EventPayload::HumanResponse(response.into()),
        };
        self.bus.publish(event)
    }

    /// Build the `rosbridge_server` JSON frame for an `AskHuman` intent.
    ///
    /// Returns a serialised publish command that the dashboard can display as a
    /// UI alert.  The human operator's answer should be sent back on the
    /// `/hitl/human_response` topic.
    pub fn build_ask_human_frame(question: &str, context_image_id: Option<&str>) -> String {
        json!({
            "op": "publish",
            "topic": "/hitl/ask_human",
            "msg": {
                "question": question,
                "context_image_id": context_image_id
            }
        })
        .to_string()
    }

    /// Build the `rosbridge_server` JSON frame for a `Drive` intent.
    ///
    /// Returns the serialised `geometry_msgs/msg/Twist` publish command that
    /// should be forwarded to the dashboard WebSocket.
    pub fn build_twist_frame(linear_velocity: f32, angular_velocity: f32) -> String {
        json!({
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear":  { "x": linear_velocity, "y": 0.0, "z": 0.0 },
                "angular": { "x": 0.0, "y": 0.0, "z": angular_velocity }
            }
        })
        .to_string()
    }
}

#[async_trait]
impl MechAdapter for DashboardSimAdapter {
    /// Translate a [`HardwareIntent`] into a simulated dashboard command.
    ///
    /// * `Drive` – serialises a `geometry_msgs/msg/Twist` JSON frame and
    ///   publishes it onto the bus as an [`EventPayload::AgentThought`].  In a
    ///   real deployment the frame would be forwarded to the dashboard's
    ///   `rosbridge_server` WebSocket; the Three.js / Rapier physics engine
    ///   then moves the virtual robot.
    ///
    /// * All other intents – publish an [`EventPayload::AgentThought`]
    ///   containing a JSON-encoded description so the dashboard can display or
    ///   log the intent.
    async fn execute_intent(&self, intent: HardwareIntent) -> Result<(), MechError> {
        match &intent {
            HardwareIntent::Drive {
                linear_velocity,
                angular_velocity,
            } => {
                let frame = Self::build_twist_frame(*linear_velocity, *angular_velocity);
                let event = Event {
                    id: Uuid::new_v4(),
                    timestamp: Utc::now(),
                    source: "mechos-middleware::dashboard/cmd_vel".to_string(),
                    payload: EventPayload::AgentThought(frame),
                };
                self.bus.publish(event).map(|_| ())
            }
            HardwareIntent::MoveEndEffector { x, y, z } => {
                let msg = json!({
                    "op": "publish",
                    "topic": "/sim/end_effector",
                    "msg": { "x": x, "y": y, "z": z }
                });
                let event = Event {
                    id: Uuid::new_v4(),
                    timestamp: Utc::now(),
                    source: "mechos-middleware::dashboard/end_effector".to_string(),
                    payload: EventPayload::AgentThought(msg.to_string()),
                };
                self.bus.publish(event).map(|_| ())
            }
            HardwareIntent::TriggerRelay { relay_id, state } => {
                let msg = json!({
                    "op": "publish",
                    "topic": format!("/sim/relay/{relay_id}"),
                    "msg": { "data": state }
                });
                let event = Event {
                    id: Uuid::new_v4(),
                    timestamp: Utc::now(),
                    source: format!("mechos-middleware::dashboard/relay/{relay_id}"),
                    payload: EventPayload::AgentThought(msg.to_string()),
                };
                self.bus.publish(event).map(|_| ())
            }
            HardwareIntent::AskHuman { question, context_image_id } => {
                let frame = Self::build_ask_human_frame(
                    question,
                    context_image_id.as_deref(),
                );
                let event = Event {
                    id: Uuid::new_v4(),
                    timestamp: Utc::now(),
                    source: "mechos-middleware::dashboard/ask_human".to_string(),
                    payload: EventPayload::AgentThought(frame),
                };
                self.bus.publish(event).map(|_| ())
            }
        }
    }

    /// Return a simulated sensor stream.
    ///
    /// In a real deployment this adapter would connect to the dashboard's
    /// `/sim_scan` WebSocket topic and yield events produced by Three.js
    /// raycasts.  This implementation returns an empty stream as a correct
    /// skeleton; callers that need live data should use
    /// [`ingest_sim_scan`][Self::ingest_sim_scan] to push frames directly onto
    /// the bus.
    async fn sensor_stream(&self) -> BoxStream<'static, EventPayload> {
        Box::pin(stream::empty())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mechos_types::EventPayload;

    fn make_adapter() -> (Arc<EventBus>, DashboardSimAdapter) {
        let bus = Arc::new(EventBus::default());
        let adapter = DashboardSimAdapter::new(Arc::clone(&bus), "ws://localhost:9090");
        (bus, adapter)
    }

    #[tokio::test]
    async fn execute_drive_publishes_twist_to_bus() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .execute_intent(HardwareIntent::Drive {
                linear_velocity: 0.5,
                angular_velocity: -0.2,
            })
            .await
            .unwrap();

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-middleware::dashboard/cmd_vel");
        if let EventPayload::AgentThought(json_str) = event.payload {
            assert!(json_str.contains("/cmd_vel"));
            assert!(json_str.contains("linear"));
            assert!(json_str.contains("angular"));
        } else {
            panic!("expected AgentThought");
        }
    }

    #[tokio::test]
    async fn ingest_sim_scan_publishes_telemetry() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .ingest_sim_scan(&[0.5, 1.0, 1.5], 1.0, 2.0, 0.3, 75)
            .unwrap();

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-middleware::dashboard/sim_scan");
        assert!(matches!(event.payload, EventPayload::Telemetry(_)));
        if let EventPayload::Telemetry(t) = event.payload {
            assert!((t.position_x - 1.0).abs() < f32::EPSILON);
            assert!((t.position_y - 2.0).abs() < f32::EPSILON);
            assert_eq!(t.battery_percent, 75);
        }
    }

    #[test]
    fn rosbridge_url_stored_correctly() {
        let bus = Arc::new(EventBus::default());
        let adapter = DashboardSimAdapter::new(Arc::clone(&bus), "ws://robot.local:9090");
        assert_eq!(adapter.rosbridge_url(), "ws://robot.local:9090");
    }

    #[test]
    fn build_twist_frame_contains_expected_fields() {
        let frame = DashboardSimAdapter::build_twist_frame(1.0, -0.5);
        assert!(frame.contains("/cmd_vel"));
        assert!(frame.contains("linear"));
        assert!(frame.contains("angular"));
    }

    #[tokio::test]
    async fn execute_ask_human_publishes_agent_thought() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .execute_intent(HardwareIntent::AskHuman {
                question: "Ready to proceed?".to_string(),
                context_image_id: None,
            })
            .await
            .unwrap();

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-middleware::dashboard/ask_human");
        if let EventPayload::AgentThought(json_str) = event.payload {
            assert!(json_str.contains("/hitl/ask_human"), "must target the HITL topic");
            assert!(json_str.contains("Ready to proceed?"));
        } else {
            panic!("expected AgentThought");
        }
    }

    #[test]
    fn build_ask_human_frame_contains_expected_fields() {
        let frame = DashboardSimAdapter::build_ask_human_frame(
            "Should I push the box?",
            Some("frame_042"),
        );
        assert!(frame.contains("/hitl/ask_human"));
        assert!(frame.contains("Should I push the box?"));
        assert!(frame.contains("frame_042"));
    }

    #[test]
    fn build_ask_human_frame_no_image() {
        let frame = DashboardSimAdapter::build_ask_human_frame("Proceed?", None);
        assert!(frame.contains("/hitl/ask_human"));
        assert!(frame.contains("Proceed?"));
    }

    #[tokio::test]
    async fn ingest_human_response_publishes_human_response_event() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter.ingest_human_response("Yes, push it").unwrap();

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-middleware::dashboard/human_response");
        if let EventPayload::HumanResponse(resp) = event.payload {
            assert_eq!(resp, "Yes, push it");
        } else {
            panic!("expected HumanResponse");
        }
    }
}
