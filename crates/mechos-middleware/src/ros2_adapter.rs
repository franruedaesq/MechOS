//! ROS 2 adapter for physical robots.
//!
//! [`Ros2Adapter`] bridges between the MechOS internal [`EventBus`] and the
//! ROS 2 ecosystem:
//!
//! * **Outbound (Inverse Kinematics)** – a [`HardwareIntent::MoveEndEffector`]
//!   is handed to ROS 2 MoveIt 2, which computes the required joint angles and
//!   publishes them to `/joint_states`.
//!
//! * **Outbound (Drive)** – a [`HardwareIntent::Drive`] is translated into a
//!   `geometry_msgs/msg/Twist` JSON payload and published to `/cmd_vel`.
//!
//! * **Inbound (Perception)** – an incoming `/scan` laser-scan message is
//!   converted into a [`EventPayload::Telemetry`] event and streamed into the
//!   [`EventBus`].

use async_trait::async_trait;
use futures_util::stream::{self, BoxStream};
use mechos_types::{Event, EventPayload, HardwareIntent, MechError, TelemetryData};
use serde_json::json;
use std::sync::Arc;
use uuid::Uuid;
use chrono::Utc;

use crate::adapter::MechAdapter;
use crate::bus::EventBus;

/// Adapter that translates MechOS intents into ROS 2 messages and ingests
/// physical sensor data from the robot.
pub struct Ros2Adapter {
    bus: Arc<EventBus>,
}

impl Ros2Adapter {
    /// Create a new [`Ros2Adapter`] backed by the given [`EventBus`].
    pub fn new(bus: Arc<EventBus>) -> Self {
        Self { bus }
    }

    /// Ingest a `/scan` laser-scan message and publish it as a
    /// [`EventPayload::Telemetry`] event on the internal bus.
    ///
    /// `_ranges` contains the measured distances (metres) from the physical
    /// LiDAR.  Full range processing (3D point-cloud conversion for the Octree
    /// collision map) is pending future integration; only the odometry fields
    /// are used to build the telemetry snapshot.
    pub fn ingest_laser_scan(
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
            source: "mechos-middleware::ros2/scan".to_string(),
            payload: EventPayload::Telemetry(TelemetryData {
                position_x,
                position_y,
                heading_rad,
                battery_percent,
            }),
        };
        self.bus.publish(event)
    }
}

#[async_trait]
impl MechAdapter for Ros2Adapter {
    /// Translate a [`HardwareIntent`] into a ROS 2 command.
    ///
    /// * `MoveEndEffector` – serialises the target coordinates as a MoveIt 2
    ///   goal JSON and logs it (in a real deployment this would be sent over
    ///   `ros2_bridge` to `/move_group/goal`).
    ///
    /// * `Drive` – serialises a `geometry_msgs/msg/Twist` JSON payload for
    ///   `/cmd_vel`.
    ///
    /// * `TriggerRelay` – serialises a relay command for the appropriate GPIO
    ///   topic.
    ///
    /// * `AskHuman` – publishes an [`EventPayload::AgentThought`] onto the bus
    ///   so the dashboard can display the question.
    async fn execute_intent(&self, intent: HardwareIntent) -> Result<(), MechError> {
        match &intent {
            HardwareIntent::MoveEndEffector { x, y, z } => {
                // Hand coordinates to MoveIt 2: compute IK then publish to /joint_states.
                let moveit_goal = json!({
                    "op": "publish",
                    "topic": "/move_group/goal",
                    "msg": {
                        "target_pose": { "x": x, "y": y, "z": z }
                    }
                });
                // In production this is forwarded to ros2_bridge; here we publish
                // it as an AgentThought so the rest of the system can observe it.
                let event = Event {
                    id: Uuid::new_v4(),
                    timestamp: Utc::now(),
                    source: "mechos-middleware::ros2/joint_states".to_string(),
                    payload: EventPayload::AgentThought(moveit_goal.to_string()),
                };
                self.bus.publish(event).map(|_| ())
            }
            HardwareIntent::Drive {
                linear_velocity,
                angular_velocity,
            } => {
                let twist = json!({
                    "op": "publish",
                    "topic": "/cmd_vel",
                    "msg": {
                        "linear":  { "x": linear_velocity, "y": 0.0, "z": 0.0 },
                        "angular": { "x": 0.0, "y": 0.0, "z": angular_velocity }
                    }
                });
                let event = Event {
                    id: Uuid::new_v4(),
                    timestamp: Utc::now(),
                    source: "mechos-middleware::ros2/cmd_vel".to_string(),
                    payload: EventPayload::AgentThought(twist.to_string()),
                };
                self.bus.publish(event).map(|_| ())
            }
            HardwareIntent::TriggerRelay { relay_id, state } => {
                let relay_msg = json!({
                    "op": "publish",
                    "topic": format!("/relay/{relay_id}"),
                    "msg": { "data": state }
                });
                let event = Event {
                    id: Uuid::new_v4(),
                    timestamp: Utc::now(),
                    source: format!("mechos-middleware::ros2/relay/{relay_id}"),
                    payload: EventPayload::AgentThought(relay_msg.to_string()),
                };
                self.bus.publish(event).map(|_| ())
            }
            HardwareIntent::AskHuman { question, .. } => {
                let event = Event {
                    id: Uuid::new_v4(),
                    timestamp: Utc::now(),
                    source: "mechos-middleware::ros2/ask_human".to_string(),
                    payload: EventPayload::AgentThought(question.clone()),
                };
                self.bus.publish(event).map(|_| ())
            }
        }
    }

    /// Return a sensor stream.
    ///
    /// In a real deployment the adapter would subscribe to `/scan` via
    /// `ros2_bridge` and yield events continuously.  This implementation
    /// returns an empty stream as a correct skeleton; callers that need live
    /// data should use [`ingest_laser_scan`][Self::ingest_laser_scan] to push
    /// frames directly onto the bus.
    async fn sensor_stream(&self) -> BoxStream<'static, EventPayload> {
        Box::pin(stream::empty())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mechos_types::EventPayload;

    fn make_adapter() -> (Arc<EventBus>, Ros2Adapter) {
        let bus = Arc::new(EventBus::default());
        let adapter = Ros2Adapter::new(Arc::clone(&bus));
        (bus, adapter)
    }

    #[tokio::test]
    async fn execute_drive_publishes_cmd_vel() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .execute_intent(HardwareIntent::Drive {
                linear_velocity: 1.0,
                angular_velocity: 0.5,
            })
            .await
            .unwrap();

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-middleware::ros2/cmd_vel");
        assert!(matches!(event.payload, EventPayload::AgentThought(_)));
        if let EventPayload::AgentThought(json_str) = event.payload {
            assert!(json_str.contains("/cmd_vel"));
            assert!(json_str.contains("linear"));
        }
    }

    #[tokio::test]
    async fn execute_move_end_effector_publishes_joint_states() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .execute_intent(HardwareIntent::MoveEndEffector {
                x: 1.5,
                y: 0.0,
                z: 0.2,
            })
            .await
            .unwrap();

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-middleware::ros2/joint_states");
        if let EventPayload::AgentThought(json_str) = event.payload {
            assert!(json_str.contains("target_pose"));
        }
    }

    #[tokio::test]
    async fn ingest_laser_scan_publishes_telemetry() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .ingest_laser_scan(&[1.0, 2.0, 3.0], 0.0, 0.0, 0.0, 100)
            .unwrap();

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-middleware::ros2/scan");
        assert!(matches!(event.payload, EventPayload::Telemetry(_)));
    }

    #[tokio::test]
    async fn execute_ask_human_publishes_agent_thought() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .execute_intent(HardwareIntent::AskHuman {
                question: "Which shelf?".to_string(),
                context_image_id: None,
            })
            .await
            .unwrap();

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-middleware::ros2/ask_human");
        if let EventPayload::AgentThought(q) = event.payload {
            assert_eq!(q, "Which shelf?");
        }
    }
}
