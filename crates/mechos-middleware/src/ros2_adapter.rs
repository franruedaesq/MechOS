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

/// Maximum number of LiDAR range readings accepted in a single scan.
///
/// Payloads with more entries than this are rejected to prevent memory
/// exhaustion from malformed or malicious scan messages.
pub const MAX_LIDAR_RANGES: usize = 4096;

/// Maximum byte length of a fleet peer message.
///
/// Messages longer than this are rejected before they are broadcast on the
/// internal event bus.
pub const MAX_FLEET_MESSAGE_BYTES: usize = 64 * 1024; // 64 KiB

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

    /// Ingest a `/scan` laser-scan message, publish it as a
    /// [`EventPayload::Telemetry`] event with odometry data, and also publish
    /// a [`EventPayload::LidarScan`] event so the [`AgentLoop`] can feed the
    /// range data into its collision octree.
    ///
    /// `ranges` contains the measured distances (metres) from the physical
    /// LiDAR.  `angle_min_rad` is the bearing of the first range reading
    /// (radians, in the robot frame) and `angle_increment_rad` is the angular
    /// step between consecutive readings.
    #[allow(clippy::too_many_arguments)]
    pub fn ingest_laser_scan(
        &self,
        ranges: &[f32],
        angle_min_rad: f32,
        angle_increment_rad: f32,
        position_x: f32,
        position_y: f32,
        heading_rad: f32,
        battery_percent: u8,
    ) -> Result<usize, MechError> {
        // ── Input validation ───────────────────────────────────────────────
        if ranges.len() > MAX_LIDAR_RANGES {
            return Err(MechError::Parsing(format!(
                "laser scan has {} range readings, exceeding the limit of {}",
                ranges.len(),
                MAX_LIDAR_RANGES,
            )));
        }
        let telemetry_event = Event {
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
        self.bus.publish(telemetry_event)?;

        let lidar_event = Event {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: "mechos-middleware::ros2/scan".to_string(),
            payload: EventPayload::LidarScan {
                ranges: ranges.to_vec(),
                angle_min_rad,
                angle_increment_rad,
            },
        };
        self.bus.publish(lidar_event)
    }

    /// Ingest a fleet broadcast message arriving on `/fleet/communications` and
    /// publish it as a [`EventPayload::PeerMessage`] event on the internal bus.
    ///
    /// `from_robot_id` identifies the sender; `message` is the raw string
    /// payload that was carried inside the `std_msgs/msg/String` JSON frame.
    pub fn ingest_fleet_message(
        &self,
        from_robot_id: &str,
        message: &str,
    ) -> Result<usize, MechError> {
        // ── Input validation ───────────────────────────────────────────────
        if message.len() > MAX_FLEET_MESSAGE_BYTES {
            return Err(MechError::Parsing(format!(
                "fleet message from '{}' is {} bytes, exceeding the limit of {}",
                from_robot_id,
                message.len(),
                MAX_FLEET_MESSAGE_BYTES,
            )));
        }
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: "mechos-middleware::ros2/fleet/communications".to_string(),
            payload: EventPayload::PeerMessage {
                from_robot_id: from_robot_id.to_string(),
                message: message.to_string(),
            },
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
            HardwareIntent::MessagePeer {
                target_robot_id,
                message,
            } => {
                // Package as a std_msgs/msg/String JSON frame and publish to
                // the peer's dedicated topic.
                let peer_msg = json!({
                    "op": "publish",
                    "topic": format!("/fleet/robot/{target_robot_id}/inbox"),
                    "msg": { "data": message }
                });
                let event = Event {
                    id: Uuid::new_v4(),
                    timestamp: Utc::now(),
                    source: format!(
                        "mechos-middleware::ros2/fleet/robot/{target_robot_id}/inbox"
                    ),
                    payload: EventPayload::AgentThought(peer_msg.to_string()),
                };
                self.bus.publish(event).map(|_| ())
            }
            HardwareIntent::BroadcastFleet { message } => {
                // Package as a std_msgs/msg/String JSON frame on /fleet/communications.
                let broadcast_msg = json!({
                    "op": "publish",
                    "topic": "/fleet/communications",
                    "msg": { "data": message }
                });
                let event = Event {
                    id: Uuid::new_v4(),
                    timestamp: Utc::now(),
                    source: "mechos-middleware::ros2/fleet/communications".to_string(),
                    payload: EventPayload::AgentThought(broadcast_msg.to_string()),
                };
                self.bus.publish(event).map(|_| ())
            }
            HardwareIntent::PostTask { title, description } => {
                // Publish the task intent to the fleet task topic so remote
                // robots (and the task board consumer) can process it.
                let task_msg = json!({
                    "op": "publish",
                    "topic": "/fleet/tasks",
                    "msg": {
                        "data": serde_json::to_string(&json!({
                            "title": title,
                            "description": description
                        })).unwrap_or_default()
                    }
                });
                let event = Event {
                    id: Uuid::new_v4(),
                    timestamp: Utc::now(),
                    source: "mechos-middleware::ros2/fleet/tasks".to_string(),
                    payload: EventPayload::AgentThought(task_msg.to_string()),
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
    async fn ingest_laser_scan_rejects_oversized_ranges() {
        let (_, adapter) = make_adapter();
        let oversized_ranges: Vec<f32> = vec![1.0; MAX_LIDAR_RANGES + 1];
        let result = adapter.ingest_laser_scan(
            &oversized_ranges,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            100,
        );
        assert!(
            matches!(result, Err(MechError::Parsing(_))),
            "expected Parsing error for oversized LiDAR scan, got: {result:?}"
        );
    }

    #[tokio::test]
    async fn ingest_fleet_message_rejects_oversized_message() {
        let (_, adapter) = make_adapter();
        let oversized_msg = "x".repeat(MAX_FLEET_MESSAGE_BYTES + 1);
        let result = adapter.ingest_fleet_message("robot_alpha", &oversized_msg);
        assert!(
            matches!(result, Err(MechError::Parsing(_))),
            "expected Parsing error for oversized fleet message, got: {result:?}"
        );
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
            .ingest_laser_scan(
                &[1.0, 2.0, 3.0],
                -std::f32::consts::FRAC_PI_2,
                0.1,
                0.0,
                0.0,
                0.0,
                100,
            )
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

    #[tokio::test]
    async fn execute_broadcast_fleet_publishes_to_fleet_communications() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .execute_intent(HardwareIntent::BroadcastFleet {
                message: "I am at the Kitchen Door (X:5, Y:5).".to_string(),
            })
            .await
            .unwrap();

        let event = rx.recv().await.unwrap();
        assert_eq!(
            event.source,
            "mechos-middleware::ros2/fleet/communications"
        );
        if let EventPayload::AgentThought(json_str) = event.payload {
            assert!(json_str.contains("/fleet/communications"));
            assert!(json_str.contains("Kitchen Door"));
        }
    }

    #[tokio::test]
    async fn execute_message_peer_publishes_to_peer_inbox() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .execute_intent(HardwareIntent::MessagePeer {
                target_robot_id: "robot_bravo".to_string(),
                message: "I need help at X:5, Y:5.".to_string(),
            })
            .await
            .unwrap();

        let event = rx.recv().await.unwrap();
        assert!(event
            .source
            .contains("mechos-middleware::ros2/fleet/robot/robot_bravo"));
        if let EventPayload::AgentThought(json_str) = event.payload {
            assert!(json_str.contains("robot_bravo"));
        }
    }

    #[tokio::test]
    async fn execute_post_task_publishes_to_fleet_tasks() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .execute_intent(HardwareIntent::PostTask {
                title: "Move Box 1".to_string(),
                description: "Move the red box from shelf A to shelf B.".to_string(),
            })
            .await
            .unwrap();

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-middleware::ros2/fleet/tasks");
        if let EventPayload::AgentThought(json_str) = event.payload {
            assert!(json_str.contains("/fleet/tasks"));
            assert!(json_str.contains("Move Box 1"));
        }
    }

    #[tokio::test]
    async fn ingest_laser_scan_also_publishes_lidar_scan_event() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .ingest_laser_scan(
                &[1.5, 2.5],
                -std::f32::consts::FRAC_PI_2,
                0.1,
                0.0,
                0.0,
                0.0,
                80,
            )
            .unwrap();

        // First event is Telemetry.
        let first = rx.recv().await.unwrap();
        assert!(matches!(first.payload, EventPayload::Telemetry(_)));

        // Second event is LidarScan with the original ranges.
        let second = rx.recv().await.unwrap();
        assert_eq!(second.source, "mechos-middleware::ros2/scan");
        if let EventPayload::LidarScan {
            ranges,
            angle_min_rad,
            angle_increment_rad,
        } = second.payload
        {
            assert_eq!(ranges, vec![1.5, 2.5]);
            assert!((angle_min_rad - (-std::f32::consts::FRAC_PI_2)).abs() < 1e-5);
            assert!((angle_increment_rad - 0.1).abs() < 1e-5);
        } else {
            panic!("expected LidarScan payload as second event");
        }
    }

    #[tokio::test]
    async fn ingest_fleet_message_publishes_peer_message() {
        let (bus, adapter) = make_adapter();
        let mut rx = bus.subscribe();

        adapter
            .ingest_fleet_message("robot_alpha", "I am through. Thank you.")
            .unwrap();

        let event = rx.recv().await.unwrap();
        assert_eq!(
            event.source,
            "mechos-middleware::ros2/fleet/communications"
        );
        if let EventPayload::PeerMessage {
            from_robot_id,
            message,
        } = event.payload
        {
            assert_eq!(from_robot_id, "robot_alpha");
            assert_eq!(message, "I am through. Thank you.");
        } else {
            panic!("expected PeerMessage payload");
        }
    }
}
