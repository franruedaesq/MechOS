use chrono::{DateTime, Utc};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use thiserror::Error;
use uuid::Uuid;

/// Capability-based security model: defines what an agent or process is allowed to do.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Capability {
    /// Permission to move a physical actuator (e.g., "drive_base", "arm_joint_1")
    HardwareInvoke(String),
    /// Permission to read a specific sensor topic (e.g., "lidar/scan", "camera/rgb")
    SensorRead(String),
    /// Permission to invoke the local LLM (Ollama)
    ModelInference,
    /// Permission to access the persistent memory vector store
    MemoryAccess(String),
    /// Permission to send and receive messages over the fleet network
    FleetCommunicate,
    /// Permission to read from and write to the shared Fleet Task Board
    TaskBoardAccess,
}

/// Strict definition of physical actions the LLM is allowed to request.
/// `mechos-hal` parses these intents and translates them into motor currents.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "action", content = "payload")]
pub enum HardwareIntent {
    /// High-level: move the gripper/end-effector to a 3D world coordinate.
    /// The Universal Integration Adapter resolves the Inverse Kinematics.
    MoveEndEffector { x: f32, y: f32, z: f32 },
    /// Standard differential drive command
    Drive {
        linear_velocity: f32,
        angular_velocity: f32,
    },
    /// Command to trigger a discrete hardware action
    TriggerRelay { relay_id: String, state: bool },
    /// HITL: the AI is uncertain and requests human instruction via the Dashboard.
    AskHuman {
        question: String,
        context_image_id: Option<String>,
    },
    /// Send a direct message/request to another specific robot.
    MessagePeer { target_robot_id: String, message: String },
    /// Broadcast a state or discovery message to the entire fleet.
    BroadcastFleet { message: String },
    /// Post a task to the shared Fleet Task Board.
    PostTask { title: String, description: String },
}

/// Unified event wrapper for the headless event bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Event {
    pub id: Uuid,
    pub timestamp: DateTime<Utc>,
    /// e.g., "mechos-middleware::ros2"
    pub source: String,
    pub payload: EventPayload,
    /// W3C traceparent header propagated from the originating span.
    ///
    /// When an event is published through [`EventBus`] this field is
    /// automatically populated with the full W3C `traceparent` string
    /// (`"00-{trace_id}-{span_id}-{flags}"`) extracted from the current
    /// OpenTelemetry span context, or with `"tracing:<id>"` when no OTel
    /// provider is active.  Consumers can use this value to re-link their own
    /// spans to the originating trace and correlate the complete intent
    /// lifecycle—from LLM generation, through Kernel validation, to HAL
    /// execution—in any OTLP-compatible observability backend.
    ///
    /// Set to `None` when no span is active at publish time.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub trace_id: Option<String>,
}

/// Variants of data that can be routed over the internal event bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum EventPayload {
    Telemetry(TelemetryData),
    HardwareFault {
        component: String,
        code: u32,
        message: String,
    },
    /// The LLM's internal reasoning output
    AgentThought(String),
    /// A human operator's response to an [`HardwareIntent::AskHuman`] prompt,
    /// injected from the monitoring dashboard via the WebSocket API.
    HumanResponse(String),
    /// A message received from a peer robot over the fleet network.
    PeerMessage {
        /// The robot ID that sent the message.
        from_robot_id: String,
        /// The message content.
        message: String,
    },
    /// Raw LiDAR scan data for the Cockpit Sensory Visualizer.
    ///
    /// `ranges` contains measured distances (metres) in the order produced by
    /// the sensor; consecutive samples are separated by `angle_increment_rad`
    /// starting from `angle_min_rad` (both in radians).
    LidarScan {
        ranges: Vec<f32>,
        angle_min_rad: f32,
        angle_increment_rad: f32,
    },
    /// Cockpit mode-toggle command sent by the human operator.
    ///
    /// When `paused` is `true` the [`AgentLoop`] suspends the autonomous OODA
    /// cycle; `false` resumes it.  This is independent of the joystick
    /// manual-override interlock.
    AgentModeToggle { paused: bool },
}

/// Robot telemetry snapshot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetryData {
    pub position_x: f32,
    pub position_y: f32,
    pub heading_rad: f32,
    pub battery_percent: u8,
}

/// Returns the full set of [`Capability`] grants that a standard MechOS agent
/// must hold to operate all built-in hardware and sensors.
///
/// Use this list to bootstrap a [`mechos_kernel::CapabilityManager`] at
/// start-up and to verify that no required permission has been omitted.
///
/// # Standard capabilities
///
/// | Capability | Covers |
/// |---|---|
/// | `HardwareInvoke("drive_base")` | Differential-drive commands |
/// | `HardwareInvoke("arm_joint_1")` | Primary arm-joint actuation |
/// | `SensorRead("lidar/scan")` | LiDAR point-cloud topic |
/// | `SensorRead("camera/rgb")` | RGB camera image topic |
pub fn required_capabilities() -> Vec<Capability> {
    vec![
        Capability::HardwareInvoke("drive_base".to_string()),
        Capability::HardwareInvoke("arm_joint_1".to_string()),
        Capability::SensorRead("lidar/scan".to_string()),
        Capability::SensorRead("camera/rgb".to_string()),
        Capability::FleetCommunicate,
        Capability::TaskBoardAccess,
    ]
}

/// Global error type spanning hardware failures, LLM timeouts, and authorization rejections.
#[derive(Error, Debug, Serialize, Deserialize)]
pub enum MechError {
    #[error("Capability Denied: {0:?}")]
    Unauthorized(Capability),

    #[error("Hardware Fault on {component}: {details}")]
    HardwareFault { component: String, details: String },

    #[error("LLM Driver Error: {0}")]
    LlmInferenceFailed(String),

    #[error("Middleware Serialization Error: {0}")]
    Serialization(String),

    #[error("Channel Error: {0}")]
    Channel(String),

    #[error("Parsing Error: {0}")]
    Parsing(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capability_serialization_roundtrip() {
        let cap = Capability::HardwareInvoke("drive_base".to_string());
        let json = serde_json::to_string(&cap).unwrap();
        let back: Capability = serde_json::from_str(&json).unwrap();
        assert_eq!(cap, back);
    }

    #[test]
    fn hardware_intent_drive_roundtrip() {
        let intent = HardwareIntent::Drive {
            linear_velocity: 1.5,
            angular_velocity: -0.3,
        };
        let json = serde_json::to_string(&intent).unwrap();
        let back: HardwareIntent = serde_json::from_str(&json).unwrap();
        match back {
            HardwareIntent::Drive {
                linear_velocity,
                angular_velocity,
            } => {
                assert!((linear_velocity - 1.5).abs() < f32::EPSILON);
                assert!((angular_velocity - (-0.3)).abs() < f32::EPSILON);
            }
            _ => panic!("unexpected variant"),
        }
    }

    #[test]
    fn hardware_intent_move_end_effector_roundtrip() {
        let intent = HardwareIntent::MoveEndEffector {
            x: 0.5,
            y: -0.1,
            z: 0.3,
        };
        let json = serde_json::to_string(&intent).unwrap();
        let back: HardwareIntent = serde_json::from_str(&json).unwrap();
        match back {
            HardwareIntent::MoveEndEffector { x, y, z } => {
                assert!((x - 0.5).abs() < f32::EPSILON);
                assert!((y - (-0.1)).abs() < f32::EPSILON);
                assert!((z - 0.3).abs() < f32::EPSILON);
            }
            _ => panic!("unexpected variant"),
        }
    }

    #[test]
    fn hardware_intent_ask_human_roundtrip() {
        let intent = HardwareIntent::AskHuman {
            question: "Which shelf should I pick from?".to_string(),
            context_image_id: Some("frame_042".to_string()),
        };
        let json = serde_json::to_string(&intent).unwrap();
        let back: HardwareIntent = serde_json::from_str(&json).unwrap();
        match back {
            HardwareIntent::AskHuman {
                question,
                context_image_id,
            } => {
                assert_eq!(question, "Which shelf should I pick from?");
                assert_eq!(context_image_id.as_deref(), Some("frame_042"));
            }
            _ => panic!("unexpected variant"),
        }
    }

    #[test]
    fn hardware_intent_ask_human_no_image_roundtrip() {
        let intent = HardwareIntent::AskHuman {
            question: "Am I clear to proceed?".to_string(),
            context_image_id: None,
        };
        let json = serde_json::to_string(&intent).unwrap();
        let back: HardwareIntent = serde_json::from_str(&json).unwrap();
        assert!(matches!(
            back,
            HardwareIntent::AskHuman {
                context_image_id: None,
                ..
            }
        ));
    }

    #[test]
    fn hardware_intent_json_schema_is_derivable() {
        use schemars::schema_for;
        let schema = schema_for!(HardwareIntent);
        let json = serde_json::to_string(&schema).unwrap();
        assert!(json.contains("MoveEndEffector"));
        assert!(json.contains("Drive"));
        assert!(json.contains("TriggerRelay"));
        assert!(json.contains("AskHuman"));
        assert!(json.contains("MessagePeer"));
        assert!(json.contains("BroadcastFleet"));
        assert!(json.contains("PostTask"));
    }

    #[test]
    fn event_roundtrip() {
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: "mechos-middleware::ros2".to_string(),
            payload: EventPayload::Telemetry(TelemetryData {
                position_x: 1.0,
                position_y: 2.0,
                heading_rad: 0.5,
                battery_percent: 80,
            }),
            trace_id: None,
        };
        let json = serde_json::to_string(&event).unwrap();
        let back: Event = serde_json::from_str(&json).unwrap();
        assert_eq!(event.id, back.id);
        assert_eq!(event.source, back.source);
    }

    #[test]
    fn required_capabilities_contains_hardware_invoke_and_sensor_read() {
        let caps = required_capabilities();
        assert!(
            caps.contains(&Capability::HardwareInvoke("drive_base".to_string())),
            "drive_base HardwareInvoke must be present"
        );
        assert!(
            caps.contains(&Capability::HardwareInvoke("arm_joint_1".to_string())),
            "arm_joint_1 HardwareInvoke must be present"
        );
        assert!(
            caps.contains(&Capability::SensorRead("lidar/scan".to_string())),
            "lidar/scan SensorRead must be present"
        );
        assert!(
            caps.contains(&Capability::SensorRead("camera/rgb".to_string())),
            "camera/rgb SensorRead must be present"
        );
        assert!(
            caps.contains(&Capability::FleetCommunicate),
            "FleetCommunicate must be present"
        );
        assert!(
            caps.contains(&Capability::TaskBoardAccess),
            "TaskBoardAccess must be present"
        );
    }

    #[test]
    fn hardware_intent_message_peer_roundtrip() {
        let intent = HardwareIntent::MessagePeer {
            target_robot_id: "robot_bravo".to_string(),
            message: "I need help at X:5, Y:5.".to_string(),
        };
        let json = serde_json::to_string(&intent).unwrap();
        let back: HardwareIntent = serde_json::from_str(&json).unwrap();
        match back {
            HardwareIntent::MessagePeer {
                target_robot_id,
                message,
            } => {
                assert_eq!(target_robot_id, "robot_bravo");
                assert_eq!(message, "I need help at X:5, Y:5.");
            }
            _ => panic!("unexpected variant"),
        }
    }

    #[test]
    fn hardware_intent_broadcast_fleet_roundtrip() {
        let intent = HardwareIntent::BroadcastFleet {
            message: "I am at the Kitchen Door (X:5, Y:5).".to_string(),
        };
        let json = serde_json::to_string(&intent).unwrap();
        let back: HardwareIntent = serde_json::from_str(&json).unwrap();
        match back {
            HardwareIntent::BroadcastFleet { message } => {
                assert_eq!(message, "I am at the Kitchen Door (X:5, Y:5).");
            }
            _ => panic!("unexpected variant"),
        }
    }

    #[test]
    fn hardware_intent_post_task_roundtrip() {
        let intent = HardwareIntent::PostTask {
            title: "Move Box 1".to_string(),
            description: "Move the red box from shelf A to shelf B.".to_string(),
        };
        let json = serde_json::to_string(&intent).unwrap();
        let back: HardwareIntent = serde_json::from_str(&json).unwrap();
        match back {
            HardwareIntent::PostTask { title, description } => {
                assert_eq!(title, "Move Box 1");
                assert_eq!(description, "Move the red box from shelf A to shelf B.");
            }
            _ => panic!("unexpected variant"),
        }
    }

    #[test]
    fn peer_message_event_roundtrip() {
        let payload = EventPayload::PeerMessage {
            from_robot_id: "robot_alpha".to_string(),
            message: "I am through. Thank you.".to_string(),
        };
        let json = serde_json::to_string(&payload).unwrap();
        let back: EventPayload = serde_json::from_str(&json).unwrap();
        assert!(
            matches!(
                back,
                EventPayload::PeerMessage { ref from_robot_id, ref message }
                    if from_robot_id == "robot_alpha" && message == "I am through. Thank you."
            ),
            "PeerMessage must survive a JSON round-trip"
        );
    }

    #[test]
    fn fleet_communicate_capability_roundtrip() {
        let cap = Capability::FleetCommunicate;
        let json = serde_json::to_string(&cap).unwrap();
        let back: Capability = serde_json::from_str(&json).unwrap();
        assert_eq!(cap, back);
    }

    #[test]
    fn task_board_access_capability_roundtrip() {
        let cap = Capability::TaskBoardAccess;
        let json = serde_json::to_string(&cap).unwrap();
        let back: Capability = serde_json::from_str(&json).unwrap();
        assert_eq!(cap, back);
    }

    #[test]
    fn human_response_roundtrip() {
        let payload = EventPayload::HumanResponse("Yes, push it".to_string());
        let json = serde_json::to_string(&payload).unwrap();
        let back: EventPayload = serde_json::from_str(&json).unwrap();
        assert!(
            matches!(back, EventPayload::HumanResponse(ref s) if s == "Yes, push it"),
            "HumanResponse must survive a JSON round-trip"
        );
    }

    #[test]
    fn mech_error_display() {
        let err = MechError::Unauthorized(Capability::ModelInference);
        assert!(err.to_string().contains("Capability Denied"));

        let err2 = MechError::HardwareFault {
            component: "arm_joint_1".to_string(),
            details: "overcurrent".to_string(),
        };
        assert!(err2.to_string().contains("arm_joint_1"));
    }

    #[test]
    fn lidar_scan_roundtrip() {
        let payload = EventPayload::LidarScan {
            ranges: vec![0.5, 1.0, 1.5, 2.0],
            angle_min_rad: -std::f32::consts::FRAC_PI_2,
            angle_increment_rad: 0.017453293,
        };
        let json = serde_json::to_string(&payload).unwrap();
        let back: EventPayload = serde_json::from_str(&json).unwrap();
        match back {
            EventPayload::LidarScan {
                ranges,
                angle_min_rad,
                angle_increment_rad,
            } => {
                assert_eq!(ranges.len(), 4);
                assert!((angle_min_rad - (-std::f32::consts::FRAC_PI_2)).abs() < 1e-6);
                assert!((angle_increment_rad - 0.017453293).abs() < 1e-9);
            }
            _ => panic!("expected LidarScan"),
        }
    }

    #[test]
    fn agent_mode_toggle_paused_roundtrip() {
        let payload = EventPayload::AgentModeToggle { paused: true };
        let json = serde_json::to_string(&payload).unwrap();
        let back: EventPayload = serde_json::from_str(&json).unwrap();
        assert!(
            matches!(back, EventPayload::AgentModeToggle { paused: true }),
            "AgentModeToggle(paused=true) must survive a JSON round-trip"
        );
    }

    #[test]
    fn agent_mode_toggle_resumed_roundtrip() {
        let payload = EventPayload::AgentModeToggle { paused: false };
        let json = serde_json::to_string(&payload).unwrap();
        let back: EventPayload = serde_json::from_str(&json).unwrap();
        assert!(
            matches!(back, EventPayload::AgentModeToggle { paused: false }),
            "AgentModeToggle(paused=false) must survive a JSON round-trip"
        );
    }
}
