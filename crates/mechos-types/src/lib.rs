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
}

/// Unified event wrapper for the headless event bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Event {
    pub id: Uuid,
    pub timestamp: DateTime<Utc>,
    /// e.g., "mechos-middleware::ros2"
    pub source: String,
    pub payload: EventPayload,
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
}
