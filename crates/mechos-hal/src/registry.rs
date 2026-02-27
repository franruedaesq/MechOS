//! [`HardwareRegistry`] – central driver registry and intent dispatcher.
//!
//! The registry stores references to every registered [`Actuator`],
//! [`Relay`], and [`Camera`] driver.  When the OS issues a
//! [`HardwareIntent`][mechos_types::HardwareIntent], the registry resolves
//! the target driver by its identifier and calls the appropriate method.
//!
//! # Differential-drive mapping
//!
//! The [`HardwareIntent::Drive`] command (linear + angular velocity) is
//! decomposed into per-wheel target positions using a unit-wheelbase
//! kinematic model and forwarded to actuators named `"left_wheel"` and
//! `"right_wheel"`.  Register actuators with those identifiers to enable
//! drive support.

use std::collections::HashMap;

use mechos_types::{HardwareIntent, MechError};

use crate::actuator::Actuator;
use crate::camera::Camera;
use crate::relay::Relay;

/// Central hardware driver registry and [`HardwareIntent`] dispatcher.
///
/// Construct with [`HardwareRegistry::new`], register drivers, then call
/// [`HardwareRegistry::dispatch`] to translate intents into hardware calls.
#[derive(Default)]
pub struct HardwareRegistry {
    actuators: HashMap<String, Box<dyn Actuator>>,
    relays: HashMap<String, Box<dyn Relay>>,
    cameras: HashMap<String, Box<dyn Camera>>,
}

impl HardwareRegistry {
    /// Create an empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register an actuator driver.  Any previously registered driver with the
    /// same `id` is replaced.
    pub fn register_actuator(&mut self, actuator: Box<dyn Actuator>) {
        self.actuators.insert(actuator.id().to_string(), actuator);
    }

    /// Register a relay driver.  Any previously registered driver with the
    /// same `id` is replaced.
    pub fn register_relay(&mut self, relay: Box<dyn Relay>) {
        self.relays.insert(relay.id().to_string(), relay);
    }

    /// Register a camera driver.  Any previously registered driver with the
    /// same `id` is replaced.
    pub fn register_camera(&mut self, camera: Box<dyn Camera>) {
        self.cameras.insert(camera.id().to_string(), camera);
    }

    /// Dispatch a [`HardwareIntent`] to the appropriate registered driver.
    ///
    /// # Errors
    ///
    /// Returns [`MechError::HardwareFault`] when the target driver is not
    /// registered or when the underlying driver call fails.
    pub fn dispatch(&mut self, intent: HardwareIntent) -> Result<(), MechError> {
        match intent {
            // ----------------------------------------------------------------
            // High-level end-effector move: forward to a registered "end_effector"
            // actuator.  The HAL `Actuator` trait operates on a single f32 position
            // value; the Universal Integration Adapter is expected to translate the
            // full 3-D coordinate (x, y, z) into joint angles *before* this
            // registry is called, then register the resolved actuator.  Until an
            // IK adapter is wired in, the registry stores x as the representative
            // position and reports the full target in any error message.
            // ----------------------------------------------------------------
            HardwareIntent::MoveEndEffector { x, y, z } => {
                self.actuate("end_effector", x).map_err(|_| MechError::HardwareFault {
                    component: "end_effector".to_string(),
                    details: format!(
                        "end_effector actuator not registered (target x={x}, y={y}, z={z})"
                    ),
                })
            }

            // ----------------------------------------------------------------
            // Differential drive: decompose (v, ω) → left/right wheel targets.
            // Assumes a unit wheelbase (track width = 1 m or 1 rad unit).
            // ----------------------------------------------------------------
            HardwareIntent::Drive {
                linear_velocity,
                angular_velocity,
            } => {
                let left_target = linear_velocity - angular_velocity * 0.5;
                let right_target = linear_velocity + angular_velocity * 0.5;
                self.actuate("left_wheel", left_target)?;
                self.actuate("right_wheel", right_target)?;
                Ok(())
            }

            // ----------------------------------------------------------------
            // Discrete relay command.
            // ----------------------------------------------------------------
            HardwareIntent::TriggerRelay { relay_id, state } => {
                match self.relays.get_mut(&relay_id) {
                    Some(relay) => relay.set_state(state),
                    None => Err(MechError::HardwareFault {
                        component: relay_id.clone(),
                        details: format!("relay '{relay_id}' is not registered"),
                    }),
                }
            }

            // ----------------------------------------------------------------
            // HITL: the AI is requesting human guidance.  At the HAL level
            // this is a no-op – the intent is surfaced to the Dashboard by
            // the EventBus upstream; no physical actuator is commanded.
            // ----------------------------------------------------------------
            HardwareIntent::AskHuman { .. } => Ok(()),
        }
    }

    // Internal helper: look up an actuator and call set_position.
    fn actuate(&mut self, id: &str, target_rad: f32) -> Result<(), MechError> {
        match self.actuators.get_mut(id) {
            Some(act) => act.set_position(target_rad),
            None => Err(MechError::HardwareFault {
                component: id.to_string(),
                details: format!("actuator '{id}' is not registered"),
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actuator::Actuator;
    use crate::camera::{Camera, CameraFrame};
    use crate::relay::Relay;

    // ------------------------------------------------------------------
    // Test doubles
    // ------------------------------------------------------------------

    struct MockActuator {
        id: String,
        position: f32,
    }
    impl MockActuator {
        fn new(id: &str) -> Box<Self> {
            Box::new(Self {
                id: id.to_string(),
                position: 0.0,
            })
        }
    }
    impl Actuator for MockActuator {
        fn id(&self) -> &str {
            &self.id
        }
        fn set_position(&mut self, target_rad: f32) -> Result<(), MechError> {
            self.position = target_rad;
            Ok(())
        }
        fn position(&self) -> f32 {
            self.position
        }
    }

    struct MockRelay {
        id: String,
        state: bool,
    }
    impl MockRelay {
        fn new(id: &str) -> Box<Self> {
            Box::new(Self {
                id: id.to_string(),
                state: false,
            })
        }
    }
    impl Relay for MockRelay {
        fn id(&self) -> &str {
            &self.id
        }
        fn set_state(&mut self, active: bool) -> Result<(), MechError> {
            self.state = active;
            Ok(())
        }
        fn state(&self) -> bool {
            self.state
        }
    }

    struct MockCamera {
        id: String,
    }
    impl Camera for MockCamera {
        fn id(&self) -> &str {
            &self.id
        }
        fn capture(&mut self) -> Result<CameraFrame, MechError> {
            Ok(CameraFrame {
                width: 1,
                height: 1,
                data: vec![0u8],
            })
        }
    }

    // ------------------------------------------------------------------
    // Tests
    // ------------------------------------------------------------------

    #[test]
    fn dispatch_move_end_effector() {
        let mut registry = HardwareRegistry::new();
        registry.register_actuator(MockActuator::new("end_effector"));

        registry
            .dispatch(HardwareIntent::MoveEndEffector {
                x: 0.3,
                y: 0.1,
                z: 0.5,
            })
            .unwrap();

        // x is forwarded as the position to the end_effector actuator
        let pos = registry.actuators["end_effector"].position();
        assert!((pos - 0.3).abs() < f32::EPSILON);
    }

    #[test]
    fn dispatch_trigger_relay() {
        let mut registry = HardwareRegistry::new();
        registry.register_relay(MockRelay::new("gripper"));

        registry
            .dispatch(HardwareIntent::TriggerRelay {
                relay_id: "gripper".to_string(),
                state: true,
            })
            .unwrap();

        assert!(registry.relays["gripper"].state());
    }

    #[test]
    fn dispatch_drive_sets_wheel_targets() {
        let mut registry = HardwareRegistry::new();
        registry.register_actuator(MockActuator::new("left_wheel"));
        registry.register_actuator(MockActuator::new("right_wheel"));

        // linear=1.0, angular=0.0 → both wheels = 1.0
        registry
            .dispatch(HardwareIntent::Drive {
                linear_velocity: 1.0,
                angular_velocity: 0.0,
            })
            .unwrap();

        let left = registry.actuators["left_wheel"].position();
        let right = registry.actuators["right_wheel"].position();
        assert!((left - 1.0).abs() < f32::EPSILON);
        assert!((right - 1.0).abs() < f32::EPSILON);

        // Turn in place: linear=0, angular=1.0 → left=-0.5, right=0.5
        registry
            .dispatch(HardwareIntent::Drive {
                linear_velocity: 0.0,
                angular_velocity: 1.0,
            })
            .unwrap();

        let left = registry.actuators["left_wheel"].position();
        let right = registry.actuators["right_wheel"].position();
        assert!((left - (-0.5)).abs() < f32::EPSILON);
        assert!((right - 0.5).abs() < f32::EPSILON);
    }

    #[test]
    fn dispatch_ask_human_is_noop() {
        let mut registry = HardwareRegistry::new();
        // AskHuman does not require any hardware; should always succeed.
        assert!(registry
            .dispatch(HardwareIntent::AskHuman {
                question: "Which direction?".to_string(),
                context_image_id: None,
            })
            .is_ok());
    }

    #[test]
    fn dispatch_missing_end_effector_returns_error() {
        let mut registry = HardwareRegistry::new();
        let result = registry.dispatch(HardwareIntent::MoveEndEffector {
            x: 0.5,
            y: 0.0,
            z: 1.0,
        });
        assert!(matches!(result, Err(MechError::HardwareFault { .. })));
    }

    #[test]
    fn dispatch_missing_actuator_returns_error() {
        let mut registry = HardwareRegistry::new();
        let result = registry.dispatch(HardwareIntent::Drive {
            linear_velocity: 1.0,
            angular_velocity: 0.0,
        });
        assert!(matches!(result, Err(MechError::HardwareFault { .. })));
    }

    #[test]
    fn dispatch_missing_relay_returns_error() {
        let mut registry = HardwareRegistry::new();
        let result = registry.dispatch(HardwareIntent::TriggerRelay {
            relay_id: "nonexistent".to_string(),
            state: true,
        });
        assert!(matches!(result, Err(MechError::HardwareFault { .. })));
    }

    #[test]
    fn register_camera_and_capture() {
        let mut registry = HardwareRegistry::new();
        registry.register_camera(Box::new(MockCamera {
            id: "front_rgb".to_string(),
        }));
        let frame = registry
            .cameras
            .get_mut("front_rgb")
            .unwrap()
            .capture()
            .unwrap();
        assert_eq!(frame.width, 1);
    }

    #[test]
    fn re_registering_actuator_replaces_old_driver() {
        let mut registry = HardwareRegistry::new();
        registry.register_actuator(MockActuator::new("end_effector"));
        registry
            .dispatch(HardwareIntent::MoveEndEffector {
                x: 3.0,
                y: 0.0,
                z: 0.0,
            })
            .unwrap();

        // Re-register a fresh driver with the same id (position resets to 0).
        registry.register_actuator(MockActuator::new("end_effector"));
        let pos = registry.actuators["end_effector"].position();
        assert!((pos - 0.0).abs() < f32::EPSILON);
    }
}
