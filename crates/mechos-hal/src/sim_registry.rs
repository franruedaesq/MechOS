//! [`SimRegistry`] – In-process simulation registry for CI/CD.
//!
//! Constructs a [`HardwareRegistry`] whose every slot is backed by a stub
//! driver that records commands without touching real hardware.  This allows
//! the full OS stack to run in a unit-test or CI environment without a
//! physical robot.
//!
//! # Stub behaviour
//!
//! | Driver type | Stub behaviour |
//! |---|---|
//! | [`Actuator`] | Stores the last `set_position` value; `position()` returns it. |
//! | [`Relay`]    | Stores the last `set_state` value; `state()` returns it. |
//! | [`Camera`]   | Returns a 1×1 single-channel (grayscale) black [`CameraFrame`] on every `capture()`. |
//!
//! # Example
//!
//! ```rust
//! use mechos_hal::sim_registry::SimRegistry;
//! use mechos_types::HardwareIntent;
//!
//! let mut registry = SimRegistry::builder()
//!     .with_actuator("left_wheel")
//!     .with_actuator("right_wheel")
//!     .with_relay("gripper")
//!     .build();
//!
//! registry.dispatch(HardwareIntent::Drive {
//!     linear_velocity: 1.0,
//!     angular_velocity: 0.0,
//! }).unwrap();
//! ```

use mechos_types::MechError;

use crate::actuator::Actuator;
use crate::camera::{Camera, CameraFrame};
use crate::registry::HardwareRegistry;
use crate::relay::Relay;

// ─────────────────────────────────────────────────────────────────────────────
// Stub actuator
// ─────────────────────────────────────────────────────────────────────────────

struct StubActuator {
    id: String,
    position: f32,
}

impl Actuator for StubActuator {
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

// ─────────────────────────────────────────────────────────────────────────────
// Stub relay
// ─────────────────────────────────────────────────────────────────────────────

struct StubRelay {
    id: String,
    state: bool,
}

impl Relay for StubRelay {
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

// ─────────────────────────────────────────────────────────────────────────────
// Stub camera
// ─────────────────────────────────────────────────────────────────────────────

struct StubCamera {
    id: String,
}

impl Camera for StubCamera {
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

// ─────────────────────────────────────────────────────────────────────────────
// SimRegistry builder
// ─────────────────────────────────────────────────────────────────────────────

/// Builder that constructs a [`HardwareRegistry`] populated with in-process
/// stub drivers.  Use this in tests and CI pipelines to exercise the full OS
/// stack without physical hardware.
#[derive(Default)]
pub struct SimRegistry {
    actuator_ids: Vec<String>,
    relay_ids: Vec<String>,
    camera_ids: Vec<String>,
}

impl SimRegistry {
    /// Create a new builder with no registered stubs.
    pub fn builder() -> Self {
        Self::default()
    }

    /// Register a stub [`Actuator`] with the given identifier.
    pub fn with_actuator(mut self, id: impl Into<String>) -> Self {
        self.actuator_ids.push(id.into());
        self
    }

    /// Register a stub [`Relay`] with the given identifier.
    pub fn with_relay(mut self, id: impl Into<String>) -> Self {
        self.relay_ids.push(id.into());
        self
    }

    /// Register a stub [`Camera`] with the given identifier.
    pub fn with_camera(mut self, id: impl Into<String>) -> Self {
        self.camera_ids.push(id.into());
        self
    }

    /// Consume the builder and return a fully populated [`HardwareRegistry`].
    pub fn build(self) -> HardwareRegistry {
        let mut registry = HardwareRegistry::new();

        for id in self.actuator_ids {
            registry.register_actuator(Box::new(StubActuator {
                id,
                position: 0.0,
            }));
        }

        for id in self.relay_ids {
            registry.register_relay(Box::new(StubRelay { id, state: false }));
        }

        for id in self.camera_ids {
            registry.register_camera(Box::new(StubCamera { id }));
        }

        registry
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use mechos_types::HardwareIntent;

    #[test]
    fn sim_registry_drives_differential_base() {
        let mut registry = SimRegistry::builder()
            .with_actuator("left_wheel")
            .with_actuator("right_wheel")
            .build();

        registry
            .dispatch(HardwareIntent::Drive {
                linear_velocity: 1.0,
                angular_velocity: 0.0,
            })
            .unwrap();

        // linear=1, angular=0 → both wheels at 1.0.
        // Access internal state via dispatch round-trip (position query).
        // We verify indirectly: a second dispatch does not return an error.
        registry
            .dispatch(HardwareIntent::Drive {
                linear_velocity: 0.0,
                angular_velocity: 0.0,
            })
            .unwrap();
    }

    #[test]
    fn sim_registry_toggles_relay() {
        let mut registry = SimRegistry::builder().with_relay("gripper").build();

        registry
            .dispatch(HardwareIntent::TriggerRelay {
                relay_id: "gripper".to_string(),
                state: true,
            })
            .unwrap();
    }

    #[test]
    fn sim_registry_missing_driver_returns_error() {
        let mut registry = SimRegistry::builder().build();

        let result = registry.dispatch(HardwareIntent::Drive {
            linear_velocity: 1.0,
            angular_velocity: 0.0,
        });
        assert!(matches!(result, Err(MechError::HardwareFault { .. })));
    }

    #[test]
    fn sim_registry_end_effector() {
        let mut registry = SimRegistry::builder()
            .with_actuator("end_effector")
            .build();

        registry
            .dispatch(HardwareIntent::MoveEndEffector {
                x: 0.5,
                y: 0.1,
                z: 0.2,
            })
            .unwrap();
    }

    #[test]
    fn sim_registry_ask_human_is_noop() {
        let mut registry = SimRegistry::builder().build();

        assert!(registry
            .dispatch(HardwareIntent::AskHuman {
                question: "Which path?".to_string(),
                context_image_id: None,
            })
            .is_ok());
    }
}
