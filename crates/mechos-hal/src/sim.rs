//! In-process simulation registry for CI/CD testing without physical hardware.
//!
//! [`SimRegistry`] wraps a [`HardwareRegistry`] and pre-populates it with
//! stub drivers that record commands and return plausible kinematic state.
//! This lets the full MechOS stack run in headless tests and CI pipelines
//! without requiring any physical hardware.
//!
//! # Example
//!
//! ```rust
//! use mechos_hal::sim::SimRegistry;
//! use mechos_types::HardwareIntent;
//!
//! let mut registry = SimRegistry::new()
//!     .with_drive_base()
//!     .build();
//!
//! registry
//!     .dispatch(HardwareIntent::Drive {
//!         linear_velocity: 0.5,
//!         angular_velocity: 0.1,
//!     })
//!     .expect("sim drive must succeed");
//! ```

use mechos_types::MechError;

use crate::actuator::Actuator;
use crate::camera::{Camera, CameraFrame};
use crate::registry::HardwareRegistry;
use crate::relay::Relay;

// ────────────────────────────────────────────────────────────────────────────
// Stub actuator
// ────────────────────────────────────────────────────────────────────────────

/// A simulated position-controlled actuator that records the most recent
/// commanded position.  Always succeeds.
pub struct SimActuator {
    id: String,
    position: f32,
}

impl SimActuator {
    /// Create a new simulated actuator with the given identifier.
    pub fn new(id: impl Into<String>) -> Box<Self> {
        Box::new(Self {
            id: id.into(),
            position: 0.0,
        })
    }
}

impl Actuator for SimActuator {
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

// ────────────────────────────────────────────────────────────────────────────
// Stub relay
// ────────────────────────────────────────────────────────────────────────────

/// A simulated relay (discrete on/off device) that records the current state.
/// Always succeeds.
pub struct SimRelay {
    id: String,
    state: bool,
}

impl SimRelay {
    /// Create a new simulated relay with the given identifier.
    pub fn new(id: impl Into<String>) -> Box<Self> {
        Box::new(Self {
            id: id.into(),
            state: false,
        })
    }
}

impl Relay for SimRelay {
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

// ────────────────────────────────────────────────────────────────────────────
// Stub camera
// ────────────────────────────────────────────────────────────────────────────

/// A simulated camera that returns a blank (all-zero) 4×4 greyscale frame.
/// Always succeeds.
pub struct SimCamera {
    id: String,
}

impl SimCamera {
    /// Create a new simulated camera with the given identifier.
    pub fn new(id: impl Into<String>) -> Box<Self> {
        Box::new(Self { id: id.into() })
    }
}

impl Camera for SimCamera {
    fn id(&self) -> &str {
        &self.id
    }

    fn capture(&mut self) -> Result<CameraFrame, MechError> {
        Ok(CameraFrame {
            width: 4,
            height: 4,
            data: vec![0u8; 16],
        })
    }
}

// ────────────────────────────────────────────────────────────────────────────
// SimRegistry builder
// ────────────────────────────────────────────────────────────────────────────

/// Builder that constructs a [`HardwareRegistry`] pre-populated with simulated
/// stub drivers for headless CI/CD testing.
///
/// Call the `with_*` methods to add the simulated components you need, then
/// call [`build`][Self::build] to obtain the fully configured registry.
#[derive(Default)]
pub struct SimRegistry {
    actuators: Vec<Box<dyn Actuator>>,
    relays: Vec<Box<dyn Relay>>,
    cameras: Vec<Box<dyn Camera>>,
}

impl SimRegistry {
    /// Create an empty [`SimRegistry`] builder.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a simulated differential-drive base with `"left_wheel"` and
    /// `"right_wheel"` actuators.
    pub fn with_drive_base(mut self) -> Self {
        self.actuators.push(SimActuator::new("left_wheel"));
        self.actuators.push(SimActuator::new("right_wheel"));
        self
    }

    /// Register a simulated end-effector (robotic arm) actuator.
    pub fn with_end_effector(mut self) -> Self {
        self.actuators.push(SimActuator::new("end_effector"));
        self
    }

    /// Register a simulated relay with the given identifier.
    pub fn with_relay(mut self, id: impl Into<String>) -> Self {
        self.relays.push(SimRelay::new(id));
        self
    }

    /// Register a simulated camera with the given identifier.
    pub fn with_camera(mut self, id: impl Into<String>) -> Self {
        self.cameras.push(SimCamera::new(id));
        self
    }

    /// Register a custom actuator driver.  Useful when a test needs a
    /// specific tracking actuator to assert on command values.
    pub fn with_actuator(mut self, actuator: Box<dyn Actuator>) -> Self {
        self.actuators.push(actuator);
        self
    }

    /// Consume the builder and return a fully configured [`HardwareRegistry`].
    pub fn build(self) -> HardwareRegistry {
        let mut registry = HardwareRegistry::new();
        for a in self.actuators {
            registry.register_actuator(a);
        }
        for r in self.relays {
            registry.register_relay(r);
        }
        for c in self.cameras {
            registry.register_camera(c);
        }
        registry
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use mechos_types::HardwareIntent;

    #[test]
    fn sim_registry_drive_base_dispatches_successfully() {
        let mut registry = SimRegistry::new().with_drive_base().build();
        registry
            .dispatch(HardwareIntent::Drive {
                linear_velocity: 1.0,
                angular_velocity: 0.0,
            })
            .expect("sim drive must succeed");
    }

    #[test]
    fn sim_registry_end_effector_dispatches_successfully() {
        let mut registry = SimRegistry::new().with_end_effector().build();
        registry
            .dispatch(HardwareIntent::MoveEndEffector {
                x: 0.5,
                y: 0.2,
                z: 0.3,
            })
            .expect("sim end_effector must succeed");
    }

    #[test]
    fn sim_registry_relay_dispatches_successfully() {
        let mut registry = SimRegistry::new().with_relay("gripper").build();
        registry
            .dispatch(HardwareIntent::TriggerRelay {
                relay_id: "gripper".to_string(),
                state: true,
            })
            .expect("sim relay must succeed");
    }

    #[test]
    fn sim_actuator_records_position() {
        let mut act = SimActuator::new("test");
        assert!((act.position() - 0.0).abs() < f32::EPSILON);
        act.set_position(1.57).unwrap();
        assert!((act.position() - 1.57).abs() < f32::EPSILON);
    }

    #[test]
    fn sim_relay_records_state() {
        let mut relay = SimRelay::new("test_relay");
        assert!(!relay.state());
        relay.set_state(true).unwrap();
        assert!(relay.state());
        relay.set_state(false).unwrap();
        assert!(!relay.state());
    }

    #[test]
    fn sim_camera_returns_blank_frame() {
        let mut cam = SimCamera::new("front");
        let frame = cam.capture().unwrap();
        assert_eq!(frame.width, 4);
        assert_eq!(frame.height, 4);
        assert!(frame.data.iter().all(|&b| b == 0));
    }

    #[test]
    fn sim_registry_full_stack_no_hardware_required() {
        // Verify that a complete simulated stack can be constructed and
        // exercised without any physical hardware.
        let mut registry = SimRegistry::new()
            .with_drive_base()
            .with_end_effector()
            .with_relay("gripper")
            .with_camera("front_rgb")
            .build();

        registry
            .dispatch(HardwareIntent::Drive {
                linear_velocity: 0.5,
                angular_velocity: -0.2,
            })
            .expect("drive must succeed");

        registry
            .dispatch(HardwareIntent::MoveEndEffector {
                x: 0.1,
                y: 0.2,
                z: 0.3,
            })
            .expect("move_end_effector must succeed");

        registry
            .dispatch(HardwareIntent::TriggerRelay {
                relay_id: "gripper".to_string(),
                state: true,
            })
            .expect("relay must succeed");

        registry
            .dispatch(HardwareIntent::AskHuman {
                question: "Continue?".to_string(),
                context_image_id: None,
            })
            .expect("ask_human must succeed");
    }
}
