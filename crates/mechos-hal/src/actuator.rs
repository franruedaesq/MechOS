//! Generic `Actuator` trait for motors, servos, and any position-controlled
//! hardware.
//!
//! Drivers implement this trait and register themselves with a
//! [`HardwareRegistry`][crate::registry::HardwareRegistry].  The rest of the
//! OS only ever talks to the trait, so drivers can be swapped without touching
//! AI or planning logic.

use mechos_types::MechError;

/// A position-controlled hardware actuator (motor, servo, joint, wheel, …).
///
/// Every actuator has a stable string identifier so the
/// [`HardwareRegistry`][crate::registry::HardwareRegistry] can route
/// [`HardwareIntent`][mechos_types::HardwareIntent] commands to the correct
/// driver.
pub trait Actuator: Send + Sync {
    /// Stable identifier for this actuator, e.g. `"arm_joint_1"` or
    /// `"left_wheel"`.
    fn id(&self) -> &str;

    /// Command the actuator to move to `target_rad` (radians from its zero
    /// position).
    ///
    /// # Errors
    ///
    /// Returns [`MechError::HardwareFault`] if the command cannot be applied
    /// (e.g. the actuator is in a fault state or the target is out of range).
    fn set_position(&mut self, target_rad: f32) -> Result<(), MechError>;

    /// Return the actuator's most recently known position in radians.
    fn position(&self) -> f32;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimal in-process actuator used only for tests.
    struct MockActuator {
        id: String,
        position: f32,
    }

    impl MockActuator {
        fn new(id: &str) -> Self {
            Self {
                id: id.to_string(),
                position: 0.0,
            }
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

    #[test]
    fn mock_actuator_set_and_get_position() {
        let mut act = MockActuator::new("test_joint");
        assert_eq!(act.id(), "test_joint");
        assert!((act.position() - 0.0).abs() < f32::EPSILON);

        act.set_position(std::f32::consts::FRAC_PI_2).unwrap();
        assert!((act.position() - std::f32::consts::FRAC_PI_2).abs() < f32::EPSILON);
    }
}
