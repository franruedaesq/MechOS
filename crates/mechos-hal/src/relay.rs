//! Generic `Relay` trait for discrete on/off hardware devices (solenoids,
//! power switches, LED arrays, …).

use mechos_types::MechError;

/// A discrete on/off hardware device.
///
/// Drivers implement this trait and register themselves with a
/// [`HardwareRegistry`][crate::registry::HardwareRegistry].
pub trait Relay: Send + Sync {
    /// Stable identifier for this relay, e.g. `"gripper_solenoid"`.
    fn id(&self) -> &str;

    /// Drive the relay to `active` (`true` = energised / on,
    /// `false` = de-energised / off).
    ///
    /// # Errors
    ///
    /// Returns [`MechError::HardwareFault`] if the command cannot be applied.
    fn set_state(&mut self, active: bool) -> Result<(), MechError>;

    /// Return the relay's current state (`true` = energised).
    fn state(&self) -> bool;
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockRelay {
        id: String,
        state: bool,
    }

    impl MockRelay {
        fn new(id: &str) -> Self {
            Self {
                id: id.to_string(),
                state: false,
            }
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

    #[test]
    fn mock_relay_toggle() {
        let mut relay = MockRelay::new("gripper");
        assert_eq!(relay.id(), "gripper");
        assert!(!relay.state());

        relay.set_state(true).unwrap();
        assert!(relay.state());

        relay.set_state(false).unwrap();
        assert!(!relay.state());
    }
}
