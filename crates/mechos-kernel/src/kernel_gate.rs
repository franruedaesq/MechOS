//! [`KernelGate`] – single interception point between the runtime and the HAL.
//!
//! Before a [`HardwareIntent`] produced by `mechos-runtime` reaches
//! `mechos-hal`, it must pass through [`KernelGate::authorize_and_verify`].
//! This method enforces **two independent safety checks** in order:
//!
//! 1. **Capability check** ([`CapabilityManager`]): the requesting agent must
//!    hold the [`Capability::HardwareInvoke`] permission that corresponds to
//!    the target hardware component.  Unknown agents and missing grants receive
//!    a [`MechError::Unauthorized`] response.
//!
//! 2. **Physical invariant check** ([`StateVerifier`]): the intent values must
//!    satisfy every registered [`Rule`] (e.g. speed caps, workspace bounds).  The
//!    first violated rule returns a [`MechError::HardwareFault`].
//!
//! Only when both checks pass is the caller permitted to forward the intent to
//! the HAL.
//!
//! # Example
//!
//! ```
//! use mechos_kernel::{
//!     KernelGate,
//!     CapabilityManager,
//!     SpeedCapRule,
//!     StateVerifier,
//! };
//! use mechos_types::{Capability, HardwareIntent};
//!
//! let mut caps = CapabilityManager::new();
//! caps.grant("runtime", Capability::HardwareInvoke("drive_base".into()));
//!
//! let mut verifier = StateVerifier::new();
//! verifier.add_rule(Box::new(SpeedCapRule { max_linear: 1.0, max_angular: 1.0 }));
//!
//! let gate = KernelGate::new(caps, verifier);
//!
//! // Authorized + within caps → allowed.
//! let ok = HardwareIntent::Drive { linear_velocity: 0.5, angular_velocity: 0.0 };
//! assert!(gate.authorize_and_verify("runtime", &ok).is_ok());
//!
//! // Over speed cap → rejected.
//! let fast = HardwareIntent::Drive { linear_velocity: 5.0, angular_velocity: 0.0 };
//! assert!(gate.authorize_and_verify("runtime", &fast).is_err());
//! ```

use mechos_types::{Capability, HardwareIntent, MechError};

use crate::capability_manager::CapabilityManager;
use crate::state_verifier::StateVerifier;

/// The single gateway that `mechos-runtime` must use before forwarding any
/// [`HardwareIntent`] to `mechos-hal`.
pub struct KernelGate {
    capability_manager: CapabilityManager,
    state_verifier: StateVerifier,
}

impl KernelGate {
    /// Construct a gate from an already-configured [`CapabilityManager`] and
    /// [`StateVerifier`].
    pub fn new(capability_manager: CapabilityManager, state_verifier: StateVerifier) -> Self {
        Self {
            capability_manager,
            state_verifier,
        }
    }

    /// Authorize `agent_id` for `intent` and validate the intent against all
    /// physical invariants.
    ///
    /// The capability required by each intent variant is:
    ///
    /// | Intent | Required [`Capability`] |
    /// |--------|------------------------|
    /// | `MoveEndEffector { .. }` | `HardwareInvoke("end_effector")` |
    /// | `Drive` | `HardwareInvoke("drive_base")` |
    /// | `TriggerRelay { relay_id, .. }` | `HardwareInvoke(relay_id)` |
    /// | `AskHuman { .. }` | `HardwareInvoke("hitl")` |
    ///
    /// # Errors
    ///
    /// - [`MechError::Unauthorized`] – agent is missing the required capability.
    /// - [`MechError::HardwareFault`] – a physical safety rule was violated.
    pub fn authorize_and_verify(
        &self,
        agent_id: &str,
        intent: &HardwareIntent,
    ) -> Result<(), MechError> {
        let required_cap = Self::capability_for(intent);
        self.capability_manager.check(agent_id, &required_cap)?;
        self.state_verifier.verify(intent)?;
        Ok(())
    }

    /// Map a [`HardwareIntent`] to the [`Capability`] the agent must hold.
    fn capability_for(intent: &HardwareIntent) -> Capability {
        match intent {
            HardwareIntent::MoveEndEffector { .. } => {
                Capability::HardwareInvoke("end_effector".to_string())
            }
            HardwareIntent::Drive { .. } => Capability::HardwareInvoke("drive_base".to_string()),
            HardwareIntent::TriggerRelay { relay_id, .. } => {
                Capability::HardwareInvoke(relay_id.clone())
            }
            HardwareIntent::AskHuman { .. } => Capability::HardwareInvoke("hitl".to_string()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state_verifier::SpeedCapRule;

    fn gated_drive(max_linear: f32, max_angular: f32) -> KernelGate {
        let mut caps = CapabilityManager::new();
        caps.grant("runtime", Capability::HardwareInvoke("drive_base".into()));

        let mut verifier = StateVerifier::new();
        verifier.add_rule(Box::new(SpeedCapRule {
            max_linear,
            max_angular,
        }));

        KernelGate::new(caps, verifier)
    }

    #[test]
    fn authorized_and_within_caps_passes() {
        let gate = gated_drive(1.0, 1.0);
        assert!(gate
            .authorize_and_verify(
                "runtime",
                &HardwareIntent::Drive {
                    linear_velocity: 0.5,
                    angular_velocity: 0.0,
                }
            )
            .is_ok());
    }

    #[test]
    fn missing_capability_is_rejected() {
        let gate = gated_drive(1.0, 1.0);
        // "rogue" has no grants.
        let result = gate.authorize_and_verify(
            "rogue",
            &HardwareIntent::Drive {
                linear_velocity: 0.1,
                angular_velocity: 0.0,
            },
        );
        assert!(matches!(result, Err(MechError::Unauthorized(_))));
    }

    #[test]
    fn capability_check_fails_before_state_check() {
        // Even a physically safe intent is blocked if the agent is not authorized.
        let gate = gated_drive(1.0, 1.0);
        let result = gate.authorize_and_verify(
            "unknown_agent",
            &HardwareIntent::Drive {
                linear_velocity: 0.0,
                angular_velocity: 0.0,
            },
        );
        assert!(matches!(result, Err(MechError::Unauthorized(_))));
    }

    #[test]
    fn over_speed_cap_is_rejected_after_authorization() {
        let gate = gated_drive(1.0, 1.0);
        let result = gate.authorize_and_verify(
            "runtime",
            &HardwareIntent::Drive {
                linear_velocity: 5.0,
                angular_velocity: 0.0,
            },
        );
        assert!(matches!(result, Err(MechError::HardwareFault { .. })));
    }

    #[test]
    fn end_effector_intent_requires_end_effector_capability() {
        let mut caps = CapabilityManager::new();
        caps.grant("runtime", Capability::HardwareInvoke("end_effector".into()));

        let gate = KernelGate::new(caps, StateVerifier::new());

        assert!(gate
            .authorize_and_verify(
                "runtime",
                &HardwareIntent::MoveEndEffector {
                    x: 0.1,
                    y: 0.2,
                    z: 0.5,
                }
            )
            .is_ok());

        // Missing capability → denied.
        assert!(gate
            .authorize_and_verify(
                "unknown",
                &HardwareIntent::MoveEndEffector {
                    x: 0.1,
                    y: 0.2,
                    z: 0.5,
                }
            )
            .is_err());
    }

    #[test]
    fn relay_intent_requires_relay_capability() {
        let mut caps = CapabilityManager::new();
        caps.grant("runtime", Capability::HardwareInvoke("gripper".into()));

        let gate = KernelGate::new(caps, StateVerifier::new());

        assert!(gate
            .authorize_and_verify(
                "runtime",
                &HardwareIntent::TriggerRelay {
                    relay_id: "gripper".into(),
                    state: true,
                }
            )
            .is_ok());
    }

    #[test]
    fn ask_human_requires_hitl_capability() {
        let mut caps = CapabilityManager::new();
        caps.grant("runtime", Capability::HardwareInvoke("hitl".into()));

        let gate = KernelGate::new(caps, StateVerifier::new());

        assert!(gate
            .authorize_and_verify(
                "runtime",
                &HardwareIntent::AskHuman {
                    question: "Which path is safe?".to_string(),
                    context_image_id: None,
                }
            )
            .is_ok());
    }
}
