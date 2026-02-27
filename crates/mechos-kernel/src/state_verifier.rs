//! [`StateVerifier`] – physical safety interlock / rule engine.
//!
//! Before a [`HardwareIntent`] is dispatched to the HAL, pass it through
//! [`StateVerifier::verify`].  Every registered [`Rule`] is evaluated in
//! order; the first violation returns a [`MechError::HardwareFault`] and the
//! intent is **not** executed.
//!
//! Two built-in rules are provided:
//! - [`SpeedCapRule`] – rejects `Drive` commands whose linear or angular
//!   velocities exceed configured caps.
//! - [`EndEffectorWorkspaceRule`] – rejects `MoveEndEffector` commands that
//!   place the end-effector outside its safe cubic workspace.

use mechos_types::{HardwareIntent, MechError};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

// ────────────────────────────────────────────────────────────────────────────
// Rule trait
// ────────────────────────────────────────────────────────────────────────────

/// A single physical invariant that an intent must satisfy.
///
/// Implement this trait to create custom safety rules and add them to a
/// [`StateVerifier`] via [`StateVerifier::add_rule`].
pub trait Rule: Send + Sync {
    /// Human-readable name used in fault messages.
    fn name(&self) -> &str;

    /// Return `Ok(())` when the intent satisfies the invariant, or
    /// [`MechError::HardwareFault`] when it is violated.
    fn check(&self, intent: &HardwareIntent) -> Result<(), MechError>;
}

// ────────────────────────────────────────────────────────────────────────────
// StateVerifier
// ────────────────────────────────────────────────────────────────────────────

/// Rule engine that validates a [`HardwareIntent`] against all registered
/// [`Rule`]s before it is dispatched.
///
/// # Example
///
/// ```
/// use mechos_kernel::state_verifier::{StateVerifier, SpeedCapRule};
/// use mechos_types::HardwareIntent;
///
/// let mut verifier = StateVerifier::new();
/// verifier.add_rule(Box::new(SpeedCapRule { max_linear: 1.0, max_angular: 1.0 }));
///
/// let safe = HardwareIntent::Drive { linear_velocity: 0.5, angular_velocity: 0.2 };
/// assert!(verifier.verify(&safe).is_ok());
///
/// let too_fast = HardwareIntent::Drive { linear_velocity: 2.0, angular_velocity: 0.0 };
/// assert!(verifier.verify(&too_fast).is_err());
/// ```
#[derive(Default)]
pub struct StateVerifier {
    rules: Vec<Box<dyn Rule>>,
}

impl StateVerifier {
    /// Create an empty verifier with no rules.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a new [`Rule`].  Rules are evaluated in insertion order.
    pub fn add_rule(&mut self, rule: Box<dyn Rule>) {
        self.rules.push(rule);
    }

    /// Validate `intent` against every registered rule.
    ///
    /// Returns the first [`MechError::HardwareFault`] encountered, or `Ok(())`
    /// when all rules pass.
    pub fn verify(&self, intent: &HardwareIntent) -> Result<(), MechError> {
        for rule in &self.rules {
            rule.check(intent)?;
        }
        Ok(())
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Built-in rules
// ────────────────────────────────────────────────────────────────────────────

/// Rejects [`HardwareIntent::Drive`] commands whose `linear_velocity` or
/// `angular_velocity` magnitudes exceed configured caps.
pub struct SpeedCapRule {
    /// Maximum allowed absolute linear velocity (m/s or equivalent units).
    pub max_linear: f32,
    /// Maximum allowed absolute angular velocity (rad/s or equivalent units).
    pub max_angular: f32,
}

impl Rule for SpeedCapRule {
    fn name(&self) -> &str {
        "speed_cap"
    }

    fn check(&self, intent: &HardwareIntent) -> Result<(), MechError> {
        if let HardwareIntent::Drive {
            linear_velocity,
            angular_velocity,
        } = intent
        {
            if linear_velocity.abs() > self.max_linear {
                return Err(MechError::HardwareFault {
                    component: "drive_base".to_string(),
                    details: format!(
                        "linear_velocity {linear_velocity} exceeds cap {}",
                        self.max_linear
                    ),
                });
            }
            if angular_velocity.abs() > self.max_angular {
                return Err(MechError::HardwareFault {
                    component: "drive_base".to_string(),
                    details: format!(
                        "angular_velocity {angular_velocity} exceeds cap {}",
                        self.max_angular
                    ),
                });
            }
        }
        Ok(())
    }
}

/// Rejects [`HardwareIntent::MoveEndEffector`] commands that would place the
/// end-effector outside its safe cubic workspace `[min, max]` on each axis.
pub struct EndEffectorWorkspaceRule {
    /// Minimum allowed X coordinate (metres).
    pub min_x: f32,
    /// Maximum allowed X coordinate (metres).
    pub max_x: f32,
    /// Minimum allowed Y coordinate (metres).
    pub min_y: f32,
    /// Maximum allowed Y coordinate (metres).
    pub max_y: f32,
    /// Minimum allowed Z coordinate (metres).
    pub min_z: f32,
    /// Maximum allowed Z coordinate (metres).
    pub max_z: f32,
}

impl Rule for EndEffectorWorkspaceRule {
    fn name(&self) -> &str {
        "end_effector_workspace"
    }

    fn check(&self, intent: &HardwareIntent) -> Result<(), MechError> {
        if let HardwareIntent::MoveEndEffector { x, y, z } = intent {
            for (axis, val, min, max) in [
                ("x", x, &self.min_x, &self.max_x),
                ("y", y, &self.min_y, &self.max_y),
                ("z", z, &self.min_z, &self.max_z),
            ] {
                if *val < *min || *val > *max {
                    return Err(MechError::HardwareFault {
                        component: "end_effector".to_string(),
                        details: format!(
                            "{axis}={val} out of [{min}, {max}]"
                        ),
                    });
                }
            }
        }
        Ok(())
    }
}

/// Safety interlock that blocks AI-issued [`HardwareIntent::Drive`] commands
/// while a manual dashboard override session is active.
///
/// The interlock is armed and disarmed through the shared `active` flag.  When
/// the flag is `true` (the human operator has grabbed the on-screen joystick),
/// any AI-sourced `Drive` command is rejected so the LLM cannot fight the
/// human for control of the motors.  All other intent variants pass through
/// unaffected.
///
/// # Example
///
/// ```
/// use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
/// use mechos_kernel::{ManualOverrideInterlock, StateVerifier};
/// use mechos_types::HardwareIntent;
///
/// let flag = Arc::new(AtomicBool::new(false));
/// let mut verifier = StateVerifier::new();
/// verifier.add_rule(Box::new(ManualOverrideInterlock::new(Arc::clone(&flag))));
///
/// // Override not active – Drive passes.
/// assert!(verifier.verify(&HardwareIntent::Drive {
///     linear_velocity: 0.5, angular_velocity: 0.0,
/// }).is_ok());
///
/// // Arm the interlock.
/// flag.store(true, Ordering::Release);
///
/// // Override active – Drive is rejected.
/// assert!(verifier.verify(&HardwareIntent::Drive {
///     linear_velocity: 0.5, angular_velocity: 0.0,
/// }).is_err());
/// ```
pub struct ManualOverrideInterlock {
    /// `true` while a dashboard manual-override session is active.
    pub active: Arc<AtomicBool>,
}

impl ManualOverrideInterlock {
    /// Create a new interlock that shares the given `active` flag.
    pub fn new(active: Arc<AtomicBool>) -> Self {
        Self { active }
    }
}

impl Rule for ManualOverrideInterlock {
    fn name(&self) -> &str {
        "manual_override_interlock"
    }

    /// Reject any [`HardwareIntent::Drive`] command while the override flag is
    /// set.  All other intent variants always pass this rule.
    fn check(&self, intent: &HardwareIntent) -> Result<(), MechError> {
        if self.active.load(Ordering::Acquire) {
            if let HardwareIntent::Drive { .. } = intent {
                return Err(MechError::HardwareFault {
                    component: "drive_base".to_string(),
                    details: "manual override active; AI drive commands suspended".to_string(),
                });
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::Ordering;

    // ------------------------------------------------------------------ helpers
    fn speed_verifier(max_linear: f32, max_angular: f32) -> StateVerifier {
        let mut v = StateVerifier::new();
        v.add_rule(Box::new(SpeedCapRule {
            max_linear,
            max_angular,
        }));
        v
    }

    fn workspace_verifier(
        min_x: f32,
        max_x: f32,
        min_y: f32,
        max_y: f32,
        min_z: f32,
        max_z: f32,
    ) -> StateVerifier {
        let mut v = StateVerifier::new();
        v.add_rule(Box::new(EndEffectorWorkspaceRule {
            min_x,
            max_x,
            min_y,
            max_y,
            min_z,
            max_z,
        }));
        v
    }

    // ------------------------------------------------------------------ SpeedCapRule

    #[test]
    fn drive_within_caps_passes() {
        let v = speed_verifier(1.0, 1.0);
        assert!(v
            .verify(&HardwareIntent::Drive {
                linear_velocity: 0.5,
                angular_velocity: 0.5
            })
            .is_ok());
    }

    #[test]
    fn drive_at_cap_boundary_passes() {
        let v = speed_verifier(1.0, 1.0);
        assert!(v
            .verify(&HardwareIntent::Drive {
                linear_velocity: 1.0,
                angular_velocity: 1.0
            })
            .is_ok());
    }

    #[test]
    fn drive_linear_over_cap_rejected() {
        let v = speed_verifier(1.0, 1.0);
        assert!(matches!(
            v.verify(&HardwareIntent::Drive {
                linear_velocity: 1.1,
                angular_velocity: 0.0
            }),
            Err(MechError::HardwareFault { .. })
        ));
    }

    #[test]
    fn drive_angular_over_cap_rejected() {
        let v = speed_verifier(1.0, 1.0);
        assert!(matches!(
            v.verify(&HardwareIntent::Drive {
                linear_velocity: 0.0,
                angular_velocity: 1.5
            }),
            Err(MechError::HardwareFault { .. })
        ));
    }

    #[test]
    fn drive_negative_linear_over_cap_rejected() {
        let v = speed_verifier(1.0, 1.0);
        assert!(matches!(
            v.verify(&HardwareIntent::Drive {
                linear_velocity: -2.0,
                angular_velocity: 0.0
            }),
            Err(MechError::HardwareFault { .. })
        ));
    }

    #[test]
    fn speed_cap_does_not_apply_to_end_effector_intents() {
        let v = speed_verifier(1.0, 1.0);
        // MoveEndEffector is irrelevant to the speed cap rule.
        assert!(v
            .verify(&HardwareIntent::MoveEndEffector {
                x: 999.0,
                y: 999.0,
                z: 999.0,
            })
            .is_ok());
    }

    // ------------------------------------------------------------------ EndEffectorWorkspaceRule

    #[test]
    fn end_effector_within_workspace_passes() {
        let v = workspace_verifier(-1.0, 1.0, -1.0, 1.0, 0.0, 2.0);
        assert!(v
            .verify(&HardwareIntent::MoveEndEffector {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            })
            .is_ok());
    }

    #[test]
    fn end_effector_at_workspace_boundary_passes() {
        let v = workspace_verifier(-1.0, 1.0, -1.0, 1.0, 0.0, 2.0);
        assert!(v
            .verify(&HardwareIntent::MoveEndEffector {
                x: 1.0,
                y: -1.0,
                z: 2.0,
            })
            .is_ok());
    }

    #[test]
    fn end_effector_x_over_max_rejected() {
        let v = workspace_verifier(-1.0, 1.0, -1.0, 1.0, 0.0, 2.0);
        assert!(matches!(
            v.verify(&HardwareIntent::MoveEndEffector {
                x: 1.5,
                y: 0.0,
                z: 1.0,
            }),
            Err(MechError::HardwareFault { .. })
        ));
    }

    #[test]
    fn end_effector_z_below_min_rejected() {
        let v = workspace_verifier(-1.0, 1.0, -1.0, 1.0, 0.0, 2.0);
        assert!(matches!(
            v.verify(&HardwareIntent::MoveEndEffector {
                x: 0.0,
                y: 0.0,
                z: -0.1,
            }),
            Err(MechError::HardwareFault { .. })
        ));
    }

    #[test]
    fn workspace_rule_does_not_apply_to_drive_intents() {
        let v = workspace_verifier(-1.0, 1.0, -1.0, 1.0, 0.0, 2.0);
        assert!(v
            .verify(&HardwareIntent::Drive {
                linear_velocity: 0.0,
                angular_velocity: 0.0
            })
            .is_ok());
    }

    // ------------------------------------------------------------------ Multiple rules

    #[test]
    fn first_failing_rule_short_circuits() {
        let mut v = StateVerifier::new();
        v.add_rule(Box::new(SpeedCapRule {
            max_linear: 1.0,
            max_angular: 1.0,
        }));
        v.add_rule(Box::new(EndEffectorWorkspaceRule {
            min_x: -1.0,
            max_x: 1.0,
            min_y: -1.0,
            max_y: 1.0,
            min_z: 0.0,
            max_z: 2.0,
        }));

        // Speed cap fires first even though the workspace rule is also registered.
        let result = v.verify(&HardwareIntent::Drive {
            linear_velocity: 5.0,
            angular_velocity: 0.0,
        });
        assert!(matches!(result, Err(MechError::HardwareFault { ref component, .. }) if component == "drive_base"));
    }

    #[test]
    fn empty_verifier_always_passes() {
        let v = StateVerifier::new();
        assert!(v
            .verify(&HardwareIntent::AskHuman {
                question: "Help?".to_string(),
                context_image_id: None,
            })
            .is_ok());
        assert!(v
            .verify(&HardwareIntent::Drive {
                linear_velocity: 999.0,
                angular_velocity: 999.0
            })
            .is_ok());
    }

    #[test]
    fn ask_human_passes_all_rules() {
        let v = speed_verifier(0.0, 0.0); // zero cap – nothing allowed for Drive
        // AskHuman is not a physical motion and must always pass.
        assert!(v
            .verify(&HardwareIntent::AskHuman {
                question: "What should I do?".to_string(),
                context_image_id: None,
            })
            .is_ok());
    }

    // ------------------------------------------------------------------ ManualOverrideInterlock

    fn override_verifier(active: Arc<AtomicBool>) -> StateVerifier {
        let mut v = StateVerifier::new();
        v.add_rule(Box::new(ManualOverrideInterlock::new(active)));
        v
    }

    #[test]
    fn drive_passes_when_override_not_active() {
        let flag = Arc::new(AtomicBool::new(false));
        let v = override_verifier(Arc::clone(&flag));
        assert!(v
            .verify(&HardwareIntent::Drive {
                linear_velocity: 0.5,
                angular_velocity: 0.0,
            })
            .is_ok());
    }

    #[test]
    fn drive_rejected_when_override_active() {
        let flag = Arc::new(AtomicBool::new(true));
        let v = override_verifier(Arc::clone(&flag));
        assert!(matches!(
            v.verify(&HardwareIntent::Drive {
                linear_velocity: 0.5,
                angular_velocity: 0.0,
            }),
            Err(MechError::HardwareFault { ref details, .. })
                if details.contains("manual override active")
        ));
    }

    #[test]
    fn ask_human_passes_when_override_active() {
        let flag = Arc::new(AtomicBool::new(true));
        let v = override_verifier(Arc::clone(&flag));
        assert!(v
            .verify(&HardwareIntent::AskHuman {
                question: "Override active – what should I do?".to_string(),
                context_image_id: None,
            })
            .is_ok());
    }

    #[test]
    fn override_interlock_cleared_when_flag_reset() {
        let flag = Arc::new(AtomicBool::new(true));
        let v = override_verifier(Arc::clone(&flag));
        // Active: Drive is blocked.
        assert!(v
            .verify(&HardwareIntent::Drive {
                linear_velocity: 0.3,
                angular_velocity: 0.0,
            })
            .is_err());
        // Clear the flag – Drive should pass again.
        flag.store(false, Ordering::Release);
        assert!(v
            .verify(&HardwareIntent::Drive {
                linear_velocity: 0.3,
                angular_velocity: 0.0,
            })
            .is_ok());
    }
}
