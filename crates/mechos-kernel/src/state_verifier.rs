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
//! - [`JointLimitRule`] – rejects `ActuateJoint` commands that would move a
//!   named joint outside its safe angular range.

use mechos_types::{HardwareIntent, MechError};

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

/// Rejects [`HardwareIntent::ActuateJoint`] commands that would move a
/// specific joint outside its `[min_rad, max_rad]` safe range.
pub struct JointLimitRule {
    /// The joint this rule applies to.
    pub joint_id: String,
    /// Minimum allowed angle in radians (inclusive).
    pub min_rad: f32,
    /// Maximum allowed angle in radians (inclusive).
    pub max_rad: f32,
}

impl Rule for JointLimitRule {
    fn name(&self) -> &str {
        "joint_limit"
    }

    fn check(&self, intent: &HardwareIntent) -> Result<(), MechError> {
        if let HardwareIntent::ActuateJoint {
            joint_id,
            target_angle_rad,
        } = intent
        {
            if joint_id == &self.joint_id
                && (*target_angle_rad < self.min_rad || *target_angle_rad > self.max_rad)
            {
                return Err(MechError::HardwareFault {
                    component: joint_id.clone(),
                    details: format!(
                        "target_angle_rad {target_angle_rad} out of [{}, {}]",
                        self.min_rad, self.max_rad
                    ),
                });
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ------------------------------------------------------------------ helpers
    fn speed_verifier(max_linear: f32, max_angular: f32) -> StateVerifier {
        let mut v = StateVerifier::new();
        v.add_rule(Box::new(SpeedCapRule {
            max_linear,
            max_angular,
        }));
        v
    }

    fn joint_verifier(joint_id: &str, min_rad: f32, max_rad: f32) -> StateVerifier {
        let mut v = StateVerifier::new();
        v.add_rule(Box::new(JointLimitRule {
            joint_id: joint_id.to_string(),
            min_rad,
            max_rad,
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
    fn speed_cap_does_not_apply_to_joint_intents() {
        let v = speed_verifier(1.0, 1.0);
        // ActuateJoint is irrelevant to the speed cap rule.
        assert!(v
            .verify(&HardwareIntent::ActuateJoint {
                joint_id: "arm".into(),
                target_angle_rad: 999.0
            })
            .is_ok());
    }

    // ------------------------------------------------------------------ JointLimitRule

    #[test]
    fn joint_within_limits_passes() {
        let v = joint_verifier("arm_joint_1", -1.57, 1.57);
        assert!(v
            .verify(&HardwareIntent::ActuateJoint {
                joint_id: "arm_joint_1".into(),
                target_angle_rad: 0.0
            })
            .is_ok());
    }

    #[test]
    fn joint_at_limit_boundary_passes() {
        let v = joint_verifier("arm_joint_1", -1.57, 1.57);
        assert!(v
            .verify(&HardwareIntent::ActuateJoint {
                joint_id: "arm_joint_1".into(),
                target_angle_rad: 1.57
            })
            .is_ok());
    }

    #[test]
    fn joint_over_max_rejected() {
        let v = joint_verifier("arm_joint_1", -1.57, 1.57);
        assert!(matches!(
            v.verify(&HardwareIntent::ActuateJoint {
                joint_id: "arm_joint_1".into(),
                target_angle_rad: 2.0
            }),
            Err(MechError::HardwareFault { .. })
        ));
    }

    #[test]
    fn joint_under_min_rejected() {
        let v = joint_verifier("arm_joint_1", -1.57, 1.57);
        assert!(matches!(
            v.verify(&HardwareIntent::ActuateJoint {
                joint_id: "arm_joint_1".into(),
                target_angle_rad: -2.0
            }),
            Err(MechError::HardwareFault { .. })
        ));
    }

    #[test]
    fn joint_rule_does_not_apply_to_other_joints() {
        let v = joint_verifier("arm_joint_1", -1.57, 1.57);
        // arm_joint_2 has no rule, so any angle is fine.
        assert!(v
            .verify(&HardwareIntent::ActuateJoint {
                joint_id: "arm_joint_2".into(),
                target_angle_rad: 999.0
            })
            .is_ok());
    }

    #[test]
    fn joint_rule_does_not_apply_to_drive_intents() {
        let v = joint_verifier("arm_joint_1", -1.57, 1.57);
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
        v.add_rule(Box::new(JointLimitRule {
            joint_id: "arm_joint_1".into(),
            min_rad: -1.0,
            max_rad: 1.0,
        }));

        // Speed cap fires first even though the joint rule is also registered.
        let result = v.verify(&HardwareIntent::Drive {
            linear_velocity: 5.0,
            angular_velocity: 0.0,
        });
        assert!(matches!(result, Err(MechError::HardwareFault { ref component, .. }) if component == "drive_base"));
    }

    #[test]
    fn empty_verifier_always_passes() {
        let v = StateVerifier::new();
        assert!(v.verify(&HardwareIntent::EmergencyStop).is_ok());
        assert!(v
            .verify(&HardwareIntent::Drive {
                linear_velocity: 999.0,
                angular_velocity: 999.0
            })
            .is_ok());
    }

    #[test]
    fn emergency_stop_passes_all_rules() {
        let v = speed_verifier(0.0, 0.0); // zero cap – nothing allowed
        // EmergencyStop is special and must always pass.
        assert!(v.verify(&HardwareIntent::EmergencyStop).is_ok());
    }
}
