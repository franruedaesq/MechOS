//! [`CapabilityManager`] – enforces the principle of least privilege.
//!
//! Before any tool or hardware is invoked, call [`CapabilityManager::check`]
//! to verify the requesting agent holds the required [`Capability`].  If the
//! check fails a [`MechError::Unauthorized`] is returned and the action must
//! not be executed.

use std::collections::{HashMap, HashSet};

use mechos_types::{Capability, MechError};

/// Manages the set of [`Capability`] grants for each registered agent.
///
/// # Example
///
/// ```
/// use mechos_kernel::capability_manager::CapabilityManager;
/// use mechos_types::{Capability, MechError};
///
/// let mut mgr = CapabilityManager::new();
/// mgr.grant("agent_a", Capability::ModelInference);
///
/// assert!(mgr.check("agent_a", &Capability::ModelInference).is_ok());
/// assert!(mgr.check("agent_a", &Capability::SensorRead("lidar".into())).is_err());
/// ```
#[derive(Default)]
pub struct CapabilityManager {
    grants: HashMap<String, HashSet<Capability>>,
}

impl CapabilityManager {
    /// Create an empty manager with no grants.
    pub fn new() -> Self {
        Self::default()
    }

    /// Grant `cap` to `agent_id`.  Duplicate grants are silently ignored.
    pub fn grant(&mut self, agent_id: &str, cap: Capability) {
        self.grants
            .entry(agent_id.to_string())
            .or_default()
            .insert(cap);
    }

    /// Revoke `cap` from `agent_id`.  No-ops if the agent or capability is not
    /// present.
    pub fn revoke(&mut self, agent_id: &str, cap: &Capability) {
        if let Some(set) = self.grants.get_mut(agent_id) {
            set.remove(cap);
        }
    }

    /// Return `Ok(())` when `agent_id` holds `cap`, or
    /// [`MechError::Unauthorized`] otherwise.
    pub fn check(&self, agent_id: &str, cap: &Capability) -> Result<(), MechError> {
        let has = self
            .grants
            .get(agent_id)
            .map(|s| s.contains(cap))
            .unwrap_or(false);
        if has {
            Ok(())
        } else {
            Err(MechError::Unauthorized(cap.clone()))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_capability_manager_grant() {
        let mut mgr = CapabilityManager::new();
        mgr.grant("test_agent", Capability::ModelInference);
        assert!(mgr.check("test_agent", &Capability::ModelInference).is_ok());
    }

    #[test]
    fn grant_and_check_passes() {
        let mut mgr = CapabilityManager::new();
        mgr.grant("robot_agent", Capability::ModelInference);
        assert!(mgr
            .check("robot_agent", &Capability::ModelInference)
            .is_ok());
    }

    #[test]
    fn ungrant_capability_is_denied() {
        let mgr = CapabilityManager::new();
        let result = mgr.check("robot_agent", &Capability::ModelInference);
        assert!(matches!(result, Err(MechError::Unauthorized(_))));
    }

    #[test]
    fn unknown_agent_is_denied() {
        let mut mgr = CapabilityManager::new();
        mgr.grant("agent_a", Capability::ModelInference);
        let result = mgr.check("agent_b", &Capability::ModelInference);
        assert!(matches!(result, Err(MechError::Unauthorized(_))));
    }

    #[test]
    fn revoke_removes_capability() {
        let mut mgr = CapabilityManager::new();
        mgr.grant("robot_agent", Capability::ModelInference);
        mgr.revoke("robot_agent", &Capability::ModelInference);
        let result = mgr.check("robot_agent", &Capability::ModelInference);
        assert!(matches!(result, Err(MechError::Unauthorized(_))));
    }

    #[test]
    fn revoke_nonexistent_is_noop() {
        let mut mgr = CapabilityManager::new();
        // Should not panic even when agent or cap was never granted.
        mgr.revoke("ghost", &Capability::ModelInference);
    }

    #[test]
    fn hardware_invoke_capability_distinguished_by_id() {
        let mut mgr = CapabilityManager::new();
        mgr.grant("robot_agent", Capability::HardwareInvoke("drive_base".into()));

        assert!(mgr
            .check(
                "robot_agent",
                &Capability::HardwareInvoke("drive_base".into())
            )
            .is_ok());
        // Different hardware id must be denied.
        assert!(mgr
            .check(
                "robot_agent",
                &Capability::HardwareInvoke("arm_joint_1".into())
            )
            .is_err());
    }

    #[test]
    fn multiple_capabilities_independent() {
        let mut mgr = CapabilityManager::new();
        mgr.grant("robot_agent", Capability::ModelInference);
        mgr.grant("robot_agent", Capability::SensorRead("lidar".into()));

        assert!(mgr
            .check("robot_agent", &Capability::ModelInference)
            .is_ok());
        assert!(mgr
            .check("robot_agent", &Capability::SensorRead("lidar".into()))
            .is_ok());
        assert!(mgr
            .check("robot_agent", &Capability::MemoryAccess("episodic".into()))
            .is_err());
    }

    #[test]
    fn duplicate_grant_is_idempotent() {
        let mut mgr = CapabilityManager::new();
        mgr.grant("robot_agent", Capability::ModelInference);
        mgr.grant("robot_agent", Capability::ModelInference);
        assert!(mgr
            .check("robot_agent", &Capability::ModelInference)
            .is_ok());
        // Revoke once should still remove the capability.
        mgr.revoke("robot_agent", &Capability::ModelInference);
        assert!(mgr
            .check("robot_agent", &Capability::ModelInference)
            .is_err());
    }
}
