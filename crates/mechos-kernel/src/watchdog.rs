//! [`Watchdog`] – component health monitor.
//!
//! Every subsystem (e.g., the HAL bridge, the LLM driver, the perception
//! pipeline) should call [`Watchdog::heartbeat`] at regular intervals.  The
//! watchdog tracks the timestamp of each heartbeat and considers a component
//! *frozen* when its deadline has been exceeded.
//!
//! Call [`Watchdog::check_all`] from a supervisor loop to obtain the list of
//! frozen component IDs so that restart logic can be applied.

use std::collections::HashMap;
use std::time::{Duration, Instant};

// ────────────────────────────────────────────────────────────────────────────
// Public types
// ────────────────────────────────────────────────────────────────────────────

/// Health state reported for a single component.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ComponentHealth {
    /// The component has sent a heartbeat within its deadline.
    Healthy,
    /// The component has not sent a heartbeat within its deadline.
    TimedOut,
}

// ────────────────────────────────────────────────────────────────────────────
// Internal entry
// ────────────────────────────────────────────────────────────────────────────

struct ComponentEntry {
    last_heartbeat: Instant,
    timeout: Duration,
}

// ────────────────────────────────────────────────────────────────────────────
// Watchdog
// ────────────────────────────────────────────────────────────────────────────

/// Tracks heartbeats from registered subsystems and detects frozen components.
///
/// # Example
///
/// ```
/// use std::time::Duration;
/// use mechos_kernel::watchdog::{Watchdog, ComponentHealth};
///
/// let mut wd = Watchdog::new();
/// wd.register("perception", Duration::from_secs(1));
/// wd.heartbeat("perception");
///
/// assert_eq!(wd.health("perception"), ComponentHealth::Healthy);
/// ```
#[derive(Default)]
pub struct Watchdog {
    components: HashMap<String, ComponentEntry>,
}

impl Watchdog {
    /// Create an empty watchdog with no registered components.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register `component_id` with a maximum heartbeat `timeout`.
    ///
    /// The component's last-heartbeat timestamp is initialised to now, so it
    /// starts in a [`ComponentHealth::Healthy`] state.
    ///
    /// Re-registering an existing component resets its deadline.
    pub fn register(&mut self, component_id: &str, timeout: Duration) {
        self.components.insert(
            component_id.to_string(),
            ComponentEntry {
                last_heartbeat: Instant::now(),
                timeout,
            },
        );
    }

    /// Record a heartbeat for `component_id`, resetting its deadline.
    ///
    /// No-ops for components that have not been registered.
    pub fn heartbeat(&mut self, component_id: &str) {
        if let Some(entry) = self.components.get_mut(component_id) {
            entry.last_heartbeat = Instant::now();
        }
    }

    /// Return the [`ComponentHealth`] of `component_id`.
    ///
    /// Returns [`ComponentHealth::TimedOut`] for unknown components.
    pub fn health(&self, component_id: &str) -> ComponentHealth {
        match self.components.get(component_id) {
            Some(entry) if entry.last_heartbeat.elapsed() <= entry.timeout => {
                ComponentHealth::Healthy
            }
            _ => ComponentHealth::TimedOut,
        }
    }

    /// Return the IDs of all components whose heartbeat deadline has been
    /// exceeded.  The order of the returned list is unspecified.
    pub fn check_all(&self) -> Vec<String> {
        self.components
            .iter()
            .filter(|(_, entry)| entry.last_heartbeat.elapsed() > entry.timeout)
            .map(|(id, _)| id.clone())
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn fresh_component_is_healthy() {
        let mut wd = Watchdog::new();
        wd.register("perception", Duration::from_secs(5));
        assert_eq!(wd.health("perception"), ComponentHealth::Healthy);
    }

    #[test]
    fn heartbeat_resets_deadline() {
        let mut wd = Watchdog::new();
        // Very short timeout.
        wd.register("sensor_bridge", Duration::from_millis(20));
        thread::sleep(Duration::from_millis(10));
        // Heartbeat before deadline.
        wd.heartbeat("sensor_bridge");
        thread::sleep(Duration::from_millis(10));
        // Still alive because of the recent heartbeat.
        assert_eq!(wd.health("sensor_bridge"), ComponentHealth::Healthy);
    }

    #[test]
    fn component_times_out_when_silent() {
        let mut wd = Watchdog::new();
        wd.register("llm_driver", Duration::from_millis(20));
        thread::sleep(Duration::from_millis(30));
        assert_eq!(wd.health("llm_driver"), ComponentHealth::TimedOut);
    }

    #[test]
    fn check_all_returns_frozen_components() {
        let mut wd = Watchdog::new();
        wd.register("fast_component", Duration::from_millis(20));
        wd.register("slow_component", Duration::from_secs(60));

        thread::sleep(Duration::from_millis(30));

        let frozen = wd.check_all();
        assert_eq!(frozen.len(), 1);
        assert_eq!(frozen[0], "fast_component");
    }

    #[test]
    fn check_all_empty_when_all_healthy() {
        let mut wd = Watchdog::new();
        wd.register("component_a", Duration::from_secs(60));
        wd.register("component_b", Duration::from_secs(60));
        assert!(wd.check_all().is_empty());
    }

    #[test]
    fn unknown_component_health_is_timed_out() {
        let wd = Watchdog::new();
        assert_eq!(wd.health("ghost"), ComponentHealth::TimedOut);
    }

    #[test]
    fn heartbeat_on_unknown_component_is_noop() {
        let mut wd = Watchdog::new();
        // Should not panic.
        wd.heartbeat("ghost");
    }

    #[test]
    fn reregister_resets_timer() {
        let mut wd = Watchdog::new();
        wd.register("comp", Duration::from_millis(20));
        thread::sleep(Duration::from_millis(30));
        // Already timed out.
        assert_eq!(wd.health("comp"), ComponentHealth::TimedOut);
        // Re-register should reset the deadline.
        wd.register("comp", Duration::from_secs(60));
        assert_eq!(wd.health("comp"), ComponentHealth::Healthy);
    }
}
