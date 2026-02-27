//! [`LoopGuard`] – repetitive-action loop detector.
//!
//! Detects when the LLM is stuck requesting the same (or the same short
//! sequence of) actions repeatedly, and signals that an intervention is
//! required before the system wastes further resources or damages hardware.
//!
//! # Algorithm
//!
//! The guard maintains a rolling window of the last *N* action labels.
//! After each [`LoopGuard::record`] call it checks whether the most recent
//! `threshold` entries are all identical.  If they are, [`record`] returns
//! `true` – meaning the caller should stop the current plan and attempt a
//! recovery strategy.
//!
//! # Example
//!
//! ```rust
//! use mechos_runtime::loop_guard::LoopGuard;
//!
//! let mut guard = LoopGuard::new(3);
//!
//! assert!(!guard.record("drive_forward"));
//! assert!(!guard.record("drive_forward"));
//! assert!(guard.record("drive_forward")); // third repeat → stuck!
//!
//! // Resetting clears the history.
//! guard.reset();
//! assert!(!guard.record("drive_forward"));
//! ```

use std::collections::VecDeque;

// ─────────────────────────────────────────────────────────────────────────────
// LoopGuard
// ─────────────────────────────────────────────────────────────────────────────

/// Detects when the LLM is stuck in a repetitive action loop.
///
/// Construct with [`LoopGuard::new`], passing the number of consecutive
/// identical actions that constitute a "stuck" condition.
pub struct LoopGuard {
    /// Number of consecutive identical records that triggers detection.
    threshold: usize,
    /// Rolling window of the most recent action labels.
    history: VecDeque<String>,
}

impl LoopGuard {
    /// Create a new guard.
    ///
    /// `threshold` is the minimum number of consecutive identical action
    /// labels that will be flagged as a loop.  A `threshold` of 1 would
    /// trigger on every action; values ≥ 2 are recommended.
    pub fn new(threshold: usize) -> Self {
        Self {
            threshold,
            history: VecDeque::with_capacity(threshold),
        }
    }

    /// Record that `action` was just requested by the LLM.
    ///
    /// Returns `true` when the same action has been recorded `threshold` times
    /// in a row, indicating the system is stuck and an intervention is
    /// required.  Returns `false` otherwise.
    pub fn record(&mut self, action: &str) -> bool {
        self.history.push_back(action.to_string());
        // Keep only the last `threshold` entries.
        while self.history.len() > self.threshold {
            self.history.pop_front();
        }
        self.is_stuck()
    }

    /// Return `true` if the current history constitutes a loop.
    ///
    /// This is `true` when the window is full *and* every entry is identical.
    pub fn is_stuck(&self) -> bool {
        if self.history.len() < self.threshold {
            return false;
        }
        let first = &self.history[0];
        self.history.iter().all(|a| a == first)
    }

    /// Clear all recorded history, resetting the guard to its initial state.
    pub fn reset(&mut self) {
        self.history.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_loop_with_varied_actions() {
        let mut guard = LoopGuard::new(3);
        assert!(!guard.record("move_forward"));
        assert!(!guard.record("turn_left"));
        assert!(!guard.record("move_forward"));
    }

    #[test]
    fn detects_loop_at_threshold() {
        let mut guard = LoopGuard::new(3);
        assert!(!guard.record("drive_forward"));
        assert!(!guard.record("drive_forward"));
        assert!(guard.record("drive_forward")); // third repeat
    }

    #[test]
    fn does_not_trigger_below_threshold() {
        let mut guard = LoopGuard::new(4);
        assert!(!guard.record("action_a"));
        assert!(!guard.record("action_a"));
        assert!(!guard.record("action_a")); // only 3, threshold is 4
    }

    #[test]
    fn loop_continues_to_be_detected_after_threshold() {
        let mut guard = LoopGuard::new(3);
        guard.record("action_x");
        guard.record("action_x");
        assert!(guard.record("action_x"));
        assert!(guard.record("action_x")); // still stuck
    }

    #[test]
    fn reset_clears_history() {
        let mut guard = LoopGuard::new(3);
        guard.record("action_a");
        guard.record("action_a");
        assert!(guard.record("action_a")); // stuck
        guard.reset();
        assert!(!guard.record("action_a")); // history wiped
    }

    #[test]
    fn different_action_breaks_streak() {
        let mut guard = LoopGuard::new(3);
        guard.record("action_a");
        guard.record("action_a");
        guard.record("action_b"); // breaks the streak
        assert!(!guard.is_stuck());
    }

    #[test]
    fn window_slides_correctly() {
        let mut guard = LoopGuard::new(3);
        // Fill with alternating actions.
        guard.record("a");
        guard.record("b");
        guard.record("a");
        assert!(!guard.is_stuck());
        // Now push three identical actions; window should slide past the earlier ones.
        guard.record("c");
        guard.record("c");
        assert!(guard.record("c")); // window is now [c, c, c]
    }

    #[test]
    fn threshold_one_triggers_immediately() {
        let mut guard = LoopGuard::new(1);
        assert!(guard.record("any_action")); // threshold 1 → first record triggers
    }
}
