//! Semantic Vector State Estimator.
//!
//! Tracks the semantic state of named entities (objects, locations, concepts)
//! in the robot's environment over time by maintaining a fused confidence
//! score that is updated with new visual/sensor embeddings and decays
//! exponentially between observations.
//!
//! ## Model
//!
//! Each tracked entity is identified by a `label` string (e.g.
//! `"coffee_mug"`, `"charging_dock"`).  Its [`SemanticState`] consists of:
//!
//! * A **mean embedding** – an online weighted average of all observed
//!   embeddings, giving the estimator a stable centroid to compare against.
//! * A **confidence** value in `[0.0, 1.0]` that rises when the entity is
//!   freshly observed and decays toward zero the longer it goes unseen.
//!
//! ### Time-decay
//!
//! Every call to [`SemanticStateEstimator::decay_all`] multiplies every
//! entity's confidence by `decay_factor ∈ (0, 1)`, modelling how certainty
//! about the world erodes as the robot has not checked for that entity
//! recently:
//!
//! ```text
//! confidence(t + Δt) = confidence(t) × decay_factor^(Δt / tick_period)
//! ```
//!
//! A single call to `decay_all` corresponds to one tick.
//!
//! ### Observation update
//!
//! When [`SemanticStateEstimator::observe`] is called with a new embedding
//! and an observation confidence `obs_conf ∈ [0, 1]`:
//!
//! 1. The entity's mean embedding is updated via an exponential moving
//!    average: `mean = (1 − obs_conf) * mean + obs_conf * new_embedding`
//! 2. The entity's confidence is set to `min(1.0, current + obs_conf)`.
//!
//! # Example
//!
//! ```rust
//! use mechos_memory::semantic::SemanticStateEstimator;
//!
//! let mut est = SemanticStateEstimator::new(0.9);
//!
//! // Observe a "coffee_mug" with its embedding and detection confidence.
//! est.observe("coffee_mug", &[0.2, 0.8, 0.5], 0.85);
//!
//! let state = est.query("coffee_mug").unwrap();
//! assert!((state.confidence - 0.85).abs() < 1e-6);
//!
//! // Confidence decays when the robot has not seen the mug recently.
//! est.decay_all();
//! let state = est.query("coffee_mug").unwrap();
//! assert!(state.confidence < 0.85);
//! ```

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ─────────────────────────────────────────────────────────────────────────────
// SemanticState
// ─────────────────────────────────────────────────────────────────────────────

/// The current belief about a single named entity in the world.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SemanticState {
    /// Identifying label for this entity (e.g. `"coffee_mug"`).
    pub label: String,
    /// Online mean of all observed embedding vectors for this entity.
    pub mean_embedding: Vec<f32>,
    /// Current belief confidence in `[0.0, 1.0]`.
    ///
    /// Increases on fresh observations and decays exponentially between ticks.
    pub confidence: f32,
    /// Total number of times this entity has been observed.
    pub observation_count: u64,
}

impl SemanticState {
    fn new(label: String, embedding: Vec<f32>, confidence: f32) -> Self {
        Self {
            label,
            mean_embedding: embedding,
            confidence: confidence.clamp(0.0, 1.0),
            observation_count: 1,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SemanticStateEstimator
// ─────────────────────────────────────────────────────────────────────────────

/// Fuses past visual/sensor embeddings with a time-decay probability model to
/// maintain an up-to-date belief over the semantic state of all tracked
/// entities.
///
/// Construct with [`SemanticStateEstimator::new`], providing a `decay_factor`
/// in `(0, 1)`.  Feed new observations with
/// [`observe`][SemanticStateEstimator::observe], tick the decay clock with
/// [`decay_all`][SemanticStateEstimator::decay_all], and read beliefs with
/// [`query`][SemanticStateEstimator::query] or
/// [`most_likely_state`][SemanticStateEstimator::most_likely_state].
pub struct SemanticStateEstimator {
    /// Per-tick exponential decay factor applied to every entity's confidence.
    decay_factor: f32,
    states: HashMap<String, SemanticState>,
}

impl SemanticStateEstimator {
    /// Create a new estimator.
    ///
    /// `decay_factor` must be in `(0, 1)` and is clamped to `[0.001, 0.9999]`.
    /// Typical values: `0.9` (slow decay) to `0.5` (fast decay).
    pub fn new(decay_factor: f32) -> Self {
        Self {
            decay_factor: decay_factor.clamp(0.001, 0.9999),
            states: HashMap::new(),
        }
    }

    /// Incorporate a new observation of `label`.
    ///
    /// * `embedding` – the dense embedding vector for this observation.
    /// * `obs_conf`  – the detector's confidence for this observation,
    ///   clamped to `[0, 1]`.
    ///
    /// If the entity is seen for the first time, a new [`SemanticState`] is
    /// created.  Otherwise, the existing mean embedding and confidence are
    /// updated.
    pub fn observe(&mut self, label: &str, embedding: &[f32], obs_conf: f32) {
        let obs_conf = obs_conf.clamp(0.0, 1.0);
        match self.states.get_mut(label) {
            Some(state) => {
                // Exponential moving average of the embedding.
                if state.mean_embedding.len() == embedding.len() {
                    for (m, &e) in state.mean_embedding.iter_mut().zip(embedding) {
                        *m = (1.0 - obs_conf) * *m + obs_conf * e;
                    }
                } else {
                    // Dimension changed – reset to the new embedding.
                    state.mean_embedding = embedding.to_vec();
                }
                state.confidence = (state.confidence + obs_conf).clamp(0.0, 1.0);
                state.observation_count += 1;
            }
            None => {
                self.states.insert(
                    label.to_string(),
                    SemanticState::new(label.to_string(), embedding.to_vec(), obs_conf),
                );
            }
        }
    }

    /// Decay the confidence of every tracked entity by one tick.
    ///
    /// Entities whose confidence falls below a negligible threshold are
    /// **not** automatically removed; use [`prune`][Self::prune] for that.
    pub fn decay_all(&mut self) {
        for state in self.states.values_mut() {
            state.confidence *= self.decay_factor;
        }
    }

    /// Remove all entities whose confidence is below `threshold`.
    ///
    /// Returns the number of entities pruned.
    pub fn prune(&mut self, threshold: f32) -> usize {
        let before = self.states.len();
        self.states.retain(|_, s| s.confidence >= threshold);
        before - self.states.len()
    }

    /// Return a shared reference to the current [`SemanticState`] of `label`,
    /// or `None` if the entity has never been observed.
    pub fn query(&self, label: &str) -> Option<&SemanticState> {
        self.states.get(label)
    }

    /// Return all tracked entity labels sorted by confidence (highest first).
    pub fn all_labels_by_confidence(&self) -> Vec<&str> {
        let mut pairs: Vec<(&str, f32)> = self
            .states
            .values()
            .map(|s| (s.label.as_str(), s.confidence))
            .collect();
        pairs.sort_by(|a, b| b.1.total_cmp(&a.1));
        pairs.into_iter().map(|(l, _)| l).collect()
    }

    /// Return the entity with the highest current confidence, or `None` if no
    /// entities are being tracked.
    pub fn most_likely_state(&self) -> Option<&SemanticState> {
        self.states
            .values()
            .max_by(|a, b| a.confidence.total_cmp(&b.confidence))
    }

    /// Number of entities currently tracked.
    pub fn len(&self) -> usize {
        self.states.len()
    }

    /// Returns `true` if no entities are being tracked.
    pub fn is_empty(&self) -> bool {
        self.states.is_empty()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── observe ──────────────────────────────────────────────────────────────

    #[test]
    fn observe_first_time_creates_state() {
        let mut est = SemanticStateEstimator::new(0.9);
        est.observe("mug", &[1.0, 0.0], 0.7);

        let state = est.query("mug").unwrap();
        assert_eq!(state.label, "mug");
        assert!((state.confidence - 0.7).abs() < 1e-6);
        assert_eq!(state.observation_count, 1);
    }

    #[test]
    fn observe_second_time_updates_confidence_and_embedding() {
        let mut est = SemanticStateEstimator::new(0.9);
        est.observe("mug", &[1.0, 0.0], 0.5);
        est.observe("mug", &[0.0, 1.0], 0.5);

        let state = est.query("mug").unwrap();
        // Confidence clamped to 1.0.
        assert!((state.confidence - 1.0).abs() < 1e-6);
        assert_eq!(state.observation_count, 2);
        // EMA: mean = 0.5 * [1,0] + 0.5 * [0,1] = [0.5, 0.5]
        assert!((state.mean_embedding[0] - 0.5).abs() < 1e-6);
        assert!((state.mean_embedding[1] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn observe_clamps_confidence_to_unit_interval() {
        let mut est = SemanticStateEstimator::new(0.9);
        est.observe("mug", &[1.0], 1.5); // obs_conf clamped to 1.0
        let state = est.query("mug").unwrap();
        assert!((state.confidence - 1.0).abs() < 1e-6);
    }

    #[test]
    fn observe_resets_embedding_on_dimension_change() {
        let mut est = SemanticStateEstimator::new(0.9);
        est.observe("mug", &[1.0, 0.0], 0.5);
        // New observation with different dimension – embedding is replaced.
        est.observe("mug", &[0.5, 0.5, 0.5], 0.3);
        let state = est.query("mug").unwrap();
        assert_eq!(state.mean_embedding.len(), 3);
    }

    // ── decay_all ────────────────────────────────────────────────────────────

    #[test]
    fn decay_all_reduces_confidence() {
        let mut est = SemanticStateEstimator::new(0.8);
        est.observe("mug", &[1.0, 0.0], 1.0);
        est.decay_all();
        let state = est.query("mug").unwrap();
        assert!((state.confidence - 0.8).abs() < 1e-6);
    }

    #[test]
    fn decay_all_multiple_ticks() {
        let mut est = SemanticStateEstimator::new(0.5);
        est.observe("mug", &[1.0], 1.0);
        est.decay_all();
        est.decay_all();
        let state = est.query("mug").unwrap();
        // 1.0 * 0.5 * 0.5 = 0.25
        assert!((state.confidence - 0.25).abs() < 1e-6);
    }

    #[test]
    fn decay_does_not_remove_entities() {
        let mut est = SemanticStateEstimator::new(0.1);
        est.observe("mug", &[1.0], 0.5);
        for _ in 0..20 {
            est.decay_all();
        }
        // Entity still exists even with negligible confidence.
        assert!(est.query("mug").is_some());
    }

    // ── prune ─────────────────────────────────────────────────────────────────

    #[test]
    fn prune_removes_low_confidence_entities() {
        let mut est = SemanticStateEstimator::new(0.5);
        est.observe("mug", &[1.0], 0.9);
        est.observe("table", &[0.0], 0.05);
        let pruned = est.prune(0.1);
        assert_eq!(pruned, 1);
        assert!(est.query("mug").is_some());
        assert!(est.query("table").is_none());
    }

    #[test]
    fn prune_nothing_when_all_above_threshold() {
        let mut est = SemanticStateEstimator::new(0.9);
        est.observe("mug", &[1.0], 0.8);
        est.observe("table", &[0.5], 0.6);
        let pruned = est.prune(0.1);
        assert_eq!(pruned, 0);
    }

    // ── most_likely_state ────────────────────────────────────────────────────

    #[test]
    fn most_likely_state_returns_highest_confidence() {
        let mut est = SemanticStateEstimator::new(0.9);
        est.observe("mug", &[1.0, 0.0], 0.3);
        est.observe("table", &[0.0, 1.0], 0.9);
        let best = est.most_likely_state().unwrap();
        assert_eq!(best.label, "table");
    }

    #[test]
    fn most_likely_state_none_when_empty() {
        let est = SemanticStateEstimator::new(0.9);
        assert!(est.most_likely_state().is_none());
    }

    // ── all_labels_by_confidence ─────────────────────────────────────────────

    #[test]
    fn all_labels_ordered_by_descending_confidence() {
        let mut est = SemanticStateEstimator::new(0.9);
        est.observe("a", &[1.0], 0.2);
        est.observe("b", &[1.0], 0.8);
        est.observe("c", &[1.0], 0.5);
        let labels = est.all_labels_by_confidence();
        assert_eq!(labels, vec!["b", "c", "a"]);
    }

    // ── len / is_empty ────────────────────────────────────────────────────────

    #[test]
    fn len_and_is_empty() {
        let mut est = SemanticStateEstimator::new(0.9);
        assert!(est.is_empty());
        est.observe("mug", &[1.0], 0.5);
        assert_eq!(est.len(), 1);
        assert!(!est.is_empty());
    }

    // ── decay_factor clamping ────────────────────────────────────────────────

    #[test]
    fn decay_factor_clamped() {
        let est_hi = SemanticStateEstimator::new(2.0);
        assert!((est_hi.decay_factor - 0.9999).abs() < 1e-4);

        let est_lo = SemanticStateEstimator::new(0.0);
        assert!((est_lo.decay_factor - 0.001).abs() < 1e-4);
    }
}
