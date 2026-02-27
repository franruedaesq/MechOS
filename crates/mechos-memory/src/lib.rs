//! `mechos-memory` – The Knowledge Base.
//!
//! Provides the robot with persistent state and recall capabilities, utilizing
//! a local SQLite substrate.
//!
//! # Modules
//!
//! - [`episodic`] – [`EpisodicStore`][episodic::EpisodicStore]: a local vector
//!   database that persists interaction summaries and their embedding vectors to
//!   SQLite and supports cosine-similarity recall.
//! - [`semantic`] – [`SemanticStateEstimator`][semantic::SemanticStateEstimator]:
//!   fuses past visual/conceptual embeddings with a time-decay probability model
//!   to track the semantic state of the world over time (e.g. remembering where
//!   an object was last placed).

pub mod episodic;
pub mod semantic;
