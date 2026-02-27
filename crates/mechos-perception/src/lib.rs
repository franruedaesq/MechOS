//! `mechos-perception` – Embodied Cognition layer.
//!
//! Turns noisy sensor data into the mathematical representation of the
//! physical world that an LLM needs to reason about space and motion.
//!
//! # Modules
//!
//! - [`transform`] – [`TfEngine`][transform::TfEngine]: directed graph that
//!   computes spatial transforms (translations, rotations) between named
//!   reference frames.
//! - [`fusion`] – [`SensorFusion`][fusion::SensorFusion]: complementary filter
//!   that combines heterogeneous data streams (Odometry + IMU) into a unified
//!   [`FusedState`][fusion::FusedState].
//! - [`octree`] – [`Octree`][octree::Octree]: uses an Octree to partition 3-D
//!   space, providing fast collision detection so the LLM knows if a path is
//!   clear.

pub mod fusion;
pub mod octree;
pub mod transform;
