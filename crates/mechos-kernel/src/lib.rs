//! `mechos-kernel` – Safety & Orchestration
//!
//! The central brainstem of MechOS. It does not think; it enforces rules and
//! regulates the system.
//!
//! # Modules
//!
//! - [`capability_manager`] – [`CapabilityManager`][capability_manager::CapabilityManager]:
//!   enforces the principle of least privilege by verifying that the requesting
//!   agent holds the required [`Capability`][mechos_types::Capability] before
//!   any tool or hardware is invoked.
//! - [`state_verifier`] – [`StateVerifier`][state_verifier::StateVerifier]:
//!   a rule engine that validates every [`HardwareIntent`][mechos_types::HardwareIntent]
//!   against registered physical invariants (joint limits, speed caps, etc.)
//!   and returns a fault if any invariant is violated.
//! - [`watchdog`] – [`Watchdog`][watchdog::Watchdog]:
//!   tracks heartbeats from registered subsystems and detects frozen
//!   components so that a supervisor can trigger restarts.

pub mod capability_manager;
pub mod state_verifier;
pub mod watchdog;

pub use capability_manager::CapabilityManager;
pub use state_verifier::{JointLimitRule, Rule, SpeedCapRule, StateVerifier};
pub use watchdog::{ComponentHealth, Watchdog};

