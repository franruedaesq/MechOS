//! The Universal Adapter Pattern.
//!
//! MechOS never speaks directly to ROS 2 or WebSockets.  It publishes to its
//! internal [`EventBus`][crate::bus::EventBus].  Adapters listen to this bus
//! and translate intents into the specific protocol of the outside world.
//!
//! # Overview
//!
//! - [`MechAdapter`] – the trait every adapter must implement.
//! - [`Ros2Adapter`][crate::ros2_adapter::Ros2Adapter] – drives a physical
//!   robot via ROS 2 MoveIt 2 / `/cmd_vel`.
//! - [`DashboardSimAdapter`][crate::dashboard_sim_adapter::DashboardSimAdapter]
//!   – drives the React / Three.js simulation over a WebSocket.

use async_trait::async_trait;
use futures_util::stream::BoxStream;
use mechos_types::{EventPayload, HardwareIntent, MechError};

/// Every external-protocol adapter must implement this trait.
///
/// # Contract
///
/// * `execute_intent` – receives a high-level [`HardwareIntent`] from the
///   EventBus and translates it into external commands (e.g. ROS 2 `/cmd_vel`,
///   a WebSocket JSON frame, …).
///
/// * `sensor_stream` – returns a live stream of [`EventPayload`] values that
///   the adapter produces by translating inbound sensor data (e.g. LiDAR scans)
///   into MechOS events.
#[async_trait]
pub trait MechAdapter: Send + Sync {
    /// Translate a high-level [`HardwareIntent`] into external commands.
    async fn execute_intent(&self, intent: HardwareIntent) -> Result<(), MechError>;

    /// Translate external sensor data into a stream of [`EventPayload`] values.
    async fn sensor_stream(&self) -> BoxStream<'static, EventPayload>;
}
