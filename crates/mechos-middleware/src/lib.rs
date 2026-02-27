//! `mechos-middleware` – The Nervous System
//!
//! Routes asynchronous data between hardware, the OS kernel, and external
//! clients without caring about the data's meaning.
//!
//! # Modules
//!
//! - [`bus`] – Headless, typed, topic-based publish/subscribe event bus built
//!   on Tokio broadcast channels.
//! - [`ros2_bridge`] – Universal ROS2-to-WebSocket bridge that translates DDS
//!   robotics traffic into lightweight JSON for web clients.
//! - [`adapter`] – The [`MechAdapter`] trait: the Universal Adapter Pattern
//!   that decouples MechOS from any specific external protocol.
//! - [`ros2_adapter`] – [`Ros2Adapter`]: drives a physical robot via ROS 2
//!   MoveIt 2 and reads LiDAR data from `/scan`.
//! - [`dashboard_sim_adapter`] – [`DashboardSimAdapter`]: drives the React /
//!   Three.js simulation over a `rosbridge_server`-compatible WebSocket and
//!   ingests virtual LiDAR data from `/sim_scan`.

pub mod adapter;
pub mod bus;
pub mod dashboard_sim_adapter;
pub mod ros2_adapter;
pub mod ros2_bridge;

pub use adapter::MechAdapter;
pub use bus::{EventBus, Topic, TopicReceiver, TopicSubscriber};
pub use dashboard_sim_adapter::DashboardSimAdapter;
pub use ros2_adapter::Ros2Adapter;
pub use ros2_bridge::Ros2Bridge;
