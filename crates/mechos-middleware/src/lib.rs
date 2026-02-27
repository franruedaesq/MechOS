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

pub mod bus;
pub mod ros2_bridge;

pub use bus::{EventBus, TopicSubscriber};
pub use ros2_bridge::Ros2Bridge;
