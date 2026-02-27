//! `mechos-cockpit` – The Observability & Teleop Web UI Server
//!
//! Boots a lightweight HTTP + WebSocket server (default port `8080`) that:
//!
//! 1. **Serves** the static Cockpit Single-Page Application (HTML/CSS/JS)
//!    at every non-WebSocket HTTP path.
//!
//! 2. **Bridges** the internal [`EventBus`] to every connected browser tab
//!    over a persistent WebSocket connection so that [`TelemetryData`],
//!    [`AgentThought`], [`LidarScan`], and [`AskHuman`] events stream to the
//!    UI in real-time.
//!
//! 3. **Accepts** upstream messages from the browser:
//!    - `"/cmd_vel"` with `source: "dashboard_override"` → arms the
//!      10-second AI suspension and forwards a `Drive` command.
//!    - `"/hitl/human_response"` → publishes an
//!      [`EventPayload::HumanResponse`] so the [`AgentLoop`] can resume.
//!    - `"/agent/mode"` → publishes an [`EventPayload::AgentModeToggle`] to
//!      pause or resume the autonomous loop independently of the joystick.
//!
//! # Usage
//!
//! ```rust,no_run
//! use std::sync::Arc;
//! use mechos_middleware::EventBus;
//! use mechos_cockpit::CockpitServer;
//!
//! #[tokio::main]
//! async fn main() {
//!     let bus = Arc::new(EventBus::default());
//!     CockpitServer::new(Arc::clone(&bus))
//!         .run()
//!         .await
//!         .expect("cockpit server failed");
//! }
//! ```
//!
//! [`TelemetryData`]: mechos_types::TelemetryData
//! [`AgentThought`]: mechos_types::EventPayload::AgentThought
//! [`LidarScan`]: mechos_types::EventPayload::LidarScan
//! [`AskHuman`]: mechos_types::HardwareIntent::AskHuman
//! [`EventPayload::HumanResponse`]: mechos_types::EventPayload::HumanResponse
//! [`EventPayload::AgentModeToggle`]: mechos_types::EventPayload::AgentModeToggle
//! [`AgentLoop`]: mechos_runtime::AgentLoop

pub mod server;

pub use server::{CockpitServer, DEFAULT_PORT};
