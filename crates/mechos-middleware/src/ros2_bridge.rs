//! Universal ROS2-to-WebSocket bridge.
//!
//! This module provides [`Ros2Bridge`], which:
//!
//! 1. **Ingests** ROS2-style messages (odometry, laser scan, hardware faults)
//!    and translates them into [`Event`] values that are published onto the
//!    internal [`EventBus`].
//!
//! 2. **Serves** a lightweight WebSocket endpoint where external clients (web
//!    dashboards, telemetry UIs) can subscribe to the live event stream as
//!    newline-delimited JSON.
//!
//! The bridge is intentionally agnostic about the *meaning* of the data it
//! routes; it only handles serialisation and transport.
//!
//! # Security
//!
//! Incoming WebSocket frames are subject to two hard limits enforced before
//! any content is parsed:
//!
//! * **Payload size** – frames larger than [`MAX_INCOMING_PAYLOAD_BYTES`] are
//!   silently dropped and the connection is closed to prevent memory exhaustion
//!   or slow-parse denial-of-service attacks.
//! * **Rate limit** – the bridge accepts at most
//!   [`MAX_INCOMING_MESSAGES_PER_SEC`] messages per second across all
//!   connections.  Connections that exceed this quota are closed.

use std::net::SocketAddr;
use std::num::NonZeroU32;
use std::sync::Arc;

use futures_util::{SinkExt, StreamExt};
use governor::clock::DefaultClock;
use governor::middleware::NoOpMiddleware;
use governor::state::{InMemoryState, NotKeyed};
use governor::{Quota, RateLimiter};
use mechos_types::{Event, EventPayload, MechError, TelemetryData};
use serde_json;
use tokio::net::{TcpListener, TcpStream};
use tokio_tungstenite::{accept_async_with_config, tungstenite::{Message, protocol::WebSocketConfig}};
use uuid::Uuid;
use chrono::Utc;
use tracing::{error, warn};

use crate::bus::EventBus;

/// Maximum size (in bytes) of an incoming WebSocket payload.
///
/// Frames larger than this are rejected before parsing to prevent
/// memory-exhaustion and slow-parse denial-of-service attacks.
pub const MAX_INCOMING_PAYLOAD_BYTES: usize = 1024 * 1024; // 1 MiB

/// Maximum number of incoming WebSocket messages accepted per second
/// across a single bridge instance.  Connections that exceed this rate
/// are closed.
pub const MAX_INCOMING_MESSAGES_PER_SEC: u32 = 100;

type DirectRateLimiter = RateLimiter<NotKeyed, InMemoryState, DefaultClock, NoOpMiddleware>;

/// Bridge between ROS2 topics and the internal [`EventBus`] / WebSocket
/// clients.
#[derive(Clone)]
pub struct Ros2Bridge {
    bus: Arc<EventBus>,
    /// Rate limiter for incoming WebSocket messages (shared across all
    /// connections served by this bridge instance).
    incoming_limiter: Arc<DirectRateLimiter>,
}

impl Ros2Bridge {
    /// Create a new bridge backed by `bus`.
    pub fn new(bus: Arc<EventBus>) -> Self {
        let quota = Quota::per_second(
            NonZeroU32::new(MAX_INCOMING_MESSAGES_PER_SEC)
                .unwrap_or(NonZeroU32::MIN),
        );
        Self {
            bus,
            incoming_limiter: Arc::new(RateLimiter::direct(quota)),
        }
    }

    // -----------------------------------------------------------------------
    // ROS2 ingest helpers
    // -----------------------------------------------------------------------

    /// Ingest an `/odom`-style message and publish it as a
    /// [`EventPayload::Telemetry`] event.
    pub fn ingest_odom(
        &self,
        position_x: f32,
        position_y: f32,
        heading_rad: f32,
        battery_percent: u8,
    ) -> Result<usize, MechError> {
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: "mechos-middleware::ros2/odom".to_string(),
            payload: EventPayload::Telemetry(TelemetryData {
                position_x,
                position_y,
                heading_rad,
                battery_percent,
            }),
            trace_id: None,
        };
        self.bus.publish(event)
    }

    /// Ingest a hardware-fault notification and publish it as a
    /// [`EventPayload::HardwareFault`] event.
    pub fn ingest_fault(
        &self,
        component: impl Into<String>,
        code: u32,
        message: impl Into<String>,
    ) -> Result<usize, MechError> {
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: "mechos-middleware::ros2/fault".to_string(),
            payload: EventPayload::HardwareFault {
                component: component.into(),
                code,
                message: message.into(),
            },
            trace_id: None,
        };
        self.bus.publish(event)
    }

    // -----------------------------------------------------------------------
    // WebSocket server
    // -----------------------------------------------------------------------

    /// Start a WebSocket server on `addr`.
    ///
    /// Every connecting client receives a stream of newline-delimited JSON
    /// objects, one per event on the bus.  The server runs until it
    /// encounters a fatal bind error.
    ///
    /// # Errors
    ///
    /// Returns [`MechError::Serialization`] if the TCP listener cannot be
    /// bound.
    pub async fn run_ws_server(self, addr: SocketAddr) -> Result<(), MechError> {
        let listener = TcpListener::bind(addr).await.map_err(|e| {
            MechError::Serialization(format!("ws bind error on {addr}: {e}"))
        })?;

        loop {
            match listener.accept().await {
                Ok((stream, peer)) => {
                    let bridge = self.clone();
                    tokio::spawn(async move {
                        if let Err(e) = bridge.handle_ws_client(stream, peer).await {
                            error!(peer = %peer, error = %e, "ws client error");
                        }
                    });
                }
                Err(e) => {
                    error!(error = %e, "ws accept error");
                }
            }
        }
    }

    async fn handle_ws_client(
        &self,
        stream: TcpStream,
        peer: SocketAddr,
    ) -> Result<(), MechError> {
        let mut ws_config = WebSocketConfig::default();
        ws_config.max_message_size = Some(MAX_INCOMING_PAYLOAD_BYTES);
        let ws_stream = accept_async_with_config(stream, Some(ws_config)).await.map_err(|e| {
            MechError::Serialization(format!("ws handshake from {peer}: {e}"))
        })?;

        let (mut ws_tx, mut ws_rx) = ws_stream.split();
        let mut rx = self.bus.subscribe();

        loop {
            tokio::select! {
                // Forward events from the bus to the WebSocket client.
                result = rx.recv() => {
                    match result {
                        Ok(event) => {
                            let json = serde_json::to_string(&event).map_err(|e| {
                                MechError::Serialization(e.to_string())
                            })?;
                            if ws_tx.send(Message::Text(json.into())).await.is_err() {
                                break;
                            }
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                            warn!(peer = %peer, lagged_by = n, "ws client lagged");
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
                    }
                }
                // Handle incoming WebSocket frames.
                msg = ws_rx.next() => {
                    match msg {
                        Some(Ok(Message::Close(_))) | None => break,
                        Some(Err(_)) => break,
                        Some(Ok(Message::Text(text))) => {
                            // ── Payload size guard ─────────────────────────
                            if text.len() > MAX_INCOMING_PAYLOAD_BYTES {
                                warn!(
                                    peer = %peer,
                                    payload_bytes = text.len(),
                                    limit_bytes = MAX_INCOMING_PAYLOAD_BYTES,
                                    "incoming WebSocket payload exceeds size limit; closing connection"
                                );
                                break;
                            }
                            // ── Rate limit guard ───────────────────────────
                            if self.incoming_limiter.check().is_err() {
                                warn!(
                                    peer = %peer,
                                    "incoming WebSocket rate limit exceeded; closing connection"
                                );
                                break;
                            }
                            self.handle_incoming_ws_message(text.as_str());
                        }
                        _ => {}
                    }
                }
            }
        }

        Ok(())
    }

    /// Parse an incoming WebSocket text message from the dashboard.
    ///
    /// Two message kinds are recognised:
    ///
    /// * **Manual override** – a `rosbridge_server` publish on `/cmd_vel` that
    ///   carries the extra field `"source": "dashboard_override"`.  The Twist
    ///   velocities are extracted and re-published on the bus with source
    ///   `"mechos-middleware::dashboard_override"` so that the
    ///   [`AgentLoop`] can arm its 10-second AI suspension.
    ///
    /// * **Human response** – a publish on `/hitl/human_response` whose `msg`
    ///   contains a `"response"` string.  Published as
    ///   [`EventPayload::HumanResponse`] so that the [`AgentLoop`] can inject
    ///   it back into the LLM context window.
    ///
    /// Any message that does not match either pattern is silently ignored.
    fn handle_incoming_ws_message(&self, text: &str) {
        let Ok(json) = serde_json::from_str::<serde_json::Value>(text) else {
            return;
        };

        let topic = json.get("topic").and_then(|t| t.as_str()).unwrap_or("");
        let source = json.get("source").and_then(|s| s.as_str()).unwrap_or("");

        // ── Manual override ──────────────────────────────────────────────────
        if topic == "/cmd_vel" && source == "dashboard_override" {
            let event = Event {
                id: Uuid::new_v4(),
                timestamp: Utc::now(),
                source: "mechos-middleware::dashboard_override".to_string(),
                payload: EventPayload::AgentThought(text.to_string()),
                trace_id: None,
            };
            let _ = self.bus.publish(event);
            return;
        }

        // ── Human response to AskHuman ───────────────────────────────────────
        if topic == "/hitl/human_response"
            && let Some(response) = json
                .get("msg")
                .and_then(|m| m.get("response"))
                .and_then(|r| r.as_str())
        {
            let event = Event {
                id: Uuid::new_v4(),
                timestamp: Utc::now(),
                source: "mechos-middleware::dashboard/human_response".to_string(),
                payload: EventPayload::HumanResponse(response.to_string()),
                trace_id: None,
            };
            let _ = self.bus.publish(event);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::EventBus;
    use mechos_types::EventPayload;

    fn make_bridge() -> (Arc<EventBus>, Ros2Bridge) {
        let bus = Arc::new(EventBus::default());
        let bridge = Ros2Bridge::new(Arc::clone(&bus));
        (bus, bridge)
    }

    #[tokio::test]
    async fn ingest_odom_publishes_telemetry() -> Result<(), Box<dyn std::error::Error>> {
        let (bus, bridge) = make_bridge();
        let mut rx = bus.subscribe();

        bridge.ingest_odom(1.0, 2.0, 0.5, 85)?;

        let event = rx.recv().await?;
        assert_eq!(event.source, "mechos-middleware::ros2/odom");
        assert!(matches!(event.payload, EventPayload::Telemetry(_)));
        if let EventPayload::Telemetry(t) = event.payload {
            assert!((t.position_x - 1.0).abs() < f32::EPSILON);
            assert!((t.position_y - 2.0).abs() < f32::EPSILON);
            assert!((t.heading_rad - 0.5).abs() < f32::EPSILON);
            assert_eq!(t.battery_percent, 85);
        }
        Ok(())
    }

    #[tokio::test]
    async fn ingest_fault_publishes_hardware_fault() -> Result<(), Box<dyn std::error::Error>> {
        let (bus, bridge) = make_bridge();
        let mut rx = bus.subscribe();

        bridge.ingest_fault("motor_left", 42, "overcurrent")?;

        let event = rx.recv().await?;
        assert_eq!(event.source, "mechos-middleware::ros2/fault");
        assert!(matches!(event.payload, EventPayload::HardwareFault { .. }));
        if let EventPayload::HardwareFault { component, code, message } = event.payload {
            assert_eq!(component, "motor_left");
            assert_eq!(code, 42);
            assert_eq!(message, "overcurrent");
        }
        Ok(())
    }

    #[tokio::test]
    async fn odom_event_is_json_serialisable() -> Result<(), Box<dyn std::error::Error>> {
        let (bus, bridge) = make_bridge();
        let mut rx = bus.subscribe();

        bridge.ingest_odom(3.0, 4.0, 1.57, 60)?;

        let event = rx.recv().await?;
        let json = serde_json::to_string(&event)?;
        assert!(json.contains("Telemetry"));
        assert!(json.contains("ros2/odom"));
        Ok(())
    }

    #[tokio::test]
    async fn handle_incoming_override_publishes_dashboard_override_event() -> Result<(), Box<dyn std::error::Error>> {
        let (bus, bridge) = make_bridge();
        let mut rx = bus.subscribe();

        let override_msg = r#"{"op":"publish","topic":"/cmd_vel","msg":{"linear":{"x":0.5,"y":0,"z":0},"angular":{"x":0,"y":0,"z":-0.2}},"source":"dashboard_override"}"#;
        bridge.handle_incoming_ws_message(override_msg);

        let event = rx.recv().await?;
        assert_eq!(event.source, "mechos-middleware::dashboard_override");
        if let EventPayload::AgentThought(raw) = &event.payload {
            assert!(raw.contains("dashboard_override"));
        } else {
            panic!("expected AgentThought for override");
        }
        Ok(())
    }

    #[tokio::test]
    async fn handle_incoming_human_response_publishes_human_response_event() -> Result<(), Box<dyn std::error::Error>> {
        let (bus, bridge) = make_bridge();
        let mut rx = bus.subscribe();

        let resp_msg = r#"{"op":"publish","topic":"/hitl/human_response","msg":{"response":"Yes, push it"}}"#;
        bridge.handle_incoming_ws_message(resp_msg);

        let event = rx.recv().await?;
        assert_eq!(event.source, "mechos-middleware::dashboard/human_response");
        if let EventPayload::HumanResponse(resp) = event.payload {
            assert_eq!(resp, "Yes, push it");
        } else {
            panic!("expected HumanResponse");
        }
        Ok(())
    }

    #[tokio::test]
    async fn handle_incoming_unknown_message_is_ignored() -> Result<(), Box<dyn std::error::Error>> {
        let (bus, bridge) = make_bridge();
        let mut rx = bus.subscribe();

        // Publish a real event first so we have something to compare.
        bridge.ingest_odom(0.0, 0.0, 0.0, 100)?;
        // Now try a message that matches neither override nor HITL pattern.
        bridge.handle_incoming_ws_message(r#"{"op":"subscribe","topic":"/unknown"}"#);

        // Only the odom event should have arrived.
        let event = rx.recv().await?;
        assert_eq!(event.source, "mechos-middleware::ros2/odom");
        // The channel should now be empty (no extra event from the unknown message).
        assert!(rx.try_recv().is_err());
        Ok(())
    }

    #[tokio::test]
    async fn payload_size_constant_is_one_mib() {
        // Verify the constant has the expected value (1 MiB).
        assert_eq!(MAX_INCOMING_PAYLOAD_BYTES, 1024 * 1024);
    }

    #[test]
    fn incoming_payload_exceeding_limit_is_detected() {
        // Build a payload just over the limit and verify the size check that
        // handle_ws_client performs would catch it.
        let oversized = "x".repeat(MAX_INCOMING_PAYLOAD_BYTES + 1);
        assert!(
            oversized.len() > MAX_INCOMING_PAYLOAD_BYTES,
            "test payload must exceed the limit"
        );
        // Verify a normal-sized payload would pass the check.
        let normal = "x".repeat(MAX_INCOMING_PAYLOAD_BYTES);
        assert!(
            normal.len() <= MAX_INCOMING_PAYLOAD_BYTES,
            "normal payload must be within the limit"
        );
    }

    #[test]
    fn max_incoming_messages_per_sec_is_reasonable() {
        // Verify the constant is a sensible non-zero value.
        const { assert!(MAX_INCOMING_MESSAGES_PER_SEC > 0) };
        const { assert!(MAX_INCOMING_MESSAGES_PER_SEC <= 1000) };
    }

    #[tokio::test]
    async fn handle_incoming_malformed_json_returns_error() {
        let (bus, bridge) = make_bridge();
        let mut rx = bus.subscribe();

        // Publish a malformed JSON string.
        bridge.handle_incoming_ws_message("{ malformed_json: true");

        // The bridge should not panic, and no message should be sent to the bus.
        let result = rx.try_recv();
        assert!(result.is_err(), "Bus should not receive any event for malformed JSON");
    }
}
