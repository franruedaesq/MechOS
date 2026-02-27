//! [`CockpitServer`] – HTTP + WebSocket server for the Cockpit UI.
//!
//! Listens on `0.0.0.0:8080` (configurable via [`CockpitServer::with_port`]).
//!
//! * Regular HTTP requests → 200 OK with the embedded Cockpit HTML.
//! * WebSocket upgrades → bidirectional bridge to the [`EventBus`].

use std::net::SocketAddr;
use std::sync::Arc;

use futures_util::{SinkExt, StreamExt};
use mechos_middleware::EventBus;
use mechos_types::{Event, EventPayload, MechError};
use serde_json::Value;
use tokio::io::AsyncWriteExt;
use tokio::net::{TcpListener, TcpStream};
use tokio_tungstenite::{accept_async, tungstenite::Message};
use uuid::Uuid;
use chrono::Utc;

/// Default TCP port for the Cockpit HTTP/WebSocket server.
pub const DEFAULT_PORT: u16 = 8080;

/// The compiled-in Cockpit single-page application (HTML + CSS + JS).
const COCKPIT_HTML: &str = include_str!("cockpit.html");

// ---------------------------------------------------------------------------
// CockpitServer
// ---------------------------------------------------------------------------

/// Lightweight HTTP + WebSocket server that serves the Cockpit UI and bridges
/// the internal [`EventBus`] to every connected browser.
///
/// # Example
///
/// ```rust,no_run
/// use std::sync::Arc;
/// use mechos_middleware::EventBus;
/// use mechos_cockpit::CockpitServer;
///
/// #[tokio::main]
/// async fn main() {
///     let bus = Arc::new(EventBus::default());
///     CockpitServer::new(Arc::clone(&bus))
///         .run()
///         .await
///         .expect("cockpit server failed");
/// }
/// ```
pub struct CockpitServer {
    bus: Arc<EventBus>,
    port: u16,
}

impl CockpitServer {
    /// Create a server backed by `bus` on the [`DEFAULT_PORT`].
    pub fn new(bus: Arc<EventBus>) -> Self {
        Self {
            bus,
            port: DEFAULT_PORT,
        }
    }

    /// Override the listening port (builder-style).
    pub fn with_port(mut self, port: u16) -> Self {
        self.port = port;
        self
    }

    /// Return the configured port.
    pub fn port(&self) -> u16 {
        self.port
    }

    /// Start the server.
    ///
    /// Listens for TCP connections and dispatches each one as either a
    /// WebSocket bridge (when the HTTP request contains `Upgrade: websocket`)
    /// or a plain HTTP response serving the Cockpit HTML.
    ///
    /// # Errors
    ///
    /// Returns [`MechError::Serialization`] if the TCP listener cannot bind.
    pub async fn run(self) -> Result<(), MechError> {
        let addr = SocketAddr::from(([0, 0, 0, 0], self.port));
        let listener = TcpListener::bind(addr).await.map_err(|e| {
            MechError::Serialization(format!("[mechos-cockpit] bind error on {addr}: {e}"))
        })?;

        println!("[mechos-cockpit] Cockpit UI listening on http://localhost:{}", self.port);

        loop {
            match listener.accept().await {
                Ok((stream, peer)) => {
                    let bus = Arc::clone(&self.bus);
                    tokio::spawn(async move {
                        if let Err(e) = handle_connection(stream, peer, bus).await {
                            eprintln!("[mechos-cockpit] client {peer} error: {e}");
                        }
                    });
                }
                Err(e) => {
                    eprintln!("[mechos-cockpit] accept error: {e}");
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Per-connection handler
// ---------------------------------------------------------------------------

async fn handle_connection(
    stream: TcpStream,
    peer: SocketAddr,
    bus: Arc<EventBus>,
) -> Result<(), MechError> {
    // Peek at the first bytes of the request to decide whether to upgrade
    // to WebSocket or serve the static HTML.  `peek` does not consume the
    // data, so tungstenite's handshaker sees the full HTTP request.
    let mut buf = [0u8; 1024];
    let n = stream.peek(&mut buf).await.map_err(|e| {
        MechError::Serialization(format!("peek error from {peer}: {e}"))
    })?;

    let header_preview = String::from_utf8_lossy(&buf[..n]);
    let is_ws_upgrade = header_preview
        .lines()
        .any(|line| line.to_lowercase().starts_with("upgrade:") && line.to_lowercase().contains("websocket"));

    if is_ws_upgrade {
        handle_ws(stream, peer, bus).await
    } else {
        serve_html(stream).await
    }
}

// ---------------------------------------------------------------------------
// Plain HTTP: serve the embedded Cockpit HTML
// ---------------------------------------------------------------------------

async fn serve_html(mut stream: TcpStream) -> Result<(), MechError> {
    let body = COCKPIT_HTML;
    let response = format!(
        "HTTP/1.1 200 OK\r\n\
         Content-Type: text/html; charset=utf-8\r\n\
         Content-Length: {}\r\n\
         Connection: close\r\n\
         \r\n\
         {}",
        body.len(),
        body
    );
    stream
        .write_all(response.as_bytes())
        .await
        .map_err(|e| MechError::Serialization(format!("HTTP write error: {e}")))?;
    Ok(())
}

// ---------------------------------------------------------------------------
// WebSocket: bidirectional EventBus bridge
// ---------------------------------------------------------------------------

async fn handle_ws(
    stream: TcpStream,
    peer: SocketAddr,
    bus: Arc<EventBus>,
) -> Result<(), MechError> {
    let ws_stream = accept_async(stream).await.map_err(|e| {
        MechError::Serialization(format!("[mechos-cockpit] WS handshake from {peer}: {e}"))
    })?;

    let (mut ws_tx, mut ws_rx) = ws_stream.split();
    let mut bus_rx = bus.subscribe();

    loop {
        tokio::select! {
            // ── Downstream: EventBus → browser ─────────────────────────────
            result = bus_rx.recv() => {
                match result {
                    Ok(event) => {
                        match serde_json::to_string(&event) {
                            Ok(json) => {
                                if ws_tx.send(Message::Text(json.into())).await.is_err() {
                                    break;
                                }
                            }
                            Err(e) => {
                                eprintln!("[mechos-cockpit] serialization error: {e}");
                            }
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                        eprintln!("[mechos-cockpit] ws client {peer} lagged by {n} events");
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
                }
            }
            // ── Upstream: browser → EventBus ────────────────────────────────
            msg = ws_rx.next() => {
                match msg {
                    Some(Ok(Message::Text(text))) => {
                        handle_upstream_message(text.as_str(), &bus);
                    }
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Err(_)) => break,
                    _ => {}
                }
            }
        }
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Upstream message parser
// ---------------------------------------------------------------------------

/// Parse an incoming WebSocket text message from the Cockpit browser and
/// inject the appropriate event onto the [`EventBus`].
///
/// Recognised topics:
///
/// | Topic | Effect |
/// |---|---|
/// | `/cmd_vel` + `source: "dashboard_override"` | Arms AI suspension; publishes override event |
/// | `/hitl/human_response` | Publishes [`EventPayload::HumanResponse`] |
/// | `/agent/mode` | Publishes [`EventPayload::AgentModeToggle`] |
///
/// Unknown messages are silently ignored.
pub(crate) fn handle_upstream_message(text: &str, bus: &Arc<EventBus>) {
    let Ok(json) = serde_json::from_str::<Value>(text) else {
        return;
    };

    let topic = json.get("topic").and_then(|t| t.as_str()).unwrap_or("");
    let source = json.get("source").and_then(|s| s.as_str()).unwrap_or("");

    // ── Manual teleop override ──────────────────────────────────────────────
    if topic == "/cmd_vel" && source == "dashboard_override" {
        let event = Event {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: "mechos-middleware::dashboard_override".to_string(),
            payload: EventPayload::AgentThought(text.to_string()),
        };
        let _ = bus.publish(event);
        return;
    }

    // ── HITL human response ─────────────────────────────────────────────────
    if topic == "/hitl/human_response" {
        if let Some(response) = json
            .get("msg")
            .and_then(|m| m.get("response"))
            .and_then(|r| r.as_str())
        {
            let event = Event {
                id: Uuid::new_v4(),
                timestamp: Utc::now(),
                source: "mechos-middleware::dashboard/human_response".to_string(),
                payload: EventPayload::HumanResponse(response.to_string()),
            };
            let _ = bus.publish(event);
        }
        return;
    }

    // ── Cockpit mode toggle (pause / resume autonomous loop) ────────────────
    if topic == "/agent/mode" {
        if let Some(paused) = json
            .get("msg")
            .and_then(|m| m.get("paused"))
            .and_then(|p| p.as_bool())
        {
            let event = Event {
                id: Uuid::new_v4(),
                timestamp: Utc::now(),
                source: "mechos-cockpit::server".to_string(),
                payload: EventPayload::AgentModeToggle { paused },
            };
            let _ = bus.publish(event);
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use mechos_middleware::EventBus;
    use mechos_types::EventPayload;

    fn make_bus() -> Arc<EventBus> {
        Arc::new(EventBus::default())
    }

    // ── CockpitServer constructor ─────────────────────────────────────────────

    #[test]
    fn default_port_is_8080() {
        let bus = make_bus();
        let server = CockpitServer::new(bus);
        assert_eq!(server.port(), DEFAULT_PORT);
    }

    #[test]
    fn with_port_overrides_default() {
        let bus = make_bus();
        let server = CockpitServer::new(bus).with_port(9999);
        assert_eq!(server.port(), 9999);
    }

    // ── Upstream message handling ─────────────────────────────────────────────

    #[tokio::test]
    async fn upstream_override_publishes_agent_thought() {
        let bus = make_bus();
        let mut rx = bus.subscribe();

        let msg = r#"{"op":"publish","topic":"/cmd_vel","msg":{"linear":{"x":0.5,"y":0,"z":0},"angular":{"x":0,"y":0,"z":-0.2}},"source":"dashboard_override"}"#;
        handle_upstream_message(msg, &bus);

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-middleware::dashboard_override");
        assert!(matches!(event.payload, EventPayload::AgentThought(_)));
    }

    #[tokio::test]
    async fn upstream_hitl_response_publishes_human_response() {
        let bus = make_bus();
        let mut rx = bus.subscribe();

        let msg = r#"{"op":"publish","topic":"/hitl/human_response","msg":{"response":"Yes, push the box"}}"#;
        handle_upstream_message(msg, &bus);

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-middleware::dashboard/human_response");
        if let EventPayload::HumanResponse(resp) = event.payload {
            assert_eq!(resp, "Yes, push the box");
        } else {
            panic!("expected HumanResponse");
        }
    }

    #[tokio::test]
    async fn upstream_mode_toggle_pause_publishes_agent_mode_toggle() {
        let bus = make_bus();
        let mut rx = bus.subscribe();

        let msg = r#"{"topic":"/agent/mode","msg":{"paused":true}}"#;
        handle_upstream_message(msg, &bus);

        let event = rx.recv().await.unwrap();
        assert_eq!(event.source, "mechos-cockpit::server");
        assert!(
            matches!(event.payload, EventPayload::AgentModeToggle { paused: true }),
            "expected AgentModeToggle(paused=true)"
        );
    }

    #[tokio::test]
    async fn upstream_mode_toggle_resume_publishes_agent_mode_toggle() {
        let bus = make_bus();
        let mut rx = bus.subscribe();

        let msg = r#"{"topic":"/agent/mode","msg":{"paused":false}}"#;
        handle_upstream_message(msg, &bus);

        let event = rx.recv().await.unwrap();
        assert!(
            matches!(event.payload, EventPayload::AgentModeToggle { paused: false }),
            "expected AgentModeToggle(paused=false)"
        );
    }

    #[tokio::test]
    async fn upstream_unknown_message_is_ignored() {
        let bus = make_bus();
        let mut rx = bus.subscribe();

        // Publish a known event first so we can verify nothing else was added.
        let known_event = Event {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: "test".to_string(),
            payload: EventPayload::AgentThought("sentinel".to_string()),
        };
        let _ = bus.publish(known_event);

        // Send an unknown message.
        handle_upstream_message(r#"{"op":"subscribe","topic":"/unknown"}"#, &bus);

        // Only the sentinel event should be in the channel.
        let event = rx.recv().await.unwrap();
        if let EventPayload::AgentThought(s) = event.payload {
            assert_eq!(s, "sentinel");
        } else {
            panic!("unexpected payload");
        }
        // Channel should now be empty.
        assert!(rx.try_recv().is_err());
    }

    #[tokio::test]
    async fn upstream_invalid_json_is_ignored() {
        let bus = make_bus();
        let mut rx = bus.subscribe();

        let known_event = Event {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: "test".to_string(),
            payload: EventPayload::AgentThought("sentinel".to_string()),
        };
        let _ = bus.publish(known_event);

        handle_upstream_message("not json at all", &bus);

        let event = rx.recv().await.unwrap();
        assert!(matches!(event.payload, EventPayload::AgentThought(_)));
        assert!(rx.try_recv().is_err());
    }

    // ── HTML embedding ────────────────────────────────────────────────────────

    #[test]
    fn cockpit_html_is_non_empty() {
        assert!(!COCKPIT_HTML.is_empty(), "embedded Cockpit HTML must not be empty");
    }

    #[test]
    fn cockpit_html_contains_websocket_connect_code() {
        assert!(
            COCKPIT_HTML.contains("WebSocket"),
            "Cockpit HTML must contain WebSocket connection code"
        );
    }

    #[test]
    fn cockpit_html_contains_wasd_binding() {
        assert!(
            COCKPIT_HTML.contains("KeyW") || COCKPIT_HTML.contains("wasd") || COCKPIT_HTML.contains("WASD"),
            "Cockpit HTML must contain W-A-S-D keyboard bindings"
        );
    }
}
