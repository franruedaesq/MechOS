//! [`LlmDriver`] – OpenAI-compatible LLM interface.
//!
//! Communicates with a locally-running model server that exposes an
//! OpenAI-compatible `/v1/chat/completions` endpoint, such as
//! [Ollama](https://ollama.com) (`http://localhost:11434`).
//!
//! # Cost control
//!
//! [`LlmDriver`] includes built-in safeguards against runaway API spend:
//!
//! * **Token counter** – every call to [`LlmDriver::complete`] estimates the
//!   tokens consumed (prompt + reply) via a simple word-count heuristic and
//!   accumulates the total.  The running total is exposed via
//!   [`LlmDriver::total_tokens`].
//! * **Rate limiter** – a [`governor`]-backed token-bucket rate limiter
//!   enforces at most [`LlmDriver::DEFAULT_RPM`] requests per minute.  When
//!   the bucket is empty, [`LlmDriver::complete`] returns
//!   [`LlmError::RateLimitExceeded`] immediately rather than blocking.
//! * **Budget circuit breaker** – once the cumulative token count exceeds
//!   [`LlmDriver::DEFAULT_TOKEN_BUDGET`] (or the custom value supplied to
//!   [`LlmDriver::with_budget`]) the driver trips and every subsequent call
//!   returns [`LlmError::BudgetExceeded`] until the owner resets the counter
//!   with [`LlmDriver::reset_token_counter`].
//!
//! # Example
//!
//! ```rust,no_run
//! use mechos_runtime::llm_driver::{LlmDriver, ChatMessage, Role};
//!
//! let driver = LlmDriver::new("http://localhost:11434", "llama3");
//!
//! let messages = vec![
//!     ChatMessage { role: Role::System, content: "You are a robot brain.".into() },
//!     ChatMessage { role: Role::User, content: "What should I do next?".into() },
//! ];
//!
//! // Requires a running Ollama instance – skipped in unit tests.
//! // let reply = driver.complete(&messages).unwrap();
//! ```

use std::num::NonZeroU32;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use governor::clock::DefaultClock;
use governor::middleware::NoOpMiddleware;
use governor::state::{InMemoryState, NotKeyed};
use governor::{Quota, RateLimiter};
use mechos_types::HardwareIntent;
use schemars::schema_for;
use serde::{Deserialize, Serialize};
use thiserror::Error;
use tracing::{debug, instrument, warn};

// ─────────────────────────────────────────────────────────────────────────────
// Stability guidelines
// ─────────────────────────────────────────────────────────────────────────────

/// Anti-loop rules that are automatically appended to every system-role message
/// sent to the LLM.  They instruct the model to vary its strategy when previous
/// actions have not succeeded, preventing Ollama from getting stuck in a
/// repetitive action loop.
pub const STABILITY_GUIDELINES: &str = "\
## Stability Guidelines (anti-loop rules)
- Do not repeat the same action more than 3 times in a row.
- If an action fails, try a different approach rather than retrying immediately.
- Vary your strategy when the previous actions have not produced progress.
- Avoid issuing the same HardwareIntent consecutively more than 3 times.
- When stuck, emit an AskHuman intent to request human guidance before continuing.";

// ─────────────────────────────────────────────────────────────────────────────
// Error type
// ─────────────────────────────────────────────────────────────────────────────

/// Errors that can arise from LLM driver operations.
#[derive(Error, Debug)]
pub enum LlmError {
    /// The HTTP request to the model server failed.
    #[error("HTTP error: {0}")]
    Http(#[from] reqwest::Error),
    /// The response from the model server could not be parsed.
    #[error("Unexpected response format: {0}")]
    BadResponse(String),
    /// The per-minute request rate limit has been reached.
    ///
    /// The caller should back off and retry after a short delay.
    #[error("LLM rate limit exceeded: too many requests per minute")]
    RateLimitExceeded,
    /// The cumulative token budget has been exhausted.
    ///
    /// Call [`LlmDriver::reset_token_counter`] or increase the budget via
    /// [`LlmDriver::with_budget`] before issuing further requests.
    #[error("LLM token budget exceeded: {used} tokens used, budget is {budget}")]
    BudgetExceeded {
        /// Tokens consumed so far in this session.
        used: u64,
        /// Configured token budget.
        budget: u64,
    },
    /// The configured endpoint uses an insecure `http://` scheme for a
    /// non-localhost host.  External model endpoints must use `https://`.
    #[error("Insecure endpoint: '{0}' uses http:// for a non-localhost host; use https://")]
    InsecureEndpoint(String),
}

// ─────────────────────────────────────────────────────────────────────────────
// Message types (OpenAI-compatible)
// ─────────────────────────────────────────────────────────────────────────────

/// The role of a participant in a chat conversation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Role {
    System,
    User,
    Assistant,
}

/// A single message in a chat conversation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChatMessage {
    pub role: Role,
    pub content: String,
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal request / response shapes
// ─────────────────────────────────────────────────────────────────────────────

/// `response_format` field that enforces structured JSON Schema output.
#[derive(Serialize)]
struct ResponseFormat {
    #[serde(rename = "type")]
    kind: &'static str,
    json_schema: serde_json::Value,
}

#[derive(Serialize)]
struct ChatRequest<'a> {
    model: &'a str,
    messages: &'a [ChatMessage],
    stream: bool,
    response_format: ResponseFormat,
}

#[derive(Deserialize)]
struct ChatResponse {
    choices: Vec<Choice>,
}

#[derive(Deserialize)]
struct Choice {
    message: ChatMessage,
}

// ─────────────────────────────────────────────────────────────────────────────
// Rate-limiter type alias
// ─────────────────────────────────────────────────────────────────────────────

type DirectRateLimiter = RateLimiter<NotKeyed, InMemoryState, DefaultClock, NoOpMiddleware>;

// ─────────────────────────────────────────────────────────────────────────────
// LlmDriver
// ─────────────────────────────────────────────────────────────────────────────

/// An async client for an OpenAI-compatible chat-completions endpoint.
///
/// Construct once and reuse across OODA loop iterations.
///
/// # Cost control
///
/// See the [module-level documentation](self) for details on the built-in
/// token counter, rate limiter, and budget circuit breaker.
pub struct LlmDriver {
    base_url: String,
    model: String,
    client: reqwest::Client,
    /// Cumulative token counter (prompt + completion tokens, estimated).
    total_tokens: Arc<AtomicU64>,
    /// Maximum tokens allowed before the circuit breaker trips.
    token_budget: u64,
    /// Token-bucket rate limiter (requests per minute).
    rate_limiter: Arc<DirectRateLimiter>,
}

impl LlmDriver {
    /// Default maximum requests per minute.
    pub const DEFAULT_RPM: u32 = 20;

    /// Default token budget before the circuit breaker trips (≈ 100 k tokens).
    pub const DEFAULT_TOKEN_BUDGET: u64 = 100_000;

    /// Create a new driver pointing at `base_url` (e.g. `"http://localhost:11434"`)
    /// and using `model` (e.g. `"llama3"`).
    ///
    /// The driver is initialised with [`DEFAULT_RPM`][Self::DEFAULT_RPM] and
    /// [`DEFAULT_TOKEN_BUDGET`][Self::DEFAULT_TOKEN_BUDGET].  Use
    /// [`with_budget`][Self::with_budget] or
    /// [`with_rpm`][Self::with_rpm] to customise the limits.
    pub fn new(base_url: impl Into<String>, model: impl Into<String>) -> Self {
        Self::with_limits(
            base_url,
            model,
            Self::DEFAULT_RPM,
            Self::DEFAULT_TOKEN_BUDGET,
        )
    }

    /// Create a driver with a custom token budget (all other defaults apply).
    pub fn with_budget(
        base_url: impl Into<String>,
        model: impl Into<String>,
        token_budget: u64,
    ) -> Self {
        Self::with_limits(base_url, model, Self::DEFAULT_RPM, token_budget)
    }

    /// Create a driver with a custom requests-per-minute rate limit (all other
    /// defaults apply).
    pub fn with_rpm(
        base_url: impl Into<String>,
        model: impl Into<String>,
        rpm: u32,
    ) -> Self {
        Self::with_limits(base_url, model, rpm, Self::DEFAULT_TOKEN_BUDGET)
    }

    /// Create a driver with fully custom rate limits.
    ///
    /// # Arguments
    ///
    /// * `rpm` – maximum requests per minute.  A value of `0` is silently
    ///   clamped to `1` because the underlying [`governor`] rate limiter
    ///   requires a non-zero quota.
    /// * `token_budget` – maximum cumulative tokens before the circuit breaker
    ///   trips.
    pub fn with_limits(
        base_url: impl Into<String>,
        model: impl Into<String>,
        rpm: u32,
        token_budget: u64,
    ) -> Self {
        // Guard: governor panics on quota of zero – clamp to at least 1 RPM.
        let rpm = rpm.max(1);
        let quota = Quota::per_minute(
            NonZeroU32::new(rpm).expect("rpm is >= 1 after max(1) clamp above"),
        );
        let rate_limiter = Arc::new(RateLimiter::direct(quota));
        // Enforce a TLS 1.2 minimum for all HTTPS connections made by this
        // driver.  The application-level `is_secure_url` guard already rejects
        // plaintext HTTP to non-localhost hosts; the TLS version floor adds a
        // second layer of defence against protocol-downgrade attacks.
        let client = reqwest::ClientBuilder::new()
            .min_tls_version(reqwest::tls::Version::TLS_1_2)
            .build()
            .expect("failed to build reqwest client with TLS 1.2 minimum");
        Self {
            base_url: base_url.into(),
            model: model.into(),
            client,
            total_tokens: Arc::new(AtomicU64::new(0)),
            token_budget,
            rate_limiter,
        }
    }

    /// Return the cumulative number of tokens consumed since construction (or
    /// the last call to [`reset_token_counter`][Self::reset_token_counter]).
    ///
    /// The count is an estimate based on a simple word-count heuristic
    /// (tokens ≈ words × 1.3).
    pub fn total_tokens(&self) -> u64 {
        self.total_tokens.load(Ordering::Relaxed)
    }

    /// Reset the cumulative token counter and un-trip the budget circuit
    /// breaker, allowing further requests.
    pub fn reset_token_counter(&self) {
        self.total_tokens.store(0, Ordering::Relaxed);
    }

    /// Return the configured token budget.
    pub fn token_budget(&self) -> u64 {
        self.token_budget
    }

    /// Send `messages` to the model and return the assistant's reply text.
    ///
    /// `STABILITY_GUIDELINES` are automatically appended to every
    /// [`Role::System`] message so the model always receives the anti-loop
    /// rules regardless of how the caller constructs the conversation.  If no
    /// system message is present in `messages`, a new one containing only the
    /// guidelines is prepended.
    ///
    /// # Errors
    ///
    /// Returns [`LlmError::RateLimitExceeded`] when the per-minute request
    /// quota is exhausted, [`LlmError::BudgetExceeded`] when the cumulative
    /// token budget has been exhausted, [`LlmError::Http`] if the request
    /// fails, or [`LlmError::BadResponse`] if the response shape is
    /// unexpected.
    #[instrument(
        name = "llm_driver.complete",
        skip(self, messages),
        fields(
            model = %self.model,
            tokens_used_before = %self.total_tokens.load(Ordering::Relaxed),
            prompt_tokens = tracing::field::Empty,
            reply_tokens = tracing::field::Empty,
            tokens_used_after = tracing::field::Empty,
            inference_latency_ms = tracing::field::Empty,
        )
    )]
    pub async fn complete(&self, messages: &[ChatMessage]) -> Result<String, LlmError> {
        // ── TLS enforcement ────────────────────────────────────────────────
        // Reject plaintext HTTP connections to non-localhost hosts.
        if !Self::is_secure_url(&self.base_url) {
            return Err(LlmError::InsecureEndpoint(self.base_url.clone()));
        }

        // ── Budget circuit breaker ─────────────────────────────────────────
        let used = self.total_tokens.load(Ordering::Relaxed);
        if used >= self.token_budget {
            return Err(LlmError::BudgetExceeded {
                used,
                budget: self.token_budget,
            });
        }

        // ── Rate limiter ───────────────────────────────────────────────────
        if self.rate_limiter.check().is_err() {
            return Err(LlmError::RateLimitExceeded);
        }

        // Inject stability guidelines into every system message (or prepend one
        // if the caller did not supply a system message at all).
        let mut augmented: Vec<ChatMessage> = messages
            .iter()
            .map(|m| {
                if m.role == Role::System {
                    ChatMessage {
                        role: Role::System,
                        content: format!("{}\n\n{}", m.content, STABILITY_GUIDELINES),
                    }
                } else {
                    m.clone()
                }
            })
            .collect();

        if !augmented.iter().any(|m| m.role == Role::System) {
            augmented.insert(
                0,
                ChatMessage {
                    role: Role::System,
                    content: STABILITY_GUIDELINES.to_string(),
                },
            );
        }

        let url = format!("{}/v1/chat/completions", self.base_url);
        let schema = serde_json::to_value(schema_for!(HardwareIntent))
            .unwrap_or(serde_json::Value::Null);
        let body = ChatRequest {
            model: &self.model,
            messages: &augmented,
            stream: false,
            response_format: ResponseFormat {
                kind: "json_schema",
                json_schema: schema,
            },
        };

        let inference_start = Instant::now();
        let response: ChatResponse = self
            .client
            .post(&url)
            .json(&body)
            .send()
            .await?
            .error_for_status()?
            .json()
            .await?;
        let inference_latency_ms = inference_start.elapsed().as_millis() as u64;

        let reply = response
            .choices
            .into_iter()
            .next()
            .map(|c| c.message.content)
            .ok_or_else(|| LlmError::BadResponse("empty choices array".into()))?;

        // ── Token accounting ───────────────────────────────────────────────
        // Estimate tokens for both the prompt and the completion reply.
        // The heuristic (words × 1.3) approximates the BPE token count well
        // enough for budget enforcement without adding a heavy tokeniser
        // dependency.
        let prompt_tokens: u64 = augmented
            .iter()
            .map(|m| Self::estimate_tokens(&m.content))
            .sum();
        let reply_tokens = Self::estimate_tokens(&reply);
        let new_total = self
            .total_tokens
            .fetch_add(prompt_tokens + reply_tokens, Ordering::Relaxed)
            + prompt_tokens
            + reply_tokens;

        // ── Record span attributes ─────────────────────────────────────────
        let span = tracing::Span::current();
        span.record("prompt_tokens", prompt_tokens);
        span.record("reply_tokens", reply_tokens);
        span.record("tokens_used_after", new_total);
        span.record("inference_latency_ms", inference_latency_ms);
        debug!(
            model = %self.model,
            prompt_tokens,
            reply_tokens,
            tokens_used_after = new_total,
            inference_latency_ms,
            "LLM inference complete"
        );
        if new_total > self.token_budget {
            warn!(
                tokens_used = new_total,
                budget = self.token_budget,
                "token budget exceeded; further requests will be rejected"
            );
        } else if new_total == self.token_budget {
            warn!(
                tokens_used = new_total,
                budget = self.token_budget,
                "token budget reached; next request will be rejected"
            );
        }

        Ok(reply)
    }

    /// Return `true` when `url` is safe to connect to without further TLS
    /// enforcement.
    ///
    /// A URL is considered safe when it:
    /// * uses the `https://` scheme, **or**
    /// * uses `http://` but targets only a loopback address (`localhost`,
    ///   `127.0.0.1`, or `::1`), where TLS is unnecessary.
    ///
    /// All other `http://` URLs – i.e. connections to remote hosts in
    /// plaintext – are rejected to ensure external model endpoints always
    /// use TLS.
    pub(crate) fn is_secure_url(url: &str) -> bool {
        if url.starts_with("https://") {
            return true;
        }
        if let Some(without_scheme) = url.strip_prefix("http://") {
            // Strip any path/query after the host:port segment.
            let host_port = without_scheme.split('/').next().unwrap_or("");
            // Extract the host, handling both plain `host:port` and IPv6
            // `[addr]:port` forms.
            let host = if host_port.starts_with('[') {
                // IPv6 bracketed address: find the closing ']'.
                match host_port.find(']') {
                    Some(close) => &host_port[1..close],
                    // Malformed IPv6 literal – not safe to treat as localhost.
                    None => return false,
                }
            } else if let Some(idx) = host_port.rfind(':') {
                &host_port[..idx]
            } else {
                host_port
            };
            return matches!(host, "localhost" | "127.0.0.1" | "::1");
        }
        false
    }

    /// Estimate the number of tokens in `text` using a word-count heuristic.
    ///
    /// The formula `ceil(words × 1.3)` approximates BPE tokenisation for
    /// English text.  It is intentionally conservative (over-counts) so the
    /// budget circuit breaker errs on the side of caution.
    ///
    /// Implemented with integer arithmetic as `(words * 13 + 9) / 10` to
    /// avoid floating-point conversion.
    fn estimate_tokens(text: &str) -> u64 {
        let words = text.split_whitespace().count() as u64;
        // ceil(words × 1.3) == (words * 13 + 9) / 10  (integer ceiling)
        (words * 13).div_ceil(10)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chat_message_serializes_role() {
        let msg = ChatMessage {
            role: Role::System,
            content: "hello".into(),
        };
        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("\"system\""));
    }

    #[test]
    fn chat_message_role_variants_serialize() {
        let roles = [
            (Role::System, "system"),
            (Role::User, "user"),
            (Role::Assistant, "assistant"),
        ];
        for (role, expected) in roles {
            let msg = ChatMessage {
                role,
                content: String::new(),
            };
            let json = serde_json::to_string(&msg).unwrap();
            assert!(json.contains(expected));
        }
    }

    #[test]
    fn chat_message_roundtrip() {
        let msg = ChatMessage {
            role: Role::User,
            content: "What is next?".into(),
        };
        let json = serde_json::to_string(&msg).unwrap();
        let back: ChatMessage = serde_json::from_str(&json).unwrap();
        assert_eq!(back.role, Role::User);
        assert_eq!(back.content, "What is next?");
    }

    #[test]
    fn stability_guidelines_are_appended_to_system_message() {
        use super::STABILITY_GUIDELINES;
        let driver = LlmDriver::new("http://localhost:11434", "llama3");
        // We can't call driver.complete() without a live server, but we can verify
        // that building the augmented message vector works correctly by
        // replicating the logic inline and checking the content.
        let messages = [ChatMessage {
            role: Role::System,
            content: "You are a robot brain.".into(),
        }];
        let augmented: Vec<ChatMessage> = messages
            .iter()
            .map(|m| {
                if m.role == Role::System {
                    ChatMessage {
                        role: Role::System,
                        content: format!("{}\n\n{}", m.content, STABILITY_GUIDELINES),
                    }
                } else {
                    m.clone()
                }
            })
            .collect();
        let sys = augmented.iter().find(|m| m.role == Role::System).unwrap();
        assert!(
            sys.content.contains("Stability Guidelines"),
            "system message must contain stability guidelines"
        );
        assert!(
            sys.content.contains("You are a robot brain."),
            "original system content must be preserved"
        );
        // Suppress unused variable warning.
        drop(driver);
    }

    #[test]
    fn stability_guidelines_prepended_when_no_system_message() {
        use super::STABILITY_GUIDELINES;
        let messages = [ChatMessage {
            role: Role::User,
            content: "What should I do?".into(),
        }];
        let mut augmented: Vec<ChatMessage> = messages
            .iter()
            .map(|m| {
                if m.role == Role::System {
                    ChatMessage {
                        role: Role::System,
                        content: format!("{}\n\n{}", m.content, STABILITY_GUIDELINES),
                    }
                } else {
                    m.clone()
                }
            })
            .collect();
        if !augmented.iter().any(|m| m.role == Role::System) {
            augmented.insert(
                0,
                ChatMessage {
                    role: Role::System,
                    content: STABILITY_GUIDELINES.to_string(),
                },
            );
        }
        assert_eq!(augmented[0].role, Role::System);
        assert!(augmented[0].content.contains("Stability Guidelines"));
    }

    #[test]
    fn llm_driver_constructed_without_panic() {
        let _driver = LlmDriver::new("http://localhost:11434", "llama3");
    }

    #[test]
    fn hardware_intent_schema_is_injected_into_request_body() {
        use mechos_types::HardwareIntent;
        use schemars::schema_for;
        let schema = serde_json::to_value(schema_for!(HardwareIntent)).unwrap();
        let schema_str = schema.to_string();
        assert!(schema_str.contains("MoveEndEffector"));
        assert!(schema_str.contains("AskHuman"));
        assert!(schema_str.contains("Drive"));
        assert!(schema_str.contains("TriggerRelay"));
    }

    // ── Cost-control tests ────────────────────────────────────────────────────

    #[test]
    fn default_token_counter_starts_at_zero() {
        let driver = LlmDriver::new("http://localhost:11434", "llama3");
        assert_eq!(driver.total_tokens(), 0);
    }

    #[test]
    fn reset_token_counter_clears_accumulated_tokens() {
        let driver = LlmDriver::new("http://localhost:11434", "llama3");
        driver.total_tokens.store(9_999, Ordering::Relaxed);
        driver.reset_token_counter();
        assert_eq!(driver.total_tokens(), 0);
    }

    #[test]
    fn token_budget_accessor_returns_configured_value() {
        let driver = LlmDriver::with_budget("http://localhost:11434", "llama3", 50_000);
        assert_eq!(driver.token_budget(), 50_000);
    }

    #[tokio::test]
    async fn budget_circuit_breaker_trips_when_budget_exhausted() {
        // Set a tiny budget of 1 token so it's already exhausted.
        let driver = LlmDriver::with_budget("http://localhost:11434", "llama3", 1);
        // Simulate that 1 token has already been consumed.
        driver.total_tokens.store(1, Ordering::Relaxed);

        let messages = [ChatMessage {
            role: Role::User,
            content: "What next?".into(),
        }];
        let result = driver.complete(&messages).await;
        assert!(
            matches!(result, Err(LlmError::BudgetExceeded { .. })),
            "expected BudgetExceeded, got: {result:?}"
        );
    }

    #[tokio::test]
    async fn rate_limiter_trips_when_quota_exhausted() {
        // Create a driver with a tiny RPM so the single permitted token is
        // consumed before we call complete().
        let driver = LlmDriver::with_rpm("http://localhost:11434", "llama3", 1);
        // Exhaust the rate-limiter bucket by calling check() once.
        // (1 RPM = 1 token per 60 s; the bucket starts full with 1 token.)
        let _ = driver.rate_limiter.check();

        let messages = [ChatMessage {
            role: Role::User,
            content: "What next?".into(),
        }];
        let result = driver.complete(&messages).await;
        assert!(
            matches!(result, Err(LlmError::RateLimitExceeded)),
            "expected RateLimitExceeded, got: {result:?}"
        );
    }

    #[test]
    fn estimate_tokens_empty_string_returns_zero() {
        assert_eq!(LlmDriver::estimate_tokens(""), 0);
    }

    #[test]
    fn estimate_tokens_single_word() {
        // 1 word × 1.3 = 1.3, ceil = 2
        assert_eq!(LlmDriver::estimate_tokens("hello"), 2);
    }

    #[test]
    fn estimate_tokens_ten_words() {
        // 10 words × 1.3 = 13.0, ceil = 13
        assert_eq!(LlmDriver::estimate_tokens("one two three four five six seven eight nine ten"), 13);
    }

    #[test]
    fn is_secure_url_accepts_https() {
        assert!(LlmDriver::is_secure_url("https://api.openai.com"));
        assert!(LlmDriver::is_secure_url("https://api.anthropic.com/v1"));
    }

    #[test]
    fn is_secure_url_accepts_localhost_http() {
        assert!(LlmDriver::is_secure_url("http://localhost:11434"));
        assert!(LlmDriver::is_secure_url("http://127.0.0.1:11434"));
        assert!(LlmDriver::is_secure_url("http://localhost:11434/v1/chat"));
        assert!(LlmDriver::is_secure_url("http://[::1]:11434"));
    }

    #[test]
    fn is_secure_url_rejects_external_http() {
        assert!(!LlmDriver::is_secure_url("http://api.openai.com"));
        assert!(!LlmDriver::is_secure_url("http://my-robot-server:8080"));
        assert!(!LlmDriver::is_secure_url("http://192.168.1.1:11434"));
    }

    #[test]
    fn is_secure_url_rejects_malformed_ipv6() {
        // Missing closing bracket – must not be treated as localhost.
        assert!(!LlmDriver::is_secure_url("http://[::1:11434"));
    }

    #[tokio::test]
    async fn complete_returns_insecure_endpoint_for_external_http() {
        let driver = LlmDriver::new("http://external-server:11434", "llama3");
        let messages = [ChatMessage {
            role: Role::User,
            content: "Hello".into(),
        }];
        let result = driver.complete(&messages).await;
        assert!(
            matches!(result, Err(LlmError::InsecureEndpoint(_))),
            "expected InsecureEndpoint, got: {result:?}"
        );
    }

    #[test]
    fn with_limits_clamps_zero_rpm_to_one() {
        // Should not panic (governor would panic on zero quota).
        let driver = LlmDriver::with_limits("http://localhost:11434", "llama3", 0, 100_000);
        // The bucket should have capacity for exactly 1 request per minute.
        assert!(driver.rate_limiter.check().is_ok());
    }

    #[test]
    fn llm_driver_client_is_built_with_tls_minimum_without_panic() {
        // Constructing a driver should succeed: the ClientBuilder with
        // min_tls_version(TLS_1_2) must not panic or return an error.
        let driver = LlmDriver::new("http://localhost:11434", "llama3");
        // A non-zero model name confirms the struct was fully initialised.
        assert!(!driver.model.is_empty());
    }
}
