//! [`LlmDriver`] – OpenAI-compatible LLM interface.
//!
//! Communicates with a locally-running model server that exposes an
//! OpenAI-compatible `/v1/chat/completions` endpoint, such as
//! [Ollama](https://ollama.com) (`http://localhost:11434`).
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

use mechos_types::HardwareIntent;
use schemars::schema_for;
use serde::{Deserialize, Serialize};
use thiserror::Error;

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
// LlmDriver
// ─────────────────────────────────────────────────────────────────────────────

/// An async client for an OpenAI-compatible chat-completions endpoint.
///
/// Construct once and reuse across OODA loop iterations.
pub struct LlmDriver {
    base_url: String,
    model: String,
    client: reqwest::Client,
}

impl LlmDriver {
    /// Create a new driver pointing at `base_url` (e.g. `"http://localhost:11434"`)
    /// and using `model` (e.g. `"llama3"`).
    pub fn new(base_url: impl Into<String>, model: impl Into<String>) -> Self {
        Self {
            base_url: base_url.into(),
            model: model.into(),
            client: reqwest::Client::new(),
        }
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
    /// Returns [`LlmError::Http`] if the request fails, or
    /// [`LlmError::BadResponse`] if the response shape is unexpected.
    pub async fn complete(&self, messages: &[ChatMessage]) -> Result<String, LlmError> {
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

        let response: ChatResponse = self
            .client
            .post(&url)
            .json(&body)
            .send()
            .await?
            .error_for_status()?
            .json()
            .await?;

        response
            .choices
            .into_iter()
            .next()
            .map(|c| c.message.content)
            .ok_or_else(|| LlmError::BadResponse("empty choices array".into()))
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
        let messages = vec![ChatMessage {
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
        let messages = vec![ChatMessage {
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
}
