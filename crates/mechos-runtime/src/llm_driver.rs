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

use serde::{Deserialize, Serialize};
use thiserror::Error;

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

#[derive(Serialize)]
struct ChatRequest<'a> {
    model: &'a str,
    messages: &'a [ChatMessage],
    stream: bool,
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

/// A synchronous client for an OpenAI-compatible chat-completions endpoint.
///
/// Construct once and reuse across OODA loop iterations.
pub struct LlmDriver {
    base_url: String,
    model: String,
    client: reqwest::blocking::Client,
}

impl LlmDriver {
    /// Create a new driver pointing at `base_url` (e.g. `"http://localhost:11434"`)
    /// and using `model` (e.g. `"llama3"`).
    pub fn new(base_url: impl Into<String>, model: impl Into<String>) -> Self {
        Self {
            base_url: base_url.into(),
            model: model.into(),
            client: reqwest::blocking::Client::new(),
        }
    }

    /// Send `messages` to the model and return the assistant's reply text.
    ///
    /// # Errors
    ///
    /// Returns [`LlmError::Http`] if the request fails, or
    /// [`LlmError::BadResponse`] if the response shape is unexpected.
    pub fn complete(&self, messages: &[ChatMessage]) -> Result<String, LlmError> {
        let url = format!("{}/v1/chat/completions", self.base_url);
        let body = ChatRequest {
            model: &self.model,
            messages,
            stream: false,
        };

        let response: ChatResponse = self
            .client
            .post(&url)
            .json(&body)
            .send()?
            .error_for_status()?
            .json()?;

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
    fn llm_driver_constructed_without_panic() {
        let _driver = LlmDriver::new("http://localhost:11434", "llama3");
    }
}
