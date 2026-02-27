//! Ollama auto-discovery helpers.
//!
//! Silently pings `http://localhost:11434` and, if responsive, fetches the
//! list of locally downloaded models from the `/api/tags` endpoint.

use serde::Deserialize;

/// A single model entry returned by Ollama's `/api/tags` endpoint.
#[derive(Debug, Clone, Deserialize)]
pub struct OllamaModel {
    pub name: String,
}

/// Raw shape of the `/api/tags` JSON response.
#[derive(Debug, Deserialize)]
struct TagsResponse {
    models: Vec<OllamaModel>,
}

/// Ping the Ollama server and return the list of available models.
///
/// Returns `Ok(models)` when Ollama is running and reachable, or
/// `Err(reason)` when it is not (server offline, network error, etc.).
pub fn fetch_models(base_url: &str) -> Result<Vec<OllamaModel>, String> {
    let url = format!("{}/api/tags", base_url.trim_end_matches('/'));
    let response = reqwest::blocking::get(&url)
        .map_err(|e| format!("Ollama unreachable at {}: {}", url, e))?;

    if !response.status().is_success() {
        return Err(format!(
            "Ollama returned HTTP {}",
            response.status()
        ));
    }

    let tags: TagsResponse = response
        .json()
        .map_err(|e| format!("Failed to parse Ollama response: {}", e))?;

    Ok(tags.models)
}

/// Returns `true` if the Ollama server is reachable.
pub fn is_running(base_url: &str) -> bool {
    fetch_models(base_url).is_ok()
}
