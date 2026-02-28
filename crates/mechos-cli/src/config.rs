//! Configuration Vault – reads/writes `~/.mechos/config.toml`.

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::PathBuf;

/// Supported AI provider choices.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
#[derive(Default)]
pub enum AiProvider {
    #[default]
    Ollama,
    OpenAI,
    Anthropic,
}


impl std::fmt::Display for AiProvider {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AiProvider::Ollama => write!(f, "ollama"),
            AiProvider::OpenAI => write!(f, "openai"),
            AiProvider::Anthropic => write!(f, "anthropic"),
        }
    }
}

/// Persisted user configuration stored in `~/.mechos/config.toml`.
#[derive(Clone, Serialize, Deserialize)]
pub struct Config {
    /// WebSocket port for the ROS 2 Dashboard / rosbridge adapter.
    #[serde(default = "default_dashboard_port")]
    pub dashboard_port: u16,

    /// HTTP port for the MechOS Web UI.
    #[serde(default = "default_webui_port")]
    pub webui_port: u16,

    /// Chosen AI provider.
    #[serde(default)]
    pub ai_provider: AiProvider,

    /// Active model name (e.g. "llama3", "gpt-4o").
    #[serde(default = "default_model")]
    pub active_model: String,

    /// Base URL of the Ollama instance.
    #[serde(default = "default_ollama_url")]
    pub ollama_url: String,

    /// OpenAI API key (stored as plain text – users should restrict file
    /// permissions on `~/.mechos/config.toml`).
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub openai_api_key: String,

    /// Anthropic API key.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub anthropic_api_key: String,
}

impl std::fmt::Debug for Config {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Config")
            .field("dashboard_port", &self.dashboard_port)
            .field("webui_port", &self.webui_port)
            .field("ai_provider", &self.ai_provider)
            .field("active_model", &self.active_model)
            .field("ollama_url", &self.ollama_url)
            .field(
                "openai_api_key",
                if self.openai_api_key.is_empty() { &"<not set>" } else { &"<redacted>" },
            )
            .field(
                "anthropic_api_key",
                if self.anthropic_api_key.is_empty() { &"<not set>" } else { &"<redacted>" },
            )
            .finish()
    }
}

fn default_dashboard_port() -> u16 {
    9090
}
fn default_webui_port() -> u16 {
    8080
}
fn default_model() -> String {
    "llama3".to_string()
}
fn default_ollama_url() -> String {
    "http://localhost:11434".to_string()
}

impl Default for Config {
    fn default() -> Self {
        Self {
            dashboard_port: default_dashboard_port(),
            webui_port: default_webui_port(),
            ai_provider: AiProvider::default(),
            active_model: default_model(),
            ollama_url: default_ollama_url(),
            openai_api_key: String::new(),
            anthropic_api_key: String::new(),
        }
    }
}

/// Return the path to `~/.mechos/config.toml`.
pub fn config_path() -> PathBuf {
    config_path_for_home(
        &std::env::var("HOME")
            .or_else(|_| std::env::var("USERPROFILE"))
            .unwrap_or_else(|_| ".".to_string()),
    )
}

/// Build the config path relative to the given home directory.
/// Extracted for testability without mutating environment variables.
pub(crate) fn config_path_for_home(home: &str) -> PathBuf {
    PathBuf::from(home).join(".mechos").join("config.toml")
}

/// Load the config from disk.  Returns `None` if the file does not exist.
pub fn load() -> Result<Option<Config>, String> {
    load_from(&config_path())
}

/// Load the config from a specific path.
pub(crate) fn load_from(path: &PathBuf) -> Result<Option<Config>, String> {
    if !path.exists() {
        return Ok(None);
    }
    let raw = fs::read_to_string(path)
        .map_err(|e| format!("Failed to read config at {}: {}", path.display(), e))?;
    let mut cfg: Config = toml::from_str(&raw)
        .map_err(|e| format!("Failed to parse config: {}", e))?;
    apply_env_overrides(&mut cfg);
    Ok(Some(cfg))
}

/// Apply `MECHOS_*` environment variable overrides to `cfg`.
///
/// Supported variables:
///
/// | Variable | Config field |
/// |---|---|
/// | `MECHOS_OLLAMA_URL` | `ollama_url` |
/// | `MECHOS_MODEL` | `active_model` |
/// | `MECHOS_DASHBOARD_PORT` | `dashboard_port` |
/// | `MECHOS_WEBUI_PORT` | `webui_port` |
pub fn apply_env_overrides(cfg: &mut Config) {
    if let Ok(v) = std::env::var("MECHOS_OLLAMA_URL") {
        cfg.ollama_url = v;
    }
    if let Ok(v) = std::env::var("MECHOS_MODEL") {
        cfg.active_model = v;
    }
    if let Ok(v) = std::env::var("MECHOS_DASHBOARD_PORT")
        && let Ok(port) = v.parse::<u16>() {
            cfg.dashboard_port = port;
        }
    if let Ok(v) = std::env::var("MECHOS_WEBUI_PORT")
        && let Ok(port) = v.parse::<u16>() {
            cfg.webui_port = port;
        }
}

/// Save the config to disk, creating `~/.mechos/` if necessary.
pub fn save(cfg: &Config) -> Result<(), String> {
    save_to(cfg, &config_path())
}

/// Save the config to a specific path.
pub(crate) fn save_to(cfg: &Config, path: &PathBuf) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .map_err(|e| format!("Failed to create config directory: {}", e))?;
        // Restrict the config directory to the owner only (rwx------) on Unix.
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(parent, fs::Permissions::from_mode(0o700))
                .map_err(|e| format!("Failed to set config directory permissions: {}", e))?;
        }
    }
    let raw = toml::to_string_pretty(cfg)
        .map_err(|e| format!("Failed to serialize config: {}", e))?;
    // Write the file with owner-only read/write (rw-------) on Unix.
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        fs::OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .mode(0o600)
            .open(path)
            .and_then(|mut f| {
                use std::io::Write;
                f.write_all(raw.as_bytes())
            })
            .map_err(|e| format!("Failed to write config at {}: {}", path.display(), e))?;
    }
    #[cfg(not(unix))]
    fs::write(path, raw)
        .map_err(|e| format!("Failed to write config at {}: {}", path.display(), e))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_debug_redacts_api_keys() {
        let mut cfg = Config::default();
        cfg.openai_api_key = "sk-super-secret".to_string();
        cfg.anthropic_api_key = "ant-super-secret".to_string();
        let debug_str = format!("{:?}", cfg);
        assert!(!debug_str.contains("sk-super-secret"), "openai key must not appear in debug output");
        assert!(!debug_str.contains("ant-super-secret"), "anthropic key must not appear in debug output");
        assert!(debug_str.contains("<redacted>"), "debug output must show <redacted> for set keys");
    }

    #[test]
    fn config_debug_shows_not_set_for_empty_keys() {
        let cfg = Config::default();
        let debug_str = format!("{:?}", cfg);
        assert!(debug_str.contains("<not set>"), "empty API key must show <not set> in debug output");
    }

    #[cfg(unix)]
    #[test]
    fn config_file_has_restrictive_permissions() {
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().expect("tmp dir");
        let path = config_path_for_home(&dir.path().to_string_lossy());

        let cfg = Config::default();
        save_to(&cfg, &path).expect("save");

        let file_meta = std::fs::metadata(&path).expect("file metadata");
        let file_mode = file_meta.permissions().mode() & 0o777;
        assert_eq!(file_mode, 0o600, "config file must have 0o600 permissions");

        let dir_meta = std::fs::metadata(path.parent().unwrap()).expect("dir metadata");
        let dir_mode = dir_meta.permissions().mode() & 0o777;
        assert_eq!(dir_mode, 0o700, "config directory must have 0o700 permissions");
    }

    #[test]
    fn roundtrip_default_config() {
        let dir = tempfile::tempdir().expect("tmp dir");
        let path = config_path_for_home(&dir.path().to_string_lossy());

        let cfg = Config::default();
        save_to(&cfg, &path).expect("save");

        let loaded = load_from(&path).expect("load ok").expect("some");
        assert_eq!(loaded.dashboard_port, 9090);
        assert_eq!(loaded.webui_port, 8080);
        assert_eq!(loaded.active_model, "llama3");
        assert_eq!(loaded.ai_provider, AiProvider::Ollama);
    }

    #[test]
    fn config_path_points_to_mechos_dir() {
        let p = config_path_for_home("/home/testuser");
        assert!(p.to_string_lossy().contains(".mechos"));
        assert!(p.to_string_lossy().ends_with("config.toml"));
    }

    #[test]
    fn load_from_returns_none_when_missing() {
        let dir = tempfile::tempdir().expect("tmp dir");
        let path = config_path_for_home(&dir.path().to_string_lossy());
        let result = load_from(&path).expect("no error");
        assert!(result.is_none());
    }

    #[test]
    fn apply_env_overrides_changes_ollama_url() {
        // SAFETY: single-threaded test; no data races on env vars.
        unsafe { std::env::set_var("MECHOS_OLLAMA_URL", "http://robot-host:11434") };
        let mut cfg = Config::default();
        apply_env_overrides(&mut cfg);
        assert_eq!(cfg.ollama_url, "http://robot-host:11434");
        unsafe { std::env::remove_var("MECHOS_OLLAMA_URL") };
    }

    #[test]
    fn apply_env_overrides_changes_model() {
        // SAFETY: single-threaded test; no data races on env vars.
        unsafe { std::env::set_var("MECHOS_MODEL", "gpt-4o") };
        let mut cfg = Config::default();
        apply_env_overrides(&mut cfg);
        assert_eq!(cfg.active_model, "gpt-4o");
        unsafe { std::env::remove_var("MECHOS_MODEL") };
    }

    #[test]
    fn apply_env_overrides_changes_dashboard_port() {
        // SAFETY: single-threaded test; no data races on env vars.
        unsafe { std::env::set_var("MECHOS_DASHBOARD_PORT", "9999") };
        let mut cfg = Config::default();
        apply_env_overrides(&mut cfg);
        assert_eq!(cfg.dashboard_port, 9999);
        unsafe { std::env::remove_var("MECHOS_DASHBOARD_PORT") };
    }

    #[test]
    fn apply_env_overrides_ignores_invalid_port() {
        // SAFETY: single-threaded test; no data races on env vars.
        unsafe { std::env::set_var("MECHOS_DASHBOARD_PORT", "not-a-port") };
        let mut cfg = Config::default();
        let original_port = cfg.dashboard_port;
        apply_env_overrides(&mut cfg);
        assert_eq!(cfg.dashboard_port, original_port);
        unsafe { std::env::remove_var("MECHOS_DASHBOARD_PORT") };
    }

    #[test]
    fn apply_env_overrides_changes_webui_port() {
        // SAFETY: single-threaded test; no data races on env vars.
        unsafe { std::env::set_var("MECHOS_WEBUI_PORT", "8181") };
        let mut cfg = Config::default();
        apply_env_overrides(&mut cfg);
        assert_eq!(cfg.webui_port, 8181);
        unsafe { std::env::remove_var("MECHOS_WEBUI_PORT") };
    }
}
