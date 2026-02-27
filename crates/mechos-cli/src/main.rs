//! `mechos-cli` – MechOS Command Line Interface
//!
//! This binary is the primary entry point ("ignition switch") for the MechOS
//! stack.  It:
//!
//! 1. Checks for `~/.mechos/config.toml`; runs a **First-Run Wizard** when the
//!    file is absent.
//! 2. Probes the local Ollama instance and reports available AI models.
//! 3. Drops the user into an **interactive REPL** with slash-commands
//!    (`/settings`, `/models`, `/connections`, `/start`, `/help`).
//! 4. Intercepts **Ctrl-C** to send an `EmergencyStop` intent and exit safely.

mod config;
mod ollama;
mod repl;

use colored::Colorize;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use tracing::warn;

use mechos_middleware::{EventBus, Topic};
use mechos_types::{Event, EventPayload};

fn main() {
    // ── Structured logging ────────────────────────────────────────────────
    // Initialise tracing-subscriber using RUST_LOG (defaults to "info").
    // Set MECHOS_LOG_FORMAT=json to emit newline-delimited JSON logs suitable
    // for log aggregators (e.g. Loki, Datadog, OpenTelemetry Collector).
    // The CLI's user-facing output still uses println! for UX consistency.
    let log_level = std::env::var("RUST_LOG").unwrap_or_else(|_| "info".to_string());
    let env_filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new(&log_level));

    if std::env::var("MECHOS_LOG_FORMAT").as_deref() == Ok("json") {
        tracing_subscriber::fmt()
            .with_env_filter(env_filter)
            .with_target(true)
            .json()
            .init();
    } else {
        tracing_subscriber::fmt()
            .with_env_filter(env_filter)
            .with_target(true)
            .compact()
            .init();
    }

    print_banner();

    // ── Shared shutdown flag ──────────────────────────────────────────────
    let shutdown = Arc::new(AtomicBool::new(false));
    let shutdown_clone = shutdown.clone();

    // ── Ctrl-C handler ────────────────────────────────────────────────────
    // Build a bus so we can publish the EmergencyStop intent on shutdown.
    let bus_for_ctrlc = Arc::new(EventBus::new(64));
    let bus_ctrlc_ref = bus_for_ctrlc.clone();

    if let Err(e) = ctrlc::set_handler(move || {
        println!();
        println!("{}", "⚠  Ctrl-C received – initiating graceful shutdown …".yellow().bold());

        // Publish an EmergencyStop fault event to the event bus so any active
        // adapter or agent loop can react to it.
        let stop_event = Event {
            id: uuid::Uuid::new_v4(),
            timestamp: chrono::Utc::now(),
            source: "mechos-cli".to_string(),
            payload: EventPayload::HardwareFault {
                component: "cli".to_string(),
                code: 911,
                message: "EMERGENCY_STOP: operator Ctrl-C".to_string(),
            },
        };
        let _ = bus_ctrlc_ref.publish_to(Topic::SystemAlerts, stop_event);

        println!("{}", "  ✓ EmergencyStop published to Event Bus.".green());
        println!("{}", "  ✓ Exiting MechOS.".green());

        shutdown_clone.store(true, Ordering::SeqCst);
    }) {
        warn!(error = %e, "Failed to install Ctrl-C handler; graceful shutdown on Ctrl-C will not be available");
    }

    // ── First-Run Wizard ──────────────────────────────────────────────────
    match config::load() {
        Ok(None) => run_first_run_wizard(),
        Ok(Some(_)) => {
            println!(
                "  Config loaded from {}",
                config::config_path().display().to_string().bold()
            );
        }
        Err(e) => {
            println!("{}: {}", "Config error".red(), e);
            println!("  Using default configuration.");
        }
    }

    // ── Ollama discovery ──────────────────────────────────────────────────
    let cfg = config::load()
        .ok()
        .flatten()
        .unwrap_or_default();

    print!("\n  Probing Ollama at {} … ", cfg.ollama_url.dimmed());
    match ollama::fetch_models(&cfg.ollama_url) {
        Ok(models) => {
            println!("{} ({} model(s) available)", "online".green(), models.len());
            if !models.is_empty() {
                println!("  Available models:");
                for m in &models {
                    println!("    • {}", m.name.bold());
                }
            }
        }
        Err(_) => {
            println!("{}", "offline".yellow());
            println!(
                "  {}  Run `{}` to start a local AI.",
                "No Ollama instance detected.".dimmed(),
                "ollama serve".bold()
            );
        }
    }

    println!();
    println!(
        "  Type {} for a list of commands.\n",
        "/help".bold().cyan()
    );

    // ── Interactive REPL ──────────────────────────────────────────────────
    repl::run(shutdown);
}

// ─────────────────────────────────────────────────────────────────────────────
// First-Run Wizard
// ─────────────────────────────────────────────────────────────────────────────

fn run_first_run_wizard() {
    println!();
    println!("{}", "  ╔══════════════════════════════════════╗".bold().cyan());
    println!("{}", "  ║       MechOS First-Run Wizard        ║".bold().cyan());
    println!("{}", "  ╚══════════════════════════════════════╝".bold().cyan());
    println!();
    println!("  No configuration found.  Let's set up MechOS.\n");

    let mut cfg = config::Config::default();

    // AI provider
    println!("  Which AI provider would you like to use?");
    println!("    1) Local AI via Ollama  (default, offline-first)");
    println!("    2) Cloud AI via OpenAI");
    println!("    3) Cloud AI via Anthropic");
    let choice = prompt_line("  Enter choice [1]: ", "1");
    match choice.trim() {
        "2" => cfg.ai_provider = config::AiProvider::OpenAI,
        "3" => cfg.ai_provider = config::AiProvider::Anthropic,
        _   => cfg.ai_provider = config::AiProvider::Ollama,
    }

    // Dashboard port
    let port_str = prompt_line(
        &format!("  Dashboard (rosbridge) WebSocket port [{}]: ", cfg.dashboard_port),
        &cfg.dashboard_port.to_string(),
    );
    if let Ok(p) = port_str.trim().parse::<u16>() {
        cfg.dashboard_port = p;
    }

    // Web UI port
    let port_str = prompt_line(
        &format!("  Web UI HTTP port [{}]: ", cfg.webui_port),
        &cfg.webui_port.to_string(),
    );
    if let Ok(p) = port_str.trim().parse::<u16>() {
        cfg.webui_port = p;
    }

    match config::save(&cfg) {
        Ok(()) => println!(
            "\n  {} Config saved to {}\n",
            "✓".green().bold(),
            config::config_path().display().to_string().bold()
        ),
        Err(e) => println!("{}: {}", "Error saving config".red(), e),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Banner
// ─────────────────────────────────────────────────────────────────────────────

fn print_banner() {
    println!();
    println!("{}", r#"   __  ___        __   ____  _____"#.bold().cyan());
    println!("{}", r#"  /  |/  /__ ___ / /  / __ \/ ___/"#.bold().cyan());
    println!("{}", r#" / /|_/ / -_) __/ _ \/ /_/ /\__ \ "#.bold().cyan());
    println!("{}", r#"/_/  /_/\__/\__/_//_/\____/____/  "#.bold().cyan());
    println!();
    println!("  {} {}",
        "MechOS".bold(),
        format!("v{}", env!("CARGO_PKG_VERSION")).dimmed()
    );
    println!("  Autonomous Robot Operating System");
    println!();
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

fn prompt_line(msg: &str, default: &str) -> String {
    use std::io::{BufRead, Write};
    print!("{}", msg);
    std::io::stdout().flush().ok();
    let mut line = String::new();
    match std::io::stdin().lock().read_line(&mut line) {
        Ok(_) => {
            let t = line.trim().to_string();
            if t.is_empty() { default.to_string() } else { t }
        }
        Err(_) => default.to_string(),
    }
}
