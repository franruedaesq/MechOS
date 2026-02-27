//! REPL – Read-Eval-Print Loop for the MechOS interactive shell.
//!
//! Supported slash-commands:
//!   /help         – show this list
//!   /settings     – interactively edit `~/.mechos/config.toml`
//!   /models       – list / switch the active AI model
//!   /connections  – run an adapter connectivity diagnostic
//!   /start        – initiate the boot sequence
//!   /quit | /exit – gracefully exit the CLI

use colored::Colorize;
use std::io::{self, BufRead, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::config::{self, AiProvider, Config};
use crate::ollama;

/// Internal OS state machine.
#[derive(Debug, Clone, PartialEq)]
#[allow(dead_code)]
pub enum AppState {
    Offline,
    Booting,
    Running,
}

/// Entry point for the interactive REPL.
///
/// `shutdown` is polled each iteration; when set the REPL exits cleanly.
pub fn run(shutdown: Arc<AtomicBool>) {
    let stdin = io::stdin();
    let mut stdout = io::stdout();

    loop {
        if shutdown.load(Ordering::SeqCst) {
            break;
        }

        print!("{} ", "mechos>".bold().cyan());
        stdout.flush().ok();

        let mut line = String::new();
        match stdin.lock().read_line(&mut line) {
            Ok(0) => break, // EOF
            Ok(_) => {}
            Err(e) => {
                eprintln!("{}: {}", "Read error".red(), e);
                break;
            }
        }

        let cmd = line.trim();
        if cmd.is_empty() {
            continue;
        }

        match cmd {
            "/help" => cmd_help(),
            "/settings" => cmd_settings(),
            "/models" => cmd_models(),
            "/connections" => cmd_connections(),
            "/start" => {
                cmd_start();
                // After boot sequence completes we return to the prompt.
            }
            "/quit" | "/exit" => {
                println!("{}", "Goodbye.".green());
                shutdown.store(true, Ordering::SeqCst);
                break;
            }
            other => {
                println!(
                    "{} '{}'. Type {} for available commands.",
                    "Unknown command:".red(),
                    other.yellow(),
                    "/help".bold()
                );
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Command handlers
// ─────────────────────────────────────────────────────────────────────────────

fn cmd_help() {
    println!();
    println!("{}", "MechOS Commands".bold().underline());
    println!("  {}  – edit ~/.mechos/config.toml settings", "/settings".bold().cyan());
    println!("  {}    – list and switch AI models",           "/models".bold().cyan());
    println!("  {}  – adapter connectivity diagnostic",  "/connections".bold().cyan());
    println!("  {}     – initiate the OS boot sequence",      "/start".bold().cyan());
    println!("  {}  – exit the CLI",             "/quit  /exit".bold().cyan());
    println!();
}

fn cmd_settings() {
    let mut cfg = match config::load() {
        Ok(Some(c)) => c,
        Ok(None) => Config::default(),
        Err(e) => {
            println!("{}: {}", "Error loading config".red(), e);
            return;
        }
    };

    println!("{}", "Settings Editor".bold().underline());
    println!(
        "  Dashboard port : {} (enter new value or press Enter to keep)",
        cfg.dashboard_port.to_string().yellow()
    );
    let port = prompt_u16(&format!("  Dashboard port [{}]: ", cfg.dashboard_port), cfg.dashboard_port);
    cfg.dashboard_port = port;

    println!(
        "  Web UI port    : {}",
        cfg.webui_port.to_string().yellow()
    );
    let port = prompt_u16(&format!("  Web UI port    [{}]: ", cfg.webui_port), cfg.webui_port);
    cfg.webui_port = port;

    println!(
        "  AI provider    : {} (ollama / openai / anthropic)",
        cfg.ai_provider.to_string().yellow()
    );
    let provider = prompt_str(&format!("  AI provider    [{}]: ", cfg.ai_provider), &cfg.ai_provider.to_string());
    cfg.ai_provider = match provider.to_lowercase().as_str() {
        "openai"    => AiProvider::OpenAI,
        "anthropic" => AiProvider::Anthropic,
        _           => AiProvider::Ollama,
    };

    println!(
        "  Active model   : {}",
        cfg.active_model.yellow()
    );
    let model = prompt_str(&format!("  Active model   [{}]: ", cfg.active_model), &cfg.active_model);
    cfg.active_model = model;

    match config::save(&cfg) {
        Ok(()) => println!(
            "{} {}",
            "✓ Settings saved to".green(),
            config::config_path().display().to_string().bold()
        ),
        Err(e) => println!("{}: {}", "Error saving config".red(), e),
    }
}

fn cmd_models() {
    let cfg = load_config_or_default();

    println!("{}", "AI Models".bold().underline());
    println!("  Active model : {}", cfg.active_model.yellow());

    if cfg.ai_provider == AiProvider::Ollama {
        print!("  Probing Ollama at {} … ", cfg.ollama_url.dimmed());
        io::stdout().flush().ok();

        match ollama::fetch_models(&cfg.ollama_url) {
            Ok(models) if models.is_empty() => {
                println!("{}", "no models found".yellow());
                println!("  Run `ollama pull llama3` to download a model.");
            }
            Ok(models) => {
                println!("{}", "online".green());
                println!("  Available local models:");
                for m in &models {
                    let marker = if m.name == cfg.active_model { "▶" } else { " " };
                    println!("    {} {}", marker.green(), m.name.bold());
                }

                let new_model = prompt_str(
                    &format!("  Switch to model [{}]: ", cfg.active_model),
                    &cfg.active_model,
                );
                if new_model != cfg.active_model {
                    let valid = models.iter().any(|m| m.name == new_model);
                    if valid {
                        let mut new_cfg = cfg.clone();
                        new_cfg.active_model = new_model.clone();
                        match config::save(&new_cfg) {
                            Ok(()) => println!("{} {}", "✓ Active model set to".green(), new_model.bold()),
                            Err(e) => println!("{}: {}", "Error saving config".red(), e),
                        }
                    } else {
                        println!("{} '{}'", "Unknown model:".red(), new_model.yellow());
                    }
                }
            }
            Err(e) => {
                println!("{}", "offline".red());
                println!("  {}", e.dimmed());
                println!("  Is Ollama running?  Try: ollama serve");
            }
        }
    } else {
        println!("  Provider: {}", cfg.ai_provider.to_string().yellow());
        println!("  (Cloud model selection is managed via the API key settings.)");
    }
}

fn cmd_connections() {
    let cfg = load_config_or_default();

    println!("{}", "Connection Diagnostics".bold().underline());

    // Dashboard adapter
    let dash_url = format!("ws://localhost:{}", cfg.dashboard_port);
    println!(
        "  {} DashboardSimAdapter configured for {}",
        "🟡".yellow(),
        dash_url.bold()
    );
    println!(
        "     (adapter binds when /start is executed)"
    );

    // Ollama
    print!("  Probing Ollama at {} … ", cfg.ollama_url.dimmed());
    io::stdout().flush().ok();
    if ollama::is_running(&cfg.ollama_url) {
        println!(
            "{} {} {}",
            "🟢".green(),
            "Ollama".bold(),
            "is running".green()
        );
    } else {
        println!(
            "{} {} {}",
            "🔴".red(),
            "Ollama".bold(),
            "not detected".red()
        );
    }

    // Physical LiDAR (not yet implemented – indicate status)
    println!(
        "  {} Physical LiDAR: {}",
        "🔴".red(),
        "not detected (hardware driver not loaded)".dimmed()
    );
}

fn cmd_start() {
    let cfg = load_config_or_default();

    println!();
    println!("{}", "═══════════════════════════════════════".bold());
    println!("{}", "         MechOS Boot Sequence          ".bold().cyan());
    println!("{}", "═══════════════════════════════════════".bold());

    // ── Step 1 – Memory ────────────────────────────────────────────────────
    print!("  [1/6] {} … ", "Initializing Memory (SQLite)".bold());
    io::stdout().flush().ok();
    match mechos_memory::episodic::EpisodicStore::open_in_memory() {
        Ok(_) => println!("{}", "OK".green()),
        Err(e) => {
            println!("{}: {}", "FAILED".red(), e);
            return;
        }
    }

    // ── Step 2 – Event Bus ─────────────────────────────────────────────────
    print!("  [2/6] {} … ", "Initializing Event Bus".bold());
    io::stdout().flush().ok();
    let bus = std::sync::Arc::new(mechos_middleware::EventBus::new(256));
    println!("{}", "OK".green());

    // ── Step 3 – Kernel Safety Interlocks ──────────────────────────────────
    print!("  [3/6] {} … ", "Engaging Kernel Safety Interlocks".bold());
    io::stdout().flush().ok();
    let _watchdog = mechos_kernel::Watchdog::new();
    let _cap_mgr  = mechos_kernel::CapabilityManager::new();
    println!("{}", "OK".green());

    // ── Step 4 – Dashboard Adapter ─────────────────────────────────────────
    let dash_url = format!("ws://localhost:{}", cfg.dashboard_port);
    print!("  [4/6] {} {} … ", "Binding DashboardSimAdapter on".bold(), dash_url.yellow());
    io::stdout().flush().ok();
    let _adapter = mechos_middleware::DashboardSimAdapter::new(
        bus.clone(),
        dash_url.clone(),
    );
    println!("{}", "OK".green());

    // ── Step 5 – Web UI ────────────────────────────────────────────────────
    print!("  [5/6] {} {} … ", "Web UI port configured:".bold(), cfg.webui_port.to_string().yellow());
    io::stdout().flush().ok();
    println!("{}", "OK".green());

    // ── Step 6 – Runtime Brain ─────────────────────────────────────────────
    print!(
        "  [6/6] {} {} … ",
        "Booting Runtime Brain (model:".bold(),
        cfg.active_model.yellow()
    );
    io::stdout().flush().ok();
    let loop_config = mechos_runtime::AgentLoopConfig {
        llm_base_url: cfg.ollama_url.clone(),
        llm_model: cfg.active_model.clone(),
        ..Default::default()
    };
    let _agent = match mechos_runtime::AgentLoop::new(loop_config) {
        Ok(agent) => agent,
        Err(e) => {
            println!("{} {}", "ERROR".red(), e);
            return;
        }
    };
    println!("{}", "OK".green());

    println!("{}", "═══════════════════════════════════════".bold());
    println!(
        "  {} MechOS is {}. Type {} to stop.",
        "✓".green().bold(),
        "RUNNING".green().bold(),
        "/quit".bold()
    );
    println!("{}", "═══════════════════════════════════════".bold());
    println!();
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

fn load_config_or_default() -> Config {
    match config::load() {
        Ok(Some(c)) => c,
        Ok(None) => Config::default(),
        Err(e) => {
            println!("{}: {} – using defaults", "Config error".red(), e);
            Config::default()
        }
    }
}

/// Prompt for a u16 value.  Returns `default` when the user presses Enter.
fn prompt_u16(msg: &str, default: u16) -> u16 {
    let raw = prompt_str(msg, &default.to_string());
    match raw.parse::<u16>() {
        Ok(v) => v,
        Err(_) => {
            println!(
                "  {} '{}' is not a valid port number, keeping {}",
                "Warning:".yellow(),
                raw,
                default
            );
            default
        }
    }
}

/// Prompt for a string value.  Returns `default` when the user presses Enter.
fn prompt_str(msg: &str, default: &str) -> String {
    print!("{}", msg);
    io::stdout().flush().ok();

    let mut line = String::new();
    match io::stdin().lock().read_line(&mut line) {
        Ok(_) => {
            let trimmed = line.trim().to_string();
            if trimmed.is_empty() {
                default.to_string()
            } else {
                trimmed
            }
        }
        Err(_) => default.to_string(),
    }
}
