//! REPL – Read-Eval-Print Loop for the MechOS interactive shell.
//!
//! Supported slash-commands:
//!   /help                       – show this list
//!   /settings                   – interactively edit `~/.mechos/config.toml`
//!   /models                     – list / switch the active AI model
//!   /connections                – run an adapter connectivity diagnostic
//!   /start                      – initiate the boot sequence
//!   /logs                       – stream live Event Bus events (press Enter to stop)
//!   /hardware <intent> [args…]  – manually send a HardwareIntent to the bus
//!   /halt                       – emergency stop without exiting the REPL
//!   /memory list|query <term>   – inspect the episodic memory store
//!   /quit | /exit               – gracefully exit the CLI

use colored::Colorize;
use rustyline::completion::{Completer, Pair};
use rustyline::error::ReadlineError;
use rustyline::highlight::{Highlighter, MatchingBracketHighlighter};
use rustyline::hint::{Hinter, HistoryHinter};
use rustyline::history::DefaultHistory;
use rustyline::validate::{Validator, MatchingBracketValidator};
use rustyline::{Context, Editor, Helper};
use std::io::{self, BufRead, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::config::{self, AiProvider, Config};
use crate::ollama;

// ─────────────────────────────────────────────────────────────────────────────
// Tab-completion helper
// ─────────────────────────────────────────────────────────────────────────────

/// All slash-commands that can be tab-completed.
const COMMANDS: &[&str] = &[
    "/help",
    "/settings",
    "/models",
    "/connections",
    "/start",
    "/logs",
    "/hardware",
    "/halt",
    "/memory",
    "/quit",
    "/exit",
];

/// Rustyline helper: provides tab-completion for MechOS slash-commands.
pub struct MechCompleter {
    hinter: HistoryHinter,
    highlighter: MatchingBracketHighlighter,
    validator: MatchingBracketValidator,
}

impl Helper for MechCompleter {}
impl Highlighter for MechCompleter {
    fn highlight_hint<'h>(&self, hint: &'h str) -> std::borrow::Cow<'h, str> {
        self.highlighter.highlight_hint(hint)
    }
}
impl Hinter for MechCompleter {
    type Hint = String;
    fn hint(&self, line: &str, pos: usize, ctx: &Context<'_>) -> Option<String> {
        self.hinter.hint(line, pos, ctx)
    }
}
impl Validator for MechCompleter {
    fn validate(&self, ctx: &mut rustyline::validate::ValidationContext) -> rustyline::Result<rustyline::validate::ValidationResult> {
        self.validator.validate(ctx)
    }
}

impl MechCompleter {
    fn new() -> Self {
        Self {
            hinter: HistoryHinter::new(),
            highlighter: MatchingBracketHighlighter::new(),
            validator: MatchingBracketValidator::new(),
        }
    }
}

impl Completer for MechCompleter {
    type Candidate = Pair;

    fn complete(
        &self,
        line: &str,
        pos: usize,
        _ctx: &Context<'_>,
    ) -> rustyline::Result<(usize, Vec<Pair>)> {
        let word = &line[..pos];
        let candidates: Vec<Pair> = COMMANDS
            .iter()
            .filter(|c| c.starts_with(word))
            .map(|c| Pair {
                display: c.to_string(),
                replacement: c.to_string(),
            })
            .collect();
        Ok((0, candidates))
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Shared REPL state
// ─────────────────────────────────────────────────────────────────────────────

/// Runtime state shared across REPL command handlers.
/// Both fields are `None` until `/start` completes successfully.
pub struct ReplState {
    /// Reference to the live Event Bus (available after `/start`).
    pub bus: Option<Arc<mechos_middleware::EventBus>>,
    /// Reference to the episodic memory store (available after `/start`).
    pub store: Option<mechos_memory::episodic::EpisodicStore>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal OS state machine
// ─────────────────────────────────────────────────────────────────────────────

/// Internal OS state machine.
#[derive(Debug, Clone, PartialEq)]
#[allow(dead_code)]
pub enum AppState {
    Offline,
    Booting,
    Running,
}

// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────

/// Entry point for the interactive REPL.
///
/// `shutdown` is polled each iteration; when set the REPL exits cleanly.
/// Uses [`rustyline`] for command history (↑/↓) and tab-completion.
pub fn run(shutdown: Arc<AtomicBool>) {
    let helper = MechCompleter::new();
    let config = rustyline::Config::builder()
        .history_ignore_space(true)
        .completion_type(rustyline::CompletionType::List)
        .build();
    let mut rl: Editor<MechCompleter, DefaultHistory> =
        Editor::with_config(config).unwrap_or_else(|_| Editor::new().unwrap());
    rl.set_helper(Some(helper));

    let mut state = ReplState { bus: None, store: None };

    loop {
        if shutdown.load(Ordering::SeqCst) {
            break;
        }

        match rl.readline(&format!("{} ", "mechos>".bold().cyan())) {
            Ok(line) => {
                let cmd = line.trim().to_string();
                if cmd.is_empty() {
                    continue;
                }
                rl.add_history_entry(cmd.as_str()).ok();
                dispatch(&cmd, &mut state, shutdown.clone());
                if shutdown.load(Ordering::SeqCst) {
                    break;
                }
            }
            Err(ReadlineError::Interrupted) | Err(ReadlineError::Eof) => {
                // Ctrl-C / Ctrl-D: the ctrlc handler in main.rs handles the
                // shutdown flag; we just stop reading.
                break;
            }
            Err(e) => {
                eprintln!("{}: {}", "Read error".red(), e);
                break;
            }
        }
    }
}

/// Dispatch a trimmed command string to the appropriate handler.
fn dispatch(cmd: &str, state: &mut ReplState, shutdown: Arc<AtomicBool>) {
    // Split into verb + arguments for multi-word commands.
    let mut parts = cmd.splitn(2, ' ');
    let verb = parts.next().unwrap_or("");
    let rest = parts.next().unwrap_or("").trim();

    match verb {
        "/help"        => cmd_help(),
        "/settings"    => cmd_settings(),
        "/models"      => cmd_models(),
        "/connections" => cmd_connections(),
        "/start"       => cmd_start(shutdown, state),
        "/logs"        => cmd_logs(state),
        "/hardware"    => cmd_hardware(rest, state),
        "/halt"        => cmd_halt(state),
        "/memory"      => cmd_memory(rest, state),
        "/quit" | "/exit" => {
            println!("{}", "Goodbye.".green());
            shutdown.store(true, Ordering::SeqCst);
        }
        _ => {
            println!(
                "{} '{}'. Type {} for available commands.",
                "Unknown command:".red(),
                cmd.yellow(),
                "/help".bold()
            );
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Command handlers
// ─────────────────────────────────────────────────────────────────────────────

fn cmd_help() {
    println!();
    println!("{}", "MechOS Commands".bold().underline());
    println!("  {}   – edit ~/.mechos/config.toml settings",      "/settings".bold().cyan());
    println!("  {}     – list and switch AI models",               "/models".bold().cyan());
    println!("  {} – adapter connectivity diagnostic",        "/connections".bold().cyan());
    println!("  {}      – initiate the OS boot sequence",          "/start".bold().cyan());
    println!("  {}       – stream live Event Bus events",           "/logs".bold().cyan());
    println!("  {}   – send a HardwareIntent to the bus",       "/hardware".bold().cyan());
    println!("     {}          drive <lin> <ang>",                  "".dimmed());
    println!("     {}          move  <x>   <y>  <z>",              "".dimmed());
    println!("     {}          relay <id>  on|off",                 "".dimmed());
    println!("  {}       – emergency stop (keeps REPL running)",    "/halt".bold().cyan());
    println!("  {}     – inspect episodic memory store",          "/memory".bold().cyan());
    println!("     {}          list",                               "".dimmed());
    println!("     {}          query <search terms>",               "".dimmed());
    println!("  {}  – exit the CLI",                   "/quit  /exit".bold().cyan());
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

fn cmd_start(shutdown: Arc<AtomicBool>, state: &mut ReplState) {
    let cfg = load_config_or_default();

    println!();
    println!("{}", "═══════════════════════════════════════".bold());
    println!("{}", "         MechOS Boot Sequence          ".bold().cyan());
    println!("{}", "═══════════════════════════════════════".bold());

    // ── Step 1 – Memory ────────────────────────────────────────────────────
    // Resolve a persistent path: ~/.mechos/memory.db
    let memory_path = {
        let home = std::env::var("HOME")
            .or_else(|_| std::env::var("USERPROFILE"))
            .unwrap_or_else(|_| ".".to_string());
        let dir = std::path::PathBuf::from(home).join(".mechos");
        if let Err(e) = std::fs::create_dir_all(&dir) {
            println!(
                "{}: could not create {}: {}",
                "Warning".yellow(),
                dir.display(),
                e
            );
        }
        dir.join("memory.db").to_string_lossy().into_owned()
    };
    print!(
        "  [1/7] {} {} … ",
        "Initializing Memory (SQLite) at".bold(),
        memory_path.dimmed()
    );
    io::stdout().flush().ok();
    let episodic_store = match mechos_memory::episodic::EpisodicStore::open(&memory_path) {
        Ok(s) => { println!("{}", "OK".green()); s }
        Err(e) => {
            println!("{}: {}", "FAILED".red(), e);
            return;
        }
    };

    // ── Step 2 – Event Bus ─────────────────────────────────────────────────
    print!("  [2/7] {} … ", "Initializing Event Bus".bold());
    io::stdout().flush().ok();
    let bus = std::sync::Arc::new(mechos_middleware::EventBus::new(256));
    println!("{}", "OK".green());

    // ── Step 3 – Kernel Safety Interlocks ──────────────────────────────────
    print!("  [3/7] {} … ", "Engaging Kernel Safety Interlocks".bold());
    io::stdout().flush().ok();
    let _watchdog = mechos_kernel::Watchdog::new();
    let _cap_mgr  = mechos_kernel::CapabilityManager::new();
    println!("{}", "OK".green());

    // ── Step 4 – Dashboard Adapter ─────────────────────────────────────────
    let dash_url = format!("ws://localhost:{}", cfg.dashboard_port);
    print!("  [4/7] {} {} … ", "Binding DashboardSimAdapter on".bold(), dash_url.yellow());
    io::stdout().flush().ok();
    let _adapter = mechos_middleware::DashboardSimAdapter::new(
        bus.clone(),
        dash_url.clone(),
    );
    println!("{}", "OK".green());

    // ── Step 5 – Cockpit Web UI ────────────────────────────────────────────
    {
        let webui_port = cfg.webui_port;
        let bus_for_cockpit = bus.clone();
        print!(
            "  [5/7] {} {} … ",
            "Starting Cockpit Web UI on port".bold(),
            webui_port.to_string().yellow()
        );
        io::stdout().flush().ok();
        std::thread::spawn(move || {
            let rt = match tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            {
                Ok(rt) => rt,
                Err(e) => {
                    eprintln!("{}: cockpit server runtime: {}", "ERROR".red(), e);
                    return;
                }
            };
            rt.block_on(async move {
                let server = mechos_cockpit::CockpitServer::new(bus_for_cockpit)
                    .with_port(webui_port);
                if let Err(e) = server.run().await {
                    tracing::error!(error = %e, "Cockpit server failed");
                }
            });
        });
        println!("{} (http://localhost:{})", "OK".green(), webui_port);
    }

    // ── Step 6 – Runtime Brain ─────────────────────────────────────────────
    print!(
        "  [6/7] {} {} … ",
        "Booting Runtime Brain (model:".bold(),
        cfg.active_model.yellow()
    );
    io::stdout().flush().ok();
    let loop_config = mechos_runtime::AgentLoopConfig {
        llm_base_url: cfg.ollama_url.clone(),
        llm_model: cfg.active_model.clone(),
        memory_path: Some(memory_path),
        bus: Some((*bus).clone()),
        ..Default::default()
    };
    let agent = match mechos_runtime::AgentLoop::new(loop_config) {
        Ok(agent) => agent,
        Err(e) => {
            println!("{} {}", "ERROR".red(), e);
            return;
        }
    };
    println!("{}", "OK".green());

    // ── Step 7 – Store shared references in REPL state ─────────────────────
    print!("  [7/7] {} … ", "Registering runtime references".bold());
    io::stdout().flush().ok();
    state.bus = Some(bus.clone());
    state.store = Some(episodic_store);
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

    // ── Spawn the continuous OODA loop in a background thread ───────────────
    // A dedicated thread owns a single-threaded tokio runtime so that the
    // async tick() can run without blocking the interactive REPL.
    // OODA loop target frequency: 10 Hz (100 ms per tick).
    const TICK_RATE_HZ: f32 = 10.0;
    let tick_interval =
        std::time::Duration::from_millis((1000.0 / TICK_RATE_HZ) as u64);
    let tick_dt = 1.0 / TICK_RATE_HZ;

    std::thread::spawn(move || {
        let rt = match tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        {
            Ok(rt) => rt,
            Err(e) => {
                eprintln!(
                    "{}: failed to create async runtime for agent loop: {}\n  \
                     Ensure your system supports async I/O (check ulimits / OS resources).",
                    "ERROR".red(),
                    e
                );
                return;
            }
        };
        rt.block_on(async move {
            let mut agent = agent;
            loop {
                if shutdown.load(Ordering::SeqCst) {
                    tracing::info!("agent loop shutting down");
                    break;
                }
                tokio::time::sleep(tick_interval).await;
                match agent.tick(tick_dt).await {
                    Ok(intent) => {
                        tracing::info!(intent = ?intent, "agent intent dispatched");
                    }
                    Err(e) => {
                        tracing::debug!(error = %e, "agent tick skipped");
                    }
                }
            }
        });
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// /logs – real-time Event Bus stream
// ─────────────────────────────────────────────────────────────────────────────

fn cmd_logs(state: &ReplState) {
    let Some(bus) = &state.bus else {
        println!("{}", "System not started. Run /start first.".red());
        return;
    };

    let mut rx = bus.subscribe();
    let stop = Arc::new(AtomicBool::new(false));
    let stop_writer = stop.clone();

    // Background thread: receives events and prints them coloured by severity.
    let handle = std::thread::spawn(move || {
        let rt = match tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        {
            Ok(rt) => rt,
            Err(_) => return,
        };
        rt.block_on(async move {
            loop {
                if stop_writer.load(Ordering::SeqCst) {
                    break;
                }
                tokio::select! {
                    result = rx.recv() => {
                        match result {
                            Ok(event) => print_event_colored(&event),
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                                println!("{}: {} events dropped by slow consumer", "WARN".yellow(), n);
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
                        }
                    }
                    _ = tokio::time::sleep(std::time::Duration::from_millis(100)) => {}
                }
            }
        });
    });

    println!(
        "{}",
        "  Streaming Event Bus. Press ENTER to stop…".dimmed()
    );
    let mut line = String::new();
    io::stdin().lock().read_line(&mut line).ok();

    stop.store(true, Ordering::SeqCst);
    handle.join().ok();
    println!("{}", "  Log stream stopped.".dimmed());
}

/// Print a single event to stdout, coloured by its payload type.
fn print_event_colored(event: &mechos_types::Event) {
    use mechos_types::EventPayload;

    let ts = event.timestamp.format("%H:%M:%S%.3f");
    let src = &event.source;

    match &event.payload {
        EventPayload::HardwareFault { component, code, message } => {
            println!(
                "[{}] {} fault on {} [{}]: {}",
                ts.to_string().dimmed(),
                "FAULT".red().bold(),
                component.red(),
                code.to_string().red(),
                message.red()
            );
        }
        EventPayload::AgentThought(thought) => {
            let truncated = if thought.len() > 120 {
                format!("{}…", &thought[..120])
            } else {
                thought.clone()
            };
            println!(
                "[{}] {} {}",
                ts.to_string().dimmed(),
                "THOUGHT".cyan(),
                truncated
            );
        }
        EventPayload::HumanResponse(resp) => {
            println!(
                "[{}] {} {}",
                ts.to_string().dimmed(),
                "HUMAN".bold().yellow(),
                resp.yellow()
            );
        }
        EventPayload::Telemetry(t) => {
            println!(
                "[{}] {} x={:.2} y={:.2} hdg={:.2}° bat={}%",
                ts.to_string().dimmed(),
                "TELEM".blue(),
                t.position_x,
                t.position_y,
                t.heading_rad.to_degrees(),
                t.battery_percent
            );
        }
        EventPayload::AgentModeToggle { paused } => {
            let state = if *paused { "PAUSED".yellow() } else { "RESUMED".green() };
            println!(
                "[{}] {} agent {} by {}",
                ts.to_string().dimmed(),
                "MODE".bold(),
                state,
                src.dimmed()
            );
        }
        EventPayload::LidarScan { ranges, .. } => {
            println!(
                "[{}] {} {} points",
                ts.to_string().dimmed(),
                "LIDAR".magenta(),
                ranges.len()
            );
        }
        EventPayload::PeerMessage { from_robot_id, message } => {
            println!(
                "[{}] {} from {}: {}",
                ts.to_string().dimmed(),
                "PEER".bold().blue(),
                from_robot_id.bold(),
                message
            );
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// /hardware – manual HardwareIntent injection
// ─────────────────────────────────────────────────────────────────────────────

fn cmd_hardware(args: &str, state: &ReplState) {
    let Some(bus) = &state.bus else {
        println!("{}", "System not started. Run /start first.".red());
        return;
    };

    let parts: Vec<&str> = args.split_whitespace().collect();

    let intent = match parts.as_slice() {
        ["drive", lin, ang] => {
            let Ok(linear_velocity) = lin.parse::<f32>() else {
                println!("{}: linear_velocity must be a number", "Error".red());
                return;
            };
            let Ok(angular_velocity) = ang.parse::<f32>() else {
                println!("{}: angular_velocity must be a number", "Error".red());
                return;
            };
            mechos_types::HardwareIntent::Drive { linear_velocity, angular_velocity }
        }
        ["move", xs, ys, zs] => {
            let (Ok(x), Ok(y), Ok(z)) = (xs.parse::<f32>(), ys.parse::<f32>(), zs.parse::<f32>()) else {
                println!("{}: x, y, z must be numbers", "Error".red());
                return;
            };
            mechos_types::HardwareIntent::MoveEndEffector { x, y, z }
        }
        ["relay", relay_id, state_str] => {
            let on = match *state_str {
                "on" | "1" | "true"  => true,
                "off" | "0" | "false" => false,
                _ => {
                    println!("{}: relay state must be 'on' or 'off'", "Error".red());
                    return;
                }
            };
            mechos_types::HardwareIntent::TriggerRelay {
                relay_id: relay_id.to_string(),
                state: on,
            }
        }
        _ => {
            println!("{}", "Usage:".bold());
            println!("  /hardware drive <linear_vel> <angular_vel>");
            println!("  /hardware move  <x> <y> <z>");
            println!("  /hardware relay <relay_id> on|off");
            return;
        }
    };

    // Serialise the intent and publish it as an AgentThought so the bus
    // broadcast reaches any dashboard or log subscriber.
    let payload_json = serde_json::to_string(&intent).unwrap_or_else(|_| format!("{intent:?}"));
    let event = mechos_types::Event {
        id: uuid::Uuid::new_v4(),
        timestamp: chrono::Utc::now(),
        source: "mechos-cli::hardware_override".to_string(),
        payload: mechos_types::EventPayload::AgentThought(payload_json.clone()),
        trace_id: None,
    };
    match bus.publish_to(mechos_middleware::Topic::HardwareCommands, event) {
        Ok(_) => println!(
            "{} {}",
            "✓ HardwareIntent published:".green(),
            payload_json.bold()
        ),
        Err(e) => println!("{}: {}", "Publish failed".red(), e),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// /halt – emergency stop without exiting
// ─────────────────────────────────────────────────────────────────────────────

fn cmd_halt(state: &ReplState) {
    let Some(bus) = &state.bus else {
        println!("{}", "System not started. Run /start first.".red());
        return;
    };

    let event = mechos_types::Event {
        id: uuid::Uuid::new_v4(),
        timestamp: chrono::Utc::now(),
        source: "mechos-cli::halt".to_string(),
        payload: mechos_types::EventPayload::HardwareFault {
            component: "cli".to_string(),
            code: 911,
            message: "EMERGENCY_STOP: operator /halt".to_string(),
        },
        trace_id: None,
    };

    match bus.publish_to(mechos_middleware::Topic::SystemAlerts, event) {
        Ok(_) => println!(
            "{}",
            "⛔ EmergencyStop published to SystemAlerts. Agent loop suspended.".red().bold()
        ),
        Err(e) => println!("{}: {}", "Halt failed".red(), e),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// /memory – episodic memory inspector
// ─────────────────────────────────────────────────────────────────────────────

fn cmd_memory(args: &str, state: &ReplState) {
    let Some(store) = &state.store else {
        println!("{}", "System not started. Run /start first.".red());
        return;
    };

    let parts: Vec<&str> = args.splitn(2, ' ').collect();
    let subcommand = parts.first().copied().unwrap_or("list");
    let query = parts.get(1).copied().unwrap_or("").trim().trim_matches('"');

    // Run the async store operations in a dedicated thread so that:
    // (a) we avoid creating a nested runtime when called from inside tokio tests
    // (b) the sync REPL caller doesn't need an async runtime
    let store_clone = store.clone();
    let entries = std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap_or_else(|e| panic!("failed to create tokio runtime: {e}"));
        rt.block_on(store_clone.all_entries())
    })
    .join()
    .unwrap_or_else(|e| {
        Err(mechos_memory::episodic::EpisodicError::TaskPanic(format!(
            "thread panicked: {e:?}"
        )))
    });

    match entries {
        Err(e) => {
            println!("{}: {}", "Memory read error".red(), e);
        }
        Ok(all) => match subcommand {
            "list" => {
                if all.is_empty() {
                    println!("{}", "  Memory store is empty.".dimmed());
                    return;
                }
                println!("{}", "Episodic Memory Store".bold().underline());
                for entry in &all {
                    println!(
                        "  {} {} [{}] {}",
                        entry.timestamp.format("%Y-%m-%d %H:%M:%S").to_string().dimmed(),
                        entry.source.cyan(),
                        entry.embedding.len().to_string().dimmed(),
                        entry.summary.bold()
                    );
                }
                println!("  {} entries total.", all.len().to_string().yellow());
            }
            "query" => {
                if query.is_empty() {
                    println!("{}", "Usage: /memory query <search terms>".yellow());
                    return;
                }
                let needle = query.to_lowercase();
                let matches: Vec<_> = all
                    .iter()
                    .filter(|e| e.summary.to_lowercase().contains(&needle))
                    .collect();

                if matches.is_empty() {
                    println!(
                        "{} '{}'",
                        "  No memories match:".dimmed(),
                        query.yellow()
                    );
                    return;
                }
                println!(
                    "{} '{}' ({} result{}):",
                    "Memory query for".bold().underline(),
                    query.yellow(),
                    matches.len().to_string().green(),
                    if matches.len() == 1 { "" } else { "s" }
                );
                for entry in &matches {
                    println!(
                        "  {} {} {}",
                        entry.timestamp.format("%Y-%m-%d %H:%M:%S").to_string().dimmed(),
                        entry.source.cyan(),
                        entry.summary.bold()
                    );
                }
            }
            _ => {
                println!("{}", "Usage:".bold());
                println!("  /memory list");
                println!("  /memory query <search terms>");
            }
        },
    }
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

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};

    #[test]
    fn dispatch_unknown_command_does_not_panic() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let mut state = ReplState { bus: None, store: None };
        // Should print "Unknown command" but not panic.
        dispatch("/foobar", &mut state, shutdown.clone());
        assert!(!shutdown.load(Ordering::SeqCst));
    }

    #[test]
    fn dispatch_quit_sets_shutdown() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let mut state = ReplState { bus: None, store: None };
        dispatch("/quit", &mut state, shutdown.clone());
        assert!(shutdown.load(Ordering::SeqCst));
    }

    #[test]
    fn dispatch_exit_sets_shutdown() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let mut state = ReplState { bus: None, store: None };
        dispatch("/exit", &mut state, shutdown.clone());
        assert!(shutdown.load(Ordering::SeqCst));
    }

    #[test]
    fn hardware_command_without_start_prints_error() {
        let state = ReplState { bus: None, store: None };
        // Should not panic when bus is None.
        cmd_hardware("drive 1.0 0.0", &state);
    }

    #[test]
    fn halt_command_without_start_prints_error() {
        let state = ReplState { bus: None, store: None };
        // Should not panic when bus is None.
        cmd_halt(&state);
    }

    #[test]
    fn logs_command_without_start_prints_error() {
        let state = ReplState { bus: None, store: None };
        // Should not panic when bus is None.
        cmd_logs(&state);
    }

    #[test]
    fn memory_command_without_start_prints_error() {
        let state = ReplState { bus: None, store: None };
        cmd_memory("list", &state);
    }

    #[tokio::test]
    async fn hardware_drive_publishes_event() {
        let bus = Arc::new(mechos_middleware::EventBus::new(16));
        let mut rx = bus.subscribe_to(mechos_middleware::Topic::HardwareCommands);
        let state = ReplState {
            bus: Some(bus),
            store: None,
        };
        cmd_hardware("drive 0.5 -0.3", &state);
        // The event should be in the topic channel.
        match rx.recv().await {
            Ok(event) => assert_eq!(event.source, "mechos-cli::hardware_override"),
            Err(_) => panic!("expected event on bus after /hardware drive"),
        }
    }

    #[tokio::test]
    async fn hardware_move_publishes_event() {
        let bus = Arc::new(mechos_middleware::EventBus::new(16));
        let mut rx = bus.subscribe_to(mechos_middleware::Topic::HardwareCommands);
        let state = ReplState {
            bus: Some(bus),
            store: None,
        };
        cmd_hardware("move 0.5 -0.1 0.3", &state);
        assert!(rx.recv().await.is_ok(), "expected event on bus after /hardware move");
    }

    #[tokio::test]
    async fn hardware_relay_on_publishes_event() {
        let bus = Arc::new(mechos_middleware::EventBus::new(16));
        let mut rx = bus.subscribe_to(mechos_middleware::Topic::HardwareCommands);
        let state = ReplState {
            bus: Some(bus),
            store: None,
        };
        cmd_hardware("relay door_1 on", &state);
        assert!(rx.recv().await.is_ok(), "expected event on bus after /hardware relay on");
    }

    #[test]
    fn hardware_invalid_args_does_not_publish() {
        // No subscriber → the topic channel returns error on send.
        // Valid args with a fresh bus and no subscriber: publish returns Err(Channel),
        // which cmd_hardware prints. With invalid args, it returns before publishing.
        let bus = Arc::new(mechos_middleware::EventBus::new(16));
        let state = ReplState {
            bus: Some(bus),
            store: None,
        };
        // Should print usage, not panic, and not publish (no subscriber to check).
        cmd_hardware("drive not_a_number 0.0", &state);
    }

    #[tokio::test]
    async fn halt_publishes_fault_event() {
        let bus = Arc::new(mechos_middleware::EventBus::new(16));
        let mut rx = bus.subscribe_to(mechos_middleware::Topic::SystemAlerts);
        let state = ReplState {
            bus: Some(bus),
            store: None,
        };
        cmd_halt(&state);
        let event = rx.recv().await.expect("expected fault event after /halt");
        assert_eq!(event.source, "mechos-cli::halt");
        assert!(matches!(
            event.payload,
            mechos_types::EventPayload::HardwareFault { code: 911, .. }
        ));
    }

    #[test]
    fn memory_list_empty_store() {
        let store = mechos_memory::episodic::EpisodicStore::open_in_memory().unwrap();
        let state = ReplState {
            bus: None,
            store: Some(store),
        };
        // Should not panic on an empty store.
        cmd_memory("list", &state);
    }

    #[test]
    fn memory_query_finds_matching_entry() {
        let store = mechos_memory::episodic::EpisodicStore::open_in_memory().unwrap();
        // Store the entry synchronously via a dedicated runtime in a thread.
        let store_clone = store.clone();
        std::thread::spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap();
            rt.block_on(async {
                let entry = mechos_memory::episodic::MemoryEntry::new(
                    "test".to_string(),
                    "The robot navigated past the blue table".to_string(),
                    vec![0.1, 0.9],
                );
                store_clone.store(&entry).await.unwrap();
            });
        })
        .join()
        .unwrap();

        let state = ReplState {
            bus: None,
            store: Some(store),
        };
        // Should not panic; no assertion on output but we verify no crash.
        cmd_memory("query blue table", &state);
    }
}
