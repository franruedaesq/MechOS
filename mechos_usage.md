# Using MechOS: Runtime and Interaction

## The Interactive REPL

When you start MechOS via the `mechos-cli`, you are dropped into an interactive Read-Eval-Print Loop (REPL). This acts as the command center for the OS. The CLI intercepts standard terminal inputs and intercepts `Ctrl-C` to send an `EmergencyStop` intent to halt the system safely.

The REPL supports several slash-commands for management and diagnostics:

- `/help` - Shows a list of available commands.
- `/settings` - Opens an interactive editor for `~/.mechos/config.toml` where you can adjust your dashboard port, web UI port, AI provider, and active model.
- `/models` - Lists available AI models (e.g., probes Ollama for downloaded models) and lets you switch the active model on the fly.
- `/connections` - Runs a diagnostic check on adapter connectivity, verifying the DashboardSimAdapter, Ollama connectivity, and hardware drivers (e.g., Physical LiDAR).
- `/start` - Initiates the OS boot sequence, bringing the system online.
- `/quit` or `/exit` - Gracefully exits the CLI.

## The `/start` Boot Sequence

Executing `/start` triggers a sequential boot process that activates the different layers of the Operating System:

1. **Initializing Memory**: MechOS opens its SQLite-based persistent memory store at `~/.mechos/memory.db`.
2. **Initializing Event Bus**: The headless, typed publish/subscribe system for inter-crate communication comes online.
3. **Engaging Kernel Safety Interlocks**: The `Watchdog` and `CapabilityManager` activate, establishing the physical boundaries and permissions.
4. **Binding DashboardSimAdapter**: The system binds a WebSocket server on the configured Dashboard port (default `9090`) to facilitate external connections (ROS 2/simulation).
5. **Web UI Initialization**: The system configures the HTTP port for the user-facing web interface (default `8080`).
6. **Booting Runtime Brain**: Finally, the OODA loop orchestrator (`AgentLoop`) starts up using the active AI model.

## What Happens Next?

Once the boot sequence completes, MechOS reports that it is `RUNNING` and drops you back to the REPL prompt. Under the hood, a background thread has spawned the continuous Observe-Orient-Decide-Act (OODA) loop, running at a target frequency of 10 Hz.

While the system is running:

1. **The Brain is Active**: The runtime loop is constantly pulling spatial data, consulting episodic memory, deciding on actions, and dispatching `HardwareIntent` commands to the `EventBus`.
2. **Hardware/Simulation Execution**: External systems, such as a React/Three.js dashboard or actual ROS 2 robots, can connect to the Dashboard adapter (default `ws://localhost:9090`) to execute intents and provide sensor telemetry.
3. **Web UI Interaction**: A user-facing Web UI is available over HTTP (default `http://localhost:8080`) to provide high-level visibility, logging, and potentially HITL (Human-in-the-Loop) interactions like answering `AskHuman` intents.
4. **Interactive Overrides**: You remain in the CLI, able to issue further commands or gracefully shut the system down.

## Accessing the Web UI

MechOS runs a web UI, accessible after booting the system. Based on the system's configuration:

- **The Main Web UI**: Accessed via HTTP, by default at `http://localhost:8080`. This provides the primary user dashboard for the OS.
- **The Dashboard ROS Bridge**: Accessed via WebSocket, by default at `ws://localhost:9090`. This is the connection point for ROS 2 bridges or digital twin simulations.

You can modify these ports using the `/settings` command or by setting the `MECHOS_WEBUI_PORT` and `MECHOS_DASHBOARD_PORT` environment variables prior to launching.