# MechOS Features and Improvements

## Current Capabilities

MechOS currently supports the following capabilities related to the user interface, CLI, and system features:

### System & Core Features
- **Offline-First AI:** Runs local Large Language Models (LLMs) via Ollama, ensuring privacy and offline functionality.
- **Strict Hardware Abstraction:** Completely decoupled logic where the LLM only outputs high-level "Intents", and a Universal Integration Layer translates them to low-level math (e.g., Inverse Kinematics) and motor signals.
- **Safety Kernel:** A central Kernel enforces safety policies and capability permissions. It prevents the AI from violating physical rules or executing dangerous commands.
- **OODA Loop Execution:** Continuous "Observe-Orient-Decide-Act" loop where the robot continuously evaluates its sensors and makes decisions.
- **Structured LLM Output:** Forces the LLM to output strictly typed JSON that maps directly to Rust structs using JSON Schemas.
- **Loop Guard:** Detects and intervenes if the LLM gets stuck in repetitive hallucinations.
- **Universal ROS 2 Bridge:** Middleware translation layer converting complex robotics traffic to lightweight JSON.
- **Spatial Reasoning Engine:** Uses Octrees to partition 3D space for fast collision detection.
- **Episodic Memory Store:** Uses local vector databases (SQLite) to store experiences and recall semantically relevant memories via cosine similarity.

### CLI Features
- **First-Run Setup Wizard:** Automatically triggers when the configuration is missing, guiding the user to select their AI provider and set network ports.
- **Interactive REPL (Read-Eval-Print Loop):** Provides a continuous command prompt for interacting with the OS.
- **Slash Commands:**
  - `/settings`: Interactively edit configuration settings (ports, AI provider).
  - `/models`: List and switch the active AI model (e.g., "llama3").
  - `/connections`: Run adapter connectivity diagnostics (checks Dashboard and Ollama).
  - `/start`: Initiate the OS boot sequence (Memory, Event Bus, Kernel, Adapters, Runtime).
  - `/help`: Display a list of available commands.
  - `/quit` or `/exit`: Gracefully stop the system.
- **Emergency Stop:** Intercepts `Ctrl-C` to publish an Emergency Stop signal to immediately halt the hardware.
- **Dynamic Configuration:** Reads from `~/.mechos/config.toml` but supports overriding via environment variables (e.g., `MECHOS_OLLAMA_URL`, `MECHOS_MODEL`).

---

## Proposed Improvements

### CLI Enhancements
1. **Command Autocomplete & History:** Improve the REPL experience by adding tab-completion for commands and models, plus up/down arrow support for command history.
2. **Real-time Log Streaming:** Introduce a `/logs` command to stream live events from the Event Bus directly in the terminal, colored by severity (e.g., green for actions, red for kernel violations).
3. **Hardware Manual Override:** Add a `/hardware` command to allow developers to manually send specific `HardwareIntent` commands without invoking the LLM (e.g., `/hardware drive 1.0 0.0`).
4. **Immediate Halting:** A dedicated `/halt` command that acts as an explicit emergency stop without exiting the entire REPL (unlike `Ctrl-C`).
5. **Memory Inspector:** A `/memory query "search string"` command to manually query the SQLite vector database and inspect the agent's semantic memories.

### Web UI & Dashboard Features
1. **3D Spatial Visualization:** A web-based 3D view (e.g., using React/Three.js) showing the robot's current perceived Octree, transforms, and planned collision-free paths in real-time.
2. **OODA Loop Status Dashboard:** A visual breakdown of the current tick in the "Observe-Orient-Decide-Act" loop. Show what the sensors are seeing, what memories were retrieved, what the LLM decided, and whether the Kernel approved or rejected it.
3. **Hardware Interlock Monitor:** A live status board of all safety interlocks, capability states, and the heartbeat of system components (Event Bus, Memory Store).
4. **HITL (Human-In-The-Loop) Queue:** A UI section dedicated to `AskHuman` intents, where the AI requests guidance and the user can provide context or make binary decisions.
5. **Config & AI Profile Management:** A web interface to edit the `config.toml` without using the CLI, complete with visual testing of Ollama model availability.
