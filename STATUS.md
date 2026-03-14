# MechOS: Current Project Status

This document provides a clear snapshot of where the project stands today. It answers the fundamental questions: *Does it run? What do I need? What can it actually do?*

---

## 🚦 Does It Run?

**Yes.**

The core system is functional. The CLI, the Event Bus, the Mock Hardware Layer, and the LLM Runtime (OODA Loop) are all implemented and working together.

*   **Build Status:** Compiles successfully on stable Rust.
*   **Runtime Status:** Boots up, connects to local Ollama, and enters a REPL.
*   **Simulation Status:** Can drive a virtual robot in a web-based dashboard (Cockpit).
*   **Physical Robot Status:** Requires hardware drivers (currently using mocks).

---

## 🛠️ What Do I Need?

To run MechOS, you need the following prerequisites:

1.  **Rust Toolchain:**
    *   Version: Stable (1.75+ recommended)
    *   Install: `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
2.  **Ollama (Local LLM Server):**
    *   This is the "Brain" of the system.
    *   Install: [ollama.com](https://ollama.com)
    *   **Required Model:** You must pull a model before starting. We recommend `llama3` or `mistral`.
        ```bash
        ollama pull llama3
        ```
3.  **Operating System:**
    *   Linux (Ubuntu 22.04+) or macOS (Apple Silicon recommended for LLM speed).
    *   Windows (WSL2) is supported but experimental.

---

## 🚀 How To Run It

The entry point is the `mechos-cli` binary.

1.  **Start Ollama:**
    Open a terminal and ensure the server is running:
    ```bash
    ollama serve
    ```

2.  **Run MechOS:**
    In the project root:
    ```bash
    cargo run
    ```
    *Note: The first run will compile all dependencies, which may take a few minutes.*

3.  **First-Run Wizard:**
    If no config is found, the CLI will ask you to:
    *   Select your AI Provider (choose "Local AI via Ollama").
    *   Confirm ports (default 8080 for Web UI, 9090 for Dashboard).

---

## 🎮 What To Expect

Once running, you will see a banner and drop into the **MechOS REPL**:

```text
   __  ___        __   ____  _____
  /  |/  /__ ___ / /  / __ \/ ___/
 / /|_/ / -_) __/ _ \/ /_/ /\__ \
/_/  /_/\__/\__/_//_/\____/____/

  MechOS v0.1.0
  Autonomous Robot Operating System
```

### 1. Interactive CLI Commands

You can type slash-commands to control the system:

*   `/start` : **Boots the OS.** Connects to the event bus, initializes memory, and starts the OODA loop.
*   `/models`: Lists available AI models from Ollama and lets you switch them.
*   `/connections`: Checks if Ollama and the Dashboard are reachable.
*   `/settings`: Change ports or AI provider.

### 2. The Web Cockpit (UI)

After running `/start`, open your browser to:
**`http://localhost:8080`**

*   **Visualizer:** See a 3D simulation of the robot.
*   **Controls:** Manual override (WASD keys).
*   **Logs:** Real-time stream of the LLM's "thoughts" and system events.

### 3. Capabilities

**What can it actually do right now?**

*   **Think:** It runs an OODA loop. It observes a (simulated) world, retrieves memories, and asks the LLM "What should I do?"
*   **Drive (Simulated):** The LLM can output intents like `Drive { speed: 0.5 }`. The system validates this and moves the virtual robot.
*   **Remember:** It stores interaction summaries in a local vector database (SQLite) and recalls them later.
*   **Ask for Help (HITL):** If the LLM is confused, it can issue an `AskHuman` intent, which pops up an alert on the Dashboard for you to answer.

**What can it NOT do yet?**

*   It does not control *physical* hardware out of the box (you need to write a `mechos-hal` driver for your specific robot platform).
*   It does not have advanced computer vision (it relies on mock "semantic descriptions" of the scene).
