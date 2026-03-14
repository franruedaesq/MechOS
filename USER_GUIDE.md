# MechOS User Guide: Getting Started

Welcome to the **MechOS** User Guide! This document is your comprehensive operational manual for setting up, running, and interacting with the MechOS stack. Whether you are a developer looking to extend the codebase or a user wanting to experiment with an autonomous AI robot brain, this guide is for you.

---

## 1. Prerequisites & Installation

### A. Install Rust

MechOS is built in Rust. You need the Rust toolchain (compiler and package manager) installed.

**Linux / macOS:**
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
```

**Windows:**
Visit [rust-lang.org/tools/install](https://www.rust-lang.org/tools/install) and download `rustup-init.exe`.

### B. Install Ollama (The AI Brain)

MechOS relies on a local Large Language Model (LLM) server to function. We use **Ollama** because it is lightweight, cross-platform, and free.

1.  **Download & Install:**
    Go to [ollama.com](https://ollama.com) and follow the instructions for your OS.

2.  **Pull a Model:**
    Open a terminal and download a model. We recommend `llama3` for its balance of speed and reasoning capability.
    ```bash
    ollama pull llama3
    ```

3.  **Start the Server:**
    Usually, Ollama runs in the background. If not, start it manually:
    ```bash
    ollama serve
    ```
    *Keep this terminal window open!*

---

## 2. Launching MechOS (CLI)

The Command Line Interface (CLI) is the main entry point for interacting with the system.

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/your-username/mechos.git
    cd mechos
    ```

2.  **Run the CLI:**
    ```bash
    cargo run
    ```

3.  **First-Run Configuration:**
    If this is your first time running MechOS, the wizard will guide you:
    *   **AI Provider:** Select `1` for Local AI (Ollama).
    *   **Ports:** Accept the defaults (Web UI: 8080, Dashboard: 9090).

---

## 3. The MechOS REPL

Once launched, you will see the `mechos>` prompt. This is your command center.

### Basic Commands:

*   **`/help`**: Lists all available commands.
*   **`/connections`**: Runs a quick diagnostic to check if:
    *   The Ollama server is reachable.
    *   The Dashboard websocket server is ready.
*   **`/models`**: Shows which AI models are installed on your machine and lets you switch between them (e.g., from `llama3` to `mistral`).

### Starting the Brain:

To boot the Operating System (initialize memory, start the event bus, and launch the OODA loop):

```bash
mechos> /start
```

You will see a boot sequence:
```text
  [1/6] Initializing Memory (SQLite) … OK
  [2/6] Initializing Event Bus … OK
  [3/6] Engaging Kernel Safety Interlocks … OK
  ...
  ✓ MechOS is RUNNING. Type /quit to stop.
```

---

## 4. The Web Cockpit (UI)

While the CLI handles system commands, the **Web Cockpit** is where you visualize what the robot is thinking and doing.

1.  **Open your browser** (Chrome/Firefox recommended).
2.  **Navigate to:** `http://localhost:8080`

### Interface Overview:

*   **3D Viewport:** A real-time visualization of the robot's position and the simulated world.
*   **Log Console:** A stream of "thoughts" from the AI. You can watch it reason:
    > *"I see an obstacle ahead. Checking memory for similar situations. Deciding to turn left."*
*   **Manual Override:**
    *   Click on the viewport to focus.
    *   Use **W / A / S / D** keys to drive the robot manually.
    *   *Note:* When you take manual control, the AI is temporarily suspended for safety.

---

## 5. Experimenting with the Robot Brain

Here are some things to try:

### A. The "Human-in-the-Loop" (HITL) Demo

1.  In the Cockpit, observe the robot navigating autonomously.
2.  If the AI encounters a situation it is unsure about (e.g., "I see a red button but don't know if I should push it"), it will issue an `AskHuman` intent.
3.  A popup will appear on the Dashboard: *"Should I push the red button?"*
4.  Type your answer: *"Yes, proceed carefully."*
5.  Watch the Log Console: The AI receives your guidance, updates its memory, and executes the action.

### B. Memory Recall

1.  Let the robot navigate for a while. It is building up **Episodic Memory**.
2.  Stop the system (`/quit` in CLI) and restart it.
3.  The robot will "remember" previous interactions because they are stored in the local SQLite database. It uses this context to make better decisions in the future.

---

## 6. Troubleshooting

**Q: "No Ollama instance detected"**
*   **A:** Ensure you ran `ollama serve` in a separate terminal. Check if `http://localhost:11434` is accessible in your browser.

**Q: "The robot isn't moving."**
*   **A:** Check the Log Console in the Web UI. Is the AI stuck in a loop? Is the Kernel rejecting unsafe commands? Try using manual override (WASD) to unstick it.

**Q: "How do I clear the memory?"**
*   **A:** Delete the `episodic_memory.sqlite` file in the project root directory. The system will create a fresh one on the next boot.
