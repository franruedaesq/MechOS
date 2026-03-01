# MechOS: Project Readiness and Configuration Guide

## Introduction
MechOS is an offline-first Agent Operating System designed for physical robotics. It bridges the gap between high-level, non-deterministic cognitive reasoning (using large language models via Ollama, OpenAI, or Anthropic) and low-level, deterministic hardware execution (ROS 2 and physical actuators). It emphasizes a strict separation between the AI "brain" and the hardware "muscle", utilizing a central Kernel to enforce safety and manage permissions.

## Readiness: Is the project ready to use?
Based on the current state of the codebase, MechOS is in an **early/alpha stage** but is **ready for testing and development**.
* **Core Components Functional:** The foundational crates (`mechos-types`, `mechos-kernel`, `mechos-memory`, `mechos-runtime`, `mechos-middleware`) compile and pass tests. The `mechos-cli` provides an interactive REPL for launching and interacting with the OS.
* **Integrations Implemented:** It includes working integrations for local AI via Ollama, as well as cloud AI (OpenAI/Anthropic). It also has working middleware adapters for simulated digital twins (`DashboardSimAdapter`) and physical ROS 2 robots (`Ros2Adapter`, `Ros2Bridge`).
* **Limitations:** The project is likely missing some advanced hardware drivers and robust error handling for unexpected edge cases in physical environments. The ROS 2 integration expects specific topic structures (`/cmd_vel`, `/scan`, `/odom`, etc.) that may need to be adapted to your specific robot platform.

## Step-by-Step Configuration on MacBook

To run MechOS on a macOS (MacBook) system, follow these steps:

### 1. Install Prerequisites
You need Rust installed. If you don't have it, install it via `rustup`:
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```
Make sure you are using the stable toolchain.

### 2. Configure AI Provider
MechOS supports local (Ollama) and cloud (OpenAI, Anthropic) AI models.

#### Option A: Local AI via Ollama (Recommended for offline-first)
1. Download and install Ollama for macOS from [ollama.com](https://ollama.com/).
2. Start the Ollama server:
   ```bash
   ollama serve
   ```
3. In a separate terminal, pull a model (e.g., `llama3`):
   ```bash
   ollama pull llama3
   ```

#### Option B: Cloud AI (OpenAI / Anthropic)
If you prefer to use cloud APIs, you need to set the corresponding environment variables. MechOS will read these when it starts.
* **OpenAI:**
  ```bash
  export MECHOS_OPENAI_API_KEY="sk-your-openai-api-key"
  ```
* **Anthropic:**
  ```bash
  export MECHOS_ANTHROPIC_API_KEY="sk-ant-your-anthropic-api-key"
  ```

### 3. Build and Run MechOS
1. Clone the MechOS repository and navigate into it.
2. Build the project:
   ```bash
   cargo build
   ```
3. Run the CLI:
   ```bash
   cargo run --bin mechos
   ```
   *On the first run, the First-Run Wizard will appear. Follow the prompts to select your AI provider and configure the ports (default Dashboard port is 9090, Web UI port is 8080).*

## What to Try
Once inside the interactive `mechos>` REPL, you can try the following commands:
* `/help`: Display all available commands.
* `/settings`: View current configuration (AI provider, active model, ports).
* `/models`: If using Ollama, list downloaded local models and switch between them.
* `/connections`: View diagnostic information (Ollama status, Dashboard connection status).
* `/start`: Boot the MechOS Runtime Brain. This initializes the SQLite memory, Event Bus, Kernel safety interlocks, Dashboard adapter, and starts the continuous OODA (Observe-Orient-Decide-Act) loop.
* After running `/start`, the background loop will be running. Press `Ctrl-C` to initiate a graceful shutdown and send an `EmergencyStop` intent.

## Connecting to ROS 2 or a Digital Twin

MechOS abstracts the physical execution through the `mechos-middleware` crate. It currently provides two primary adapters:

### Connecting to a Simulated Digital Twin
By default, when you run `/start` in the CLI, MechOS initializes the `DashboardSimAdapter`.
* This adapter binds to a local WebSocket port (default `9090`) to simulate the `rosbridge_server` protocol.
* It expects a digital twin (e.g., a React/Three.js web dashboard) to connect to `ws://localhost:9090`.
* The AI's generated intents (like `Drive` or `MoveEndEffector`) are translated into JSON messages on topics like `/sim/cmd_vel` or `/sim/end_effector` and pushed over the WebSocket.
* You can simulate LiDAR/sensor input by sending JSON telemetry messages back over the WebSocket to topics like `/sim_scan`.

### Connecting to a Physical ROS 2 Robot
To connect MechOS to a real robot running ROS 2:
1. Ensure your robot is running `rosbridge_suite` to expose ROS 2 topics over WebSockets.
2. The `mechos-middleware` crate contains `Ros2Bridge` and `Ros2Adapter`. The `Ros2Bridge` is responsible for translating the DDS robotics traffic into lightweight JSON over WebSockets.
3. The `Ros2Adapter` translates abstract `HardwareIntent` commands into specific ROS 2 messages:
   * `Drive` intents are published to `mechos-middleware::ros2/cmd_vel`.
   * `MoveEndEffector` intents are published to `mechos-middleware::ros2/joint_states`.
4. Currently, the CLI defaults to `DashboardSimAdapter`. To use `Ros2Adapter`, the system initialization code (e.g., in `crates/mechos-cli/src/repl.rs`) needs to be modified to instantiate `Ros2Adapter` and `Ros2Bridge` instead, pointing them to the robot's `rosbridge` WebSocket URL.
