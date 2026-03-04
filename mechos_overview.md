# MechOS: Overview and Configuration

## What is MechOS?
MechOS is a strict, offline-first Agent Operating System designed specifically for physical robotics. It serves as the critical bridge between high-level, non-deterministic cognitive reasoning (like local LLMs via Ollama, or cloud models via OpenAI/Anthropic) and low-level, deterministic hardware execution (ROS 2 and physical actuators).

Unlike simple scripts or web-based agent frameworks, MechOS is built as a true Operating System. It treats the LLM as the "brain" and the hardware as the "muscle," utilizing a central Kernel to enforce strict physical safety boundaries, manage permissions, and prevent catastrophic hardware failures.

MechOS is built in Rust as a modular Cargo workspace, meaning it is highly performant and compiles into a single native binary for deployment, while keeping modules entirely independent.

## Core Philosophy
**MechOS is a Cognitive Brain, not a physics engine.** LLMs are historically poor at calculus and geometry. Therefore, MechOS is designed so the AI outputs *high-level spatial intents* (e.g., "Move hand to X, Y, Z"). The underlying Universal Integration Layer is responsible for translating those intents into low-level math (Inverse Kinematics) and physical motor currents.

This separation of orchestration and execution ensures that if the LLM hallucinates or crashes, the system's "Kernel" catches the fault before any physical harm occurs.

## System Expectations
To run and utilize MechOS effectively, the system expects:
1. **A Rust Toolchain**: For compiling and running the OS (Edition 2024, stable toolchain).
2. **An AI Provider**:
   - **Local (Offline-first)**: A running local Ollama instance (typically on `http://localhost:11434`) with a pulled model (e.g., `llama3`). This is the default and recommended approach for latency and privacy.
   - **Cloud**: API keys for OpenAI or Anthropic if opting for cloud-based cognition.
3. **Environment setup**: The ability to bind to local ports for its Web UI and Dashboard ROS bridge adapters.

## Step-by-Step Configuration

When you run MechOS for the first time via its Command Line Interface (`cargo run` in the `mechos-cli` crate or the compiled binary), it will guide you through a **First-Run Wizard** if it doesn't detect an existing configuration.

Here is what the configuration entails:

### 1. The Configuration File (`~/.mechos/config.toml`)
MechOS stores its settings securely in `~/.mechos/config.toml`. The wizard helps you populate this file, but you can also edit it later using the `/settings` command in the interactive REPL.

### 2. Choosing an AI Provider
The wizard will prompt you to select your AI provider:
- **1) Local AI via Ollama (default)**: The system expects Ollama to be running locally.
- **2) Cloud AI via OpenAI**
- **3) Cloud AI via Anthropic**

### 3. Setting Up Ports
MechOS exposes two primary network interfaces that need configuration:
- **Dashboard Port (Default: `9090`)**: The WebSocket port for the ROS 2 Dashboard / rosbridge adapter. This is where physical hardware or simulation dashboards connect to send and receive telemetry and commands.
- **Web UI Port (Default: `8080`)**: The HTTP port for the MechOS Web UI, which provides a user-facing interface to interact with the OS.

### 4. Advanced Configuration (Environment Variables)
For production deployments or advanced use cases, MechOS supports overriding the `config.toml` settings via environment variables. This is the recommended way to handle sensitive API keys. Supported variables include:
- `MECHOS_OLLAMA_URL`
- `MECHOS_MODEL`
- `MECHOS_DASHBOARD_PORT`
- `MECHOS_WEBUI_PORT`
- `MECHOS_OPENAI_API_KEY`
- `MECHOS_ANTHROPIC_API_KEY`

Once the First-Run Wizard completes, your configuration is saved securely, and MechOS will probe the selected AI provider to ensure connectivity before dropping you into the interactive shell.