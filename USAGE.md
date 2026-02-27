# Usage Guide

This project is a set of Rust libraries that form the core of the MechOS Agent Operating System. It is currently structured as a Cargo Workspace containing multiple crates.

## Prerequisites

To work with this codebase, you need the following installed:

1.  **Rust Toolchain**: Ensure you have the latest stable Rust installed (via `rustup`).
    ```bash
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
    ```
2.  **Ollama**: The default LLM backend is Ollama. Install and run it locally.
    *   Install from [ollama.com](https://ollama.com).
    *   Pull the default model (Llama 3):
        ```bash
        ollama run llama3
        ```
    *   Ensure the Ollama server is running (usually on `http://localhost:11434`).

## How to Run

Currently, the project does not have a single executable binary. It is designed to be used as a library or developed further by adding a binary crate.

### 1. Running Tests

The primary way to verify the system is working is by running the comprehensive test suite. This includes unit tests for the logic, safety verifiers, and mocked interactions.

```bash
cargo test
```

### 2. Theoretical Usage (Creating a Binary)

To run a MechOS agent, you would create a `main.rs` file in a new binary crate (or add a `[[bin]]` target to `mechos-runtime`). Here is a minimal example of how to bootstrap the agent loop:

**`src/main.rs`**

```rust
use mechos_runtime::agent_loop::{AgentLoop, AgentLoopConfig};
use mechos_types::MechError;
use std::thread;
use std::time::{Duration, Instant};

fn main() -> Result<(), MechError> {
    // 1. Configure the Agent
    let config = AgentLoopConfig::default();

    // 2. Initialize the Loop
    // Note: In a real app, you'd handle the Result properly instead of unwrapping
    let mut agent = AgentLoop::new(config);

    println!("MechOS Agent Initialized. Entering OODA Loop...");

    let mut last_tick = Instant::now();

    // 3. Run the Loop
    loop {
        // Calculate delta time for physics/fusion updates
        let dt = last_tick.elapsed().as_secs_f32();
        last_tick = Instant::now();

        // Execute one tick of the OODA loop
        match agent.tick(dt) {
            Ok(intent) => {
                println!("Agent Decided: {:?}", intent);
                // In a real robot, this intent would be sent to the HAL
            }
            Err(e) => {
                eprintln!("Error: {}", e);
                // Handle errors (e.g., LLM timeout, safety violation)
            }
        }

        // Maintain a control loop frequency (e.g., 10Hz)
        thread::sleep(Duration::from_millis(100));
    }
}
```

### 3. Exploring the Capabilities

The system exposes several key capabilities you can experiment with in tests or your own binary:

*   **`mechos-kernel`**: Define custom safety rules by implementing the `Rule` trait.
*   **`mechos-perception`**: Feed raw sensor data into `SensorFusion` and query the `Octree` for obstacles.
*   **`mechos-memory`**: Store and retrieve semantic memories using `EpisodicStore`.

## Troubleshooting

*   **LLM Connection Refused**: Ensure Ollama is running on port 11434. Check `curl http://localhost:11434`.
*   **Compilation Errors**: Run `cargo clean` and `cargo build` to ensure a fresh build.
*   **Missing Dependencies**: Ensure you have `pkg-config` and `openssl` installed if required by `reqwest` on your platform (e.g., `sudo apt install pkg-config libssl-dev` on Ubuntu).
