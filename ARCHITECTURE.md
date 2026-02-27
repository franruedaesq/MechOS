# MechOS Software Architecture

MechOS is an Agent Operating System designed to bridge the gap between high-level AI cognition (LLMs) and low-level physical robotics control.

## High-Level Concept: The "Brain, Body, and Conscience"

To understand MechOS, it helps to think of the robot as a biological entity:

1.  **The Brain (`mechos-runtime`)**: This is the cognitive center. It observes the world, remembers past experiences, and decides on a course of action. It thinks in abstract concepts like "Pick up the cup" or "Move to the kitchen". It doesn't know *how* to move the motors, just *what* it wants to achieve.
2.  **The Body (`mechos-hal` & `mechos-middleware`)**: This is the physical execution layer. It translates the Brain's abstract commands into electrical signals, motor currents, and joint angles. It also provides the senses (Lidar, Cameras) that feed information back to the Brain.
3.  **The Conscience (`mechos-kernel`)**: This is the safety and policy enforcement layer. It acts as a filter between the Brain and the Body. If the Brain decides to do something dangerous (like "drive through that wall"), the Conscience intervenes and blocks the action before it reaches the Body. It ensures the robot adheres to safety rules and permissions.

## Technical Architecture

MechOS is built as a **Modular Monolith** using a Rust Cargo Workspace. This means the system is composed of distinct, loosely coupled libraries (crates) that are compiled together into a single binary.

### Key Components

*   **`mechos-types`**: The common language of the system. It defines the data structures (Events, Intents, Capabilities) shared by all other components. It has no dependencies on other crates.
*   **`mechos-middleware`**: The nervous system. It handles communication with the outside world (ROS2, WebSockets) and routes messages internally. It doesn't "understand" the data, it just moves it.
*   **`mechos-hal`**: The Hardware Abstraction Layer. It provides a generic interface for physical hardware. It uses PID controllers to smooth out movements and handles the specific details of different actuators.
*   **`mechos-perception`**: The sensory processing unit. It takes raw data (Lidar scans, Odometry) and turns it into a coherent model of the world (e.g., "I am at X, Y" or "There is an obstacle here"). It uses Sensor Fusion to combine data from multiple sources.
*   **`mechos-memory`**: The long-term memory. It stores past interactions and events using a vector database. This allows the agent to "remember" context and learn from past experiences.
*   **`mechos-kernel`**: The security and safety enforcer. It implements:
    *   **Capability-based Security**: Agents must have explicit permission (Capabilities) to access hardware or data.
    *   **State Verification**: A rule engine that checks every action against physical safety limits (e.g., max speed, workspace bounds).
*   **`mechos-runtime`**: The main execution loop (OODA Loop). It drives the agent's behavior:
    *   **Observe**: Gather state from Perception.
    *   **Orient**: retrieve memories and build a context for the AI.
    *   **Decide**: Send the context to the LLM (Large Language Model) to get the next action.
    *   **Act**: Send the chosen action to the Kernel for verification.

### The OODA Loop

The core of `mechos-runtime` is the OODA Loop (Observe-Orient-Decide-Act). This is an infinite cycle that runs continuously:

1.  **Observe**: The system queries `mechos-perception` for the latest robot state (position, obstacles).
2.  **Orient**: The system constructs a "System Prompt" for the LLM. This prompt includes the current state, relevant memories from `mechos-memory`, and available tools.
3.  **Decide**: The prompt is sent to the LLM (e.g., via Ollama). The LLM returns a structured JSON response representing its intent (e.g., `MoveEndEffector { x: 1.0, ... }`).
4.  **Gatekeep (The Conscience)**: The Intent is passed to `mechos-kernel`. The Kernel checks:
    *   Does the agent have the `Capability` to perform this action?
    *   Does the action violate any `StateVerifier` rules (e.g., speed limits)?
5.  **Act**: If the Kernel approves, the Intent is published to the Event Bus. `mechos-hal` or `mechos-middleware` picks it up and executes it.

### Data Flow

1.  **Sensors -> Middleware -> Perception**: Raw sensor data comes in via ROS2, is normalized by Middleware, and fused by Perception into a `FusedState`.
2.  **Perception -> Runtime**: The Runtime reads the `FusedState`.
3.  **Runtime -> LLM -> Runtime**: The Runtime sends state to the LLM and gets back an `Intent`.
4.  **Runtime -> Kernel**: The Intent is sent to the Kernel for approval.
5.  **Kernel -> Bus -> HAL**: Approved Intents are put on the Bus. HAL subscribes to the Bus, reads the Intent, and drives the motors.
