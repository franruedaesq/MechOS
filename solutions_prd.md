# Product Requirements Document (PRD): Resolving Open MechOS Challenges

## Overview

This PRD outlines the technical implementation plan for resolving open questions in the MechOS architecture regarding Spatial Contracts, Integration Math, Safety Determinism, Perception Loops, and Communication Protocols.

---

## Phase 1: The Spatial Contract & Coordinate Authority

### 1.1 Goal
Establish a definitive "Zero Point", standardize units, and explicitly manage nested coordinates to ensure the LLM generates physically meaningful `HardwareIntent` commands.

### 1.2 Technical Implementation
- **Unit Standardization:**
  - Define a strict `SystemUnits` enum in `mechos-types` (e.g., Meters, Radians).
  - Inject unit requirements explicitly into the `schemars`-generated JSON Schema for `MoveEndEffector` and `Drive` intents.
  - Add a validation step in `mechos-kernel::StateVerifier` to reject intents that deviate from standard bounds, assuming the standard units.

- **Coordinate Frame Authority & Transform Tree (tf2):**
  - Enhance `mechos-perception::Transform` engine to strictly define `world`, `base_link`, and `end_effector` frames.
  - Implement a mechanism where the LLM prompt explicitly states the current frame of reference (e.g., "All coordinates are relative to `base_link` in meters").
  - The Universal Integration Layer (UIL) will be responsible for translating LLM egocentric coordinates into the global frame using the Transform Tree before passing them to the IK solver.

---

## Phase 2: The Integration Layer & Math Services

### 2.1 Goal
Provide robust math "as a service" to translate high-level LLM intents into safe, precise robot movements, handling IK, trajectory planning, and singularities.

### 2.2 Technical Implementation
- **IK Engine Abstraction:**
  - Standardize the IK interface in `mechos-hal::Registry`. Allow registration of specific numerical solvers (like Trac-IK or KDL) via plugins or external ROS2 nodes.
  - If a closed-form analytical solver is available for a specific arm, it should be registered as the primary solver for minimal latency.

- **Trajectory Planning & Self-Collision:**
  - Introduce an intermediate `Trajectory` intent within `mechos-types` that represents a sequence of safe waypoints.
  - The UIL (via MoveIt 2 or a native Rust planner) will accept `MoveEndEffector`, query the `mechos-perception::Octree` for self-collision and environment collision, and generate the `Trajectory`.
  - The LLM will only command the final destination or key waypoints, not the continuous curve.

- **Singularity Management:**
  - Implement a `SingularityRule` in `mechos-kernel::StateVerifier`.
  - The rule will pre-calculate the manipulability ellipsoid (or use simplified workspace heuristics) for the requested `x, y, z`. If the target is at or near a singularity, the command is rejected with a specific `MechError::KinematicSingularity`, prompting the LLM to choose an alternative approach.

---

## Phase 3: Safety, Determinism & Catastrophe Prevention

### 3.1 Goal
Ensure the Kernel acts as an absolute safety officer with real-time pre-emption, velocity limiting, and robust watchdog mechanisms.

### 3.2 Technical Implementation
- **Real-time Pre-emption:**
  - Implement a high-priority `EmergencyStop` broadcast channel in `mechos-middleware::EventBus`.
  - Hardware drivers in `mechos-hal` must listen to this channel asynchronously and immediately cut motor power or set velocities to 0, bypassing the normal command queue.

- **Velocity/Torque Limiting:**
  - Enhance the `mechos-hal::PID` controller with an "Acceleration Profiler" (e.g., trapezoidal or S-curve velocity profiling).
  - The `StateVerifier` will continue to enforce maximum speed caps, but the HAL will ensure the transition to that speed obeys maximum acceleration limits to protect joints.

- **Watchdog & Fallback Logic:**
  - Extend the `mechos-kernel::Watchdog`. If a critical component (like the LLM driver) misses a heartbeat, the Watchdog will issue a `SafeHalt` command to the `EventBus`.
  - The HAL will receive `SafeHalt` and execute a predefined safe posture or slowly ramp down velocities, rather than just cutting power abruptly (which could drop a payload).

---

## Phase 4: Perception-Action Loop & Brain Feedback

### 4.1 Goal
Provide the LLM with a highly compressed, accurate representation of the world state and close the proprioceptive loop.

### 4.2 Technical Implementation
- **World State Serialization:**
  - Create a `StateSerializer` in `mechos-perception`. It will take the complex `Octree` and identify clusters of occupied voxels.
  - It will output a simplified JSON list of "Obstacle Bounding Boxes" (e.g., `[{"id": 1, "center": [1.0, 0.5, 0.0], "size": [0.2, 0.2, 0.2]}]`) to be injected into the LLM context window.

- **Proprioception (Closing the Loop):**
  - Ensure the `mechos-runtime::AgentLoop`'s "Observe" phase explicitly requests the current `end_effector` position from the `Transform` engine.
  - The prompt will read: "Current Hand Position: [X, Y, Z]. Target was: [Target_X, Target_Y, Target_Z]. Error is [E]." This allows the LLM to correct its own high-level plans based on physical execution error.

---

## Phase 5: Communication Protocol & Real-time Sync

### 5.1 Goal
Minimize latency between the Brain and Muscle, and ensure state consistency across physical and virtual domains.

### 5.2 Technical Implementation
- **Low-Latency Communication:**
  - While HTTP/WebSockets are fine for the Dashboard, core real-time traffic (Brain to Muscle) must prioritize the `tokio::sync::broadcast` channels internally.
  - For distributed setups (e.g., Brain on PC, Muscle on ESP32), implement a raw UDP or Zenoh bridge in `mechos-middleware` to replace HTTP overhead, ensuring sub-millisecond dispatch of motor commands.

- **State Consistency (Virtual & Physical):**
  - Enforce a strict Single Source of Truth: The `mechos-middleware::EventBus` is the only entity allowed to broadcast state changes.
  - Both `Ros2Adapter` (Physical) and `DashboardSimAdapter` (Virtual) must subscribe to the identical `StateUpdated` events.
  - Implement a logical clock or sequence ID in `EventPayload` to ensure clients process states in order and can detect dropped packets.
