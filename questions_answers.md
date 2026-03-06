# 1. The Spatial Contract: "How do we define the world?"

**Coordinate Frame Authority:**
MechOS relies on the `mechos-perception` crate, specifically its Transform Frame (TF) Engine, which computes spatial transforms (translations, rotations) between named reference frames. Based on typical ROS2 and robotics patterns, MechOS supports both global and local frames by using this Transform Tree. The LLM primarily reasons in a local/egocentric system, and the Transform Engine computes relative coordinates.

**Unit Standardization:**
Unit standardization is enforced by the OS's shared vocabulary in `mechos-types` and the Universal Integration Layer. To ensure consistency, all coordinate values in the system (like `x, y, z` in `MoveEndEffector`) are implicitly defined in a standard unit (typically meters in ROS-like ecosystems), and the generated JSON Schema injected into the LLM prompt provides strict typing.

**Transform Tree (tf2):**
The `mechos-perception` crate includes a Transform Frame (TF) Engine. It handles nested coordinates via a directed graph computing spatial transforms between named reference frames, similar to ROS `tf2`. When a parent frame (e.g., torso) moves, the TF Engine calculates the updated relative position of child frames (e.g., hand) with respect to the world.

# 2. The Integration Layer: "How do we handle the math?"

**IK Engine:**
MechOS abstracts Inverse Kinematics (IK) out of the core AI reasoning loop using the Universal Integration Layer. The OS exposes `HardwareIntent::MoveEndEffector { x, y, z }`, which is dispatched to the Universal ROS2 Bridge (`mechos-middleware`). The Bridge offloads IK resolution to tools like MoveIt 2 (numerical solvers like KDL/Trac-IK) or a registered custom IK adapter before commanding joint angles.

**Trajectory Planning:**
Trajectory planning is delegated to the low-level execution layers via the Universal Integration Layer (e.g., MoveIt 2 via the ROS2 Adapter). `mechos-runtime` observes collision data from the `Octree` in `mechos-perception`, allowing the LLM to provide intermediate safe points or avoid rough areas. Smooth curve generation and self-collision prevention are managed by the integration point.

**Singularity Management:**
The Kernel's `StateVerifier` (specifically `EndEffectorWorkspaceRule`) checks if a target coordinate is within the physical workspace bounds before the command is even sent to the IK solver. If an impossible or dangerous coordinate is requested, the Kernel rejects the intent, preventing the IK solver from entering mathematical singularities.

# 3. Safety & Determinism: "How do we stop a catastrophe?"

**Real-time Pre-emption:**
MechOS enforces safety through `mechos-kernel`. Every `HardwareIntent` must pass through the `KernelGate` (Capability Manager and State Verifier). If real-time sensors detect an obstacle, the middleware or low-level HAL (Hardware Abstraction Layer) can halt the command. Additionally, the CLI intercepts Ctrl-C to send an `EmergencyStop` intent.

**Velocity/Torque Limiting:**
The `StateVerifier` acts as a safety interlock, verifying bounds and enforcing rules such as `SpeedCapRule`. High-level "Move to X" commands are translated into controlled movements by `mechos-hal`, which uses a Generic PID Controller Engine. This ensures smooth motor movements (acceleration curves) and prevents torque spikes that could damage hardware.

**Watchdog Logic:**
The `mechos-kernel` includes a `Watchdog` health monitor. It tracks heartbeats from all components (the LLM driver, adapters, etc.). If the "Brain" crashes or hangs during operation, the Watchdog detects the missing heartbeats and triggers a fallback behavior or restart, ensuring the low-level execution halts safely.

# 4. Perception-Action Loop: "How does the brain see?"

**World State Serialization:**
`mechos-perception` processes complex sensor data (like point clouds from LiDAR) into a spatial map using an `Octree` for 3D space partitioning and collision detection. This data, along with odometry, is fused into a `FusedState`. `mechos-runtime` (the OODA loop) formats this state into a strict system prompt (a highly compressed JSON/text string) so the LLM understands the environment.

**Proprioception:**
The OS uses a closed-loop system where `mechos-perception` constantly updates the `FusedState` with the actual physical state of the robot (from joint encoders, IMU, etc. via the middleware). During the "Observe" and "Orient" phases of the OODA loop, the LLM is fed the true, current state of the robot, closing the loop between intended and actual positions.

# 5. Communication Protocol: "How fast is the bridge?"

**Latency:**
MechOS uses `mechos-middleware` to handle real-time communication. For physical hardware, it utilizes a Universal ROS2 Bridge that converts heavy DDS (Data Distribution Service) traffic into a format usable by the OS. It also employs a lightweight, headless Event Bus (using `tokio::sync::broadcast` channels) for rapid inter-crate communication, avoiding HTTP overhead for core real-time operations.

**State Consistency:**
State consistency between the "Virtual Robot" (DashboardSimAdapter) and the "Physical Robot" (Ros2Adapter) is maintained because both adapters plug into the same central Event Bus and Universal Integration Layer. They receive the exact same `HardwareIntent` commands and publish back to the same state topics, ensuring that any client or physical body is synchronized with the central Kernel's state.
