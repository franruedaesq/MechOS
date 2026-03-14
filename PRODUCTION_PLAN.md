# MechOS Production Readiness Plan

This document outlines a comprehensive, phased roadmap detailing the necessary improvements, refactoring, and additions required to transition the MechOS Agent Operating System from an early/alpha stage to a robust, production-ready system for physical robotics.

## Phase 1: Robustness, Stability & Error Handling
*Goal: Ensure the system can run continuously without crashing due to unexpected inputs, data formats, or system states.*

1. **Eliminate Bare `unwrap()` and `expect()` Calls:**
   - **Audit:** Systematically audit all application code (especially in `mechos-middleware`, `mechos-perception`, `mechos-types`, and `mechos-cli`) for `unwrap()` and `expect()`.
   - **Action:** Replace these calls with robust error handling using the `?` operator, pattern matching, or `.unwrap_or_default()`. Map all underlying errors to the standardized `MechError` enum to prevent unhandled runtime panics and ensure smooth failure cascades.
2. **Secure Configuration Management:**
   - **Audit:** Review how sensitive data (like LLM API keys in `~/.mechos/config.toml`) is handled.
   - **Action:** Use `std::os::unix::fs::OpenOptionsExt` to enforce strict `0o600` (read/write by owner only) permissions on Unix systems during the creation of configuration files.
3. **Async Executor Protection:**
   - **Audit:** Identify blocking I/O calls or CPU-intensive operations in the asynchronous codebase.
   - **Action:** Wrap all blocking I/O calls (e.g., SQLite operations in `mechos-memory`) and CPU-heavy tasks (e.g., intensive inverse kinematics or transform frame calculations in `mechos-perception`) within `tokio::task::spawn_blocking` to prevent stalls in the `tokio` asynchronous runtime.

## Phase 2: Observability & Telemetry
*Goal: Gain deep insights into the system's runtime behavior to diagnose issues in production efficiently.*

1. **Distributed Tracing & OpenTelemetry:**
   - **Audit:** Assess existing tracing capabilities.
   - **Action:** Implement comprehensive observability across the workspace. Add `tracing` and OpenTelemetry spans to the `mechos-runtime` LLM driver and `mechos-kernel`. Track prompt generation, LLM inference latency, context retrieval times, and OODA loop cycle durations.
2. **Structured Logging:**
   - **Audit:** Review log formats across all crates.
   - **Action:** Standardize log formats for external aggregation (e.g., JSON logs) and ensure contextual fields (agent ID, hardware component ID, memory sequence ID, timestamp) are consistently included.
3. **System Health Dashboards:**
   - **Audit:** Evaluate current system metrics exposure.
   - **Action:** Enhance the `Watchdog` in `mechos-kernel` to emit periodic health telemetry (e.g., memory usage, event bus lag, hardware registry status) to the `mechos-cockpit` server for real-time monitoring.

## Phase 3: Cost Control & LLM Reliability
*Goal: Manage the operational costs of AI models and ensure high availability of the cognitive engine.*

1. **Token Cost Estimation and Budgeting:**
   - **Audit:** Review the architectural TODOs in the `LlmDriver` (`mechos-runtime`).
   - **Action:** Implement real-time token counting, cost estimation logic, and strict session budgets. If the budget is exceeded, trip a circuit breaker to halt further LLM requests until authorized or reset.
2. **Resilient LLM Inference:**
   - **Audit:** Analyze how network and provider failures are handled.
   - **Action:** Add exponential backoff, retry mechanisms, and provider fallbacks (e.g., failing over from a cloud API to a local Ollama instance) within the `AgentLoop` when rate limits (`LlmError::RateLimitExceeded`) or network timeouts occur.

## Phase 4: Safety & Security Boundaries
*Goal: Harden the system against malicious inputs and physical damage.*

1. **Strict Transport Layer Security:**
   - **Audit:** Inspect network communications.
   - **Action:** Enforce TLS 1.2+ minimums on all network interfaces, including the `mechos-cockpit` web server and API clients, to protect data in transit.
2. **Input Validation and Payload Limits:**
   - **Audit:** Review incoming payload processing.
   - **Action:** Implement maximum payload size limits on incoming WebSockets and REST endpoints in `mechos-middleware` and `mechos-cockpit` to prevent resource exhaustion and DoS attacks.
3. **Physical Safety Invariants (The "Conscience"):**
   - **Audit:** Assess the `StateVerifier` and `CapabilityManager`.
   - **Action:** Expand capability grants to include strict parameter limits (e.g., maximum bounding boxes for `MoveEndEffector`, speed caps for `Drive`). Ensure `mechos-hal` PID controllers enforce anti-windup and gracefully clamp outputs to physical bounds.

## Phase 5: State Persistence & Data Management
*Goal: Ensure long-term operational efficiency without unbounded resource growth.*

1. **Memory Bounds and Pruning:**
   - **Audit:** Monitor the growth of `mechos-memory`'s SQLite database.
   - **Action:** Implement automated pruning policies or tiered storage logic for the `EpisodicStore` to archive or delete outdated episodic embeddings and prevent indefinite database growth.
2. **Database Integrity and Migrations:**
   - **Audit:** Evaluate how database schema changes are managed.
   - **Action:** Add a formal schema migration system for the SQLite database to safely handle upgrades to vector structures or table schemas without risking data loss.

## Phase 6: Hardware & Middleware Resilience
*Goal: Guarantee physical hardware stays synchronized and safe.*

1. **ROS2 DDS Reliability:**
   - **Audit:** Review the `Ros2Adapter` and `Ros2Bridge`.
   - **Action:** Improve the adapters to automatically reconnect and resynchronize gracefully when DDS nodes crash or network partitions occur in noisy robotics environments.
2. **Hardware Halting and Recovery:**
   - **Audit:** Inspect the `Drop` implementations in the `HardwareRegistry`.
   - **Action:** Validate that the `Drop` implementations consistently and safely transition all actuators to a zero-energy state (`set_position(0.0)`) during panic unwinding or expected teardown. Provide a clear manual and automated recovery procedure to resume normal operation after an emergency halt.
