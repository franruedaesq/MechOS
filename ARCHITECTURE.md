# MechOS Architecture & System Design Guide

Welcome to the architectural deep-dive of **MechOS**. This document is designed to bridge the gap between a beginner's understanding of robotics and the rigorous systems thinking of a Senior Software Architect. We will explore *how* this operating system is built, *why* it was built that way, and define the technical concepts that make it tick.

---

## 1. High-Level Concept: The "Brain, Body, and Conscience" Model

At its simplest, MechOS is designed like a biological organism.

*   **The Brain (Runtime & LLM):** This is the creative, intelligent part. It looks at the world, decides what to do, and makes plans. However, like a human brain, it can sometimes be wrong, distracted, or "hallucinate."
*   **The Body (HAL & Middleware):** These are the muscles and nerves. They don't think; they just do exactly what they are told. If you tell the arm to move to a dangerous position, the arm will try to do it.
*   **The Conscience (Kernel):** This is the critical safety layer. It sits between the Brain and the Body. It checks every command from the Brain. If the Brain says "punch yourself in the face," the Conscience says "No, that violates the safety rules," and blocks the command.

### 🏛️ Senior Architect's View: The "Safety-Critical Control Loop"

In technical terms, this is a **Safety-Critical Control Loop**. We treat the Large Language Model (LLM) as a **Non-Deterministic** component—meaning we cannot predict its output 100% of the time. We treat the hardware as **Deterministic**—it obeys physics and code strictly.

The architecture is designed to **contain the uncertainty** of the AI within a "sandbox" (the Runtime) and filter its outputs through a rigorous **Verification Layer** (the Kernel) before any physical actuation occurs. This is the **Principle of Least Privilege** applied to AI: the AI is only allowed to do what the Kernel explicitly permits it to do.

---

## 2. System Architecture: The Modular Monolith

MechOS is built as a **Modular Monolith**.

*   **Beginner Explanation:** Imagine building a Lego castle. You could glue all the pieces together into one big lump (a Monolith), or you could build separate towers, bridges, and walls that snap together (Modular). MechOS is the latter. All the code lives in one project (repo), but it is split into separate folders (crates) that have strict rules about how they talk to each other.
*   **Technical Definition:** A **Modular Monolith** is a software architecture where the system is deployed as a single unit (binary) but structured internally as independent modules with defined boundaries. This provides the simplicity of deployment of a monolith with the maintainability of microservices.

### The Crate Structure (Dependency Graph)

The system is organized into **Crates** (Rust's term for libraries/packages). The dependency flows strictly downwards:

1.  `mechos-cli` / `mechos-cockpit` (The Entry Points)
    *   *Depends on:* Runtime, Middleware, Kernel
2.  `mechos-runtime` (The OODA Loop)
    *   *Depends on:* Perception, Memory, Kernel, Middleware
3.  `mechos-kernel` (The Safety Layer)
    *   *Depends on:* Types
4.  `mechos-perception` (Vision & Sensing)
    *   *Depends on:* Types
5.  `mechos-memory` (Storage)
    *   *Depends on:* Types
6.  `mechos-hal` (Hardware Abstraction)
    *   *Depends on:* Types
7.  `mechos-middleware` (Communication)
    *   *Depends on:* Types
8.  `mechos-types` (The Shared Vocabulary)
    *   *Depends on:* Nothing.

---

## 3. Core Architectural Patterns

### A. Event-Driven Architecture (The Nervous System)

**Concept:** Instead of Component A calling Component B directly (like calling a phone number), Component A shouts "I did something!" into a room, and anyone who cares listens.

*   **In MechOS:** The `mechos-middleware` crate provides an **Event Bus**.
*   **How it works:**
    1.  The LiDAR sensor publishes a `Telemetry` event: "I see a wall 2 meters away."
    2.  The Perception engine listens for `Telemetry`.
    3.  The Dashboard listens for `Telemetry` to draw the map.
    4.  The components are **decoupled**—the LiDAR driver doesn't know the Perception engine exists.

**Technical Term: Decoupling**
This means separating components so that changes in one don't break the other. It makes the system easier to test and upgrade.

### B. Hardware Abstraction Layer (HAL)

**Concept:** The "Universal Remote." You don't want to rewrite your robot's brain just because you changed the brand of motor you are using.

*   **In MechOS:** `mechos-hal` defines **Traits** (Interfaces).
    *   `Trait Actuator`: Must have a function `set_position(angle)`.
*   **Implementation:** You can write a `DynamixelMotor` driver or a `SimulatedMotor` driver. Both implement `Actuator`. The Brain just talks to `Actuator`. It doesn't care if it's a real motor or a simulation.

**Technical Term: Dependency Inversion / Interface Segregation**
High-level modules (Brain) should not depend on low-level modules (Specific Motor Driver). Both should depend on abstractions (The `Actuator` Trait).

---

## 4. The Cognitive Cycle: The OODA Loop

The `mechos-runtime` crate runs the robot's main life cycle, known as the **OODA Loop**. This is a concept from military strategy adopted for robotics.

1.  **Observe:**
    *   *Action:* Gather data from sensors.
    *   *Component:* `mechos-perception` fuses Odometry (movement) and IMU (gyroscope) data into a single "State." It also checks the "Octree" (a 3D map) for obstacles.
2.  **Orient:**
    *   *Action:* Make sense of the data. Contextualize.
    *   *Component:* `mechos-memory` retrieves past memories relevant to the current situation ("Have I been here before?"). It builds a prompt for the AI.
3.  **Decide:**
    *   *Action:* Determine the next move.
    *   *Component:* `mechos-runtime` sends the prompt to the LLM (Ollama). The LLM replies with a structured JSON intent (e.g., `Drive { speed: 1.0 }`).
4.  **Act (with Gatekeeping):**
    *   *Action:* Execute the plan.
    *   *Component:* `mechos-kernel` intercepts the intent. It checks:
        *   **Permissions:** Does this AI have the `Capability` to drive?
        *   **Safety:** Is the speed within safe limits? Is the path clear?
        *   If Safe -> Send to `mechos-hal` to move the motors.
        *   If Unsafe -> Reject and trigger an alarm.

---

## 5. Memory Systems: Vector Database

Robots usually have "amnesia"—they process the current frame and forget it. MechOS gives the robot **Episodic Memory**.

*   **How it works:** When something happens, the system writes a summary ("I avoided a cat") and converts that text into a **Vector Embedding** (a long list of numbers representing the *meaning* of the text).
*   **Retrieval:** When the robot is in a new situation, it asks the database: "Give me memories that are mathematically similar to what is happening now."
*   **Result:** The robot "remembers" similar past situations to help it decide what to do.

**Technical Term: Semantic Search / RAG (Retrieval-Augmented Generation)**
This is the process of fetching relevant data to provide context to an LLM, allowing it to "know" things outside of its initial training data.

---

## Summary for the Aspiring Architect

To think like a Senior Architect about MechOS:

1.  **Don't trust the AI.** Treat it as a powerful but unreliable plugin. Build a cage (Kernel) around it.
2.  **Decouple everything.** Use Events and Interfaces (Traits) so you can swap out parts (Simulators vs Real Hardware) without rewriting the core logic.
3.  **State is King.** Manage your data (Memory, Perception) carefully. A robot that doesn't know where it is cannot make good decisions.
4.  **Safety First.** In software, a bug crashes the app. In robotics, a bug crashes the robot into a wall. The architecture must prioritize physical safety over everything else.
