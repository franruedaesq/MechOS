//! `mechos-runtime` – The AI Brain (OODA Loop Engine)
//!
//! The execution engine where the "thinking" happens, implementing the
//! Observe-Orient-Decide-Act (OODA) loop.
//!
//! # Modules
//!
//! - [`llm_driver`] – [`LlmDriver`][llm_driver::LlmDriver]:
//!   an OpenAI-compatible synchronous HTTP client that communicates with local
//!   models such as [Ollama](https://ollama.com) (`http://localhost:11434`).
//!   [`STABILITY_GUIDELINES`][llm_driver::STABILITY_GUIDELINES] are
//!   automatically injected into every system-role message to prevent the model
//!   from getting stuck in repetitive action loops.
//! - [`behavior_tree`] – [`BehaviorNode`][behavior_tree::BehaviorNode]:
//!   a composable behavior tree executor supporting [`Sequence`][behavior_tree::BehaviorNode::Sequence],
//!   [`Selector`][behavior_tree::BehaviorNode::Selector], and
//!   [`Leaf`][behavior_tree::BehaviorNode::Leaf] nodes.  The LLM selects
//!   high-level behaviors rather than controlling raw motor ticks.
//! - [`loop_guard`] – [`LoopGuard`][loop_guard::LoopGuard]:
//!   a safety mechanism that detects when the LLM is stuck requesting the same
//!   failing action repeatedly and signals that an intervention is required.
//!
//! # Kernel gating
//!
//! Every [`HardwareIntent`][mechos_types::HardwareIntent] produced by the OODA
//! loop **must** pass through [`KernelGate::authorize_and_verify`] before being
//! forwarded to `mechos-hal`.  [`KernelGate`] is re-exported here so that
//! runtime orchestration code can hold and use the gate without an additional
//! explicit dependency on `mechos-kernel`.

pub mod behavior_tree;
pub mod llm_driver;
pub mod loop_guard;

pub use behavior_tree::{BehaviorNode, NodeStatus};
pub use llm_driver::{ChatMessage, LlmDriver, LlmError, Role, STABILITY_GUIDELINES};
pub use loop_guard::LoopGuard;

// Re-export the kernel gate so the runtime can use it as its hardware dispatch
// interception point without callers needing a direct dependency on
// mechos-kernel.
pub use mechos_kernel::KernelGate;

