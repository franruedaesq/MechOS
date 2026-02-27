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
//! - [`behavior_tree`] – [`BehaviorNode`][behavior_tree::BehaviorNode]:
//!   a composable behavior tree executor supporting [`Sequence`][behavior_tree::BehaviorNode::Sequence],
//!   [`Selector`][behavior_tree::BehaviorNode::Selector], and
//!   [`Leaf`][behavior_tree::BehaviorNode::Leaf] nodes.  The LLM selects
//!   high-level behaviors rather than controlling raw motor ticks.
//! - [`loop_guard`] – [`LoopGuard`][loop_guard::LoopGuard]:
//!   a safety mechanism that detects when the LLM is stuck requesting the same
//!   failing action repeatedly and signals that an intervention is required.

pub mod behavior_tree;
pub mod llm_driver;
pub mod loop_guard;

pub use behavior_tree::{BehaviorNode, NodeStatus};
pub use llm_driver::{ChatMessage, LlmDriver, LlmError, Role};
pub use loop_guard::LoopGuard;

