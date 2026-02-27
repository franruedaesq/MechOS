//! Behavior Tree Engine.
//!
//! Provides a lightweight, composable behavior tree executor.  The LLM
//! selects high-level behaviors by name; the runtime maps those names onto
//! a pre-built tree and ticks it once per OODA loop iteration.
//!
//! # Composites
//!
//! | Node type    | Description                                                      |
//! |--------------|------------------------------------------------------------------|
//! | [`Sequence`] | Ticks children left-to-right; fails on first child failure.     |
//! | [`Selector`] | Ticks children left-to-right; succeeds on first child success.  |
//! | [`Leaf`]     | Executes an arbitrary closure and returns its status.           |
//!
//! # Example
//!
//! ```rust
//! use mechos_runtime::behavior_tree::{BehaviorNode, NodeStatus};
//!
//! // A leaf that always succeeds.
//! let always_ok = BehaviorNode::leaf("always_ok", || NodeStatus::Success);
//!
//! // A sequence of two leaves.
//! let tree = BehaviorNode::sequence(vec![
//!     BehaviorNode::leaf("step_a", || NodeStatus::Success),
//!     BehaviorNode::leaf("step_b", || NodeStatus::Success),
//! ]);
//!
//! assert_eq!(tree.tick(), NodeStatus::Success);
//! ```

// ─────────────────────────────────────────────────────────────────────────────
// NodeStatus
// ─────────────────────────────────────────────────────────────────────────────

/// The execution status returned by a behavior tree node after a single tick.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NodeStatus {
    /// The node completed its task successfully.
    Success,
    /// The node encountered an unrecoverable failure.
    Failure,
    /// The node has started but has not yet finished (long-running actions).
    Running,
}

// ─────────────────────────────────────────────────────────────────────────────
// BehaviorNode
// ─────────────────────────────────────────────────────────────────────────────

/// A node in a behavior tree.
///
/// Build trees with the [`BehaviorNode::leaf`], [`BehaviorNode::sequence`], and
/// [`BehaviorNode::selector`] constructors, then call [`BehaviorNode::tick`] to
/// execute.
pub enum BehaviorNode {
    /// Composite: ticks children left-to-right, returns [`NodeStatus::Failure`]
    /// on the first failure, [`NodeStatus::Success`] if all succeed.
    Sequence(Vec<BehaviorNode>),
    /// Composite: ticks children left-to-right, returns [`NodeStatus::Success`]
    /// on the first success, [`NodeStatus::Failure`] if all fail.
    Selector(Vec<BehaviorNode>),
    /// Leaf: executes a boxed closure and returns its [`NodeStatus`].
    Leaf {
        name: String,
        action: Box<dyn Fn() -> NodeStatus + Send + Sync>,
    },
}

impl BehaviorNode {
    /// Construct a [`BehaviorNode::Leaf`] node.
    ///
    /// `action` is called exactly once per [`tick`][BehaviorNode::tick].
    pub fn leaf(
        name: impl Into<String>,
        action: impl Fn() -> NodeStatus + Send + Sync + 'static,
    ) -> Self {
        BehaviorNode::Leaf {
            name: name.into(),
            action: Box::new(action),
        }
    }

    /// Construct a [`BehaviorNode::Sequence`] composite.
    pub fn sequence(children: Vec<BehaviorNode>) -> Self {
        BehaviorNode::Sequence(children)
    }

    /// Construct a [`BehaviorNode::Selector`] composite.
    pub fn selector(children: Vec<BehaviorNode>) -> Self {
        BehaviorNode::Selector(children)
    }

    /// Tick this node, executing its logic and returning the resulting
    /// [`NodeStatus`].
    pub fn tick(&self) -> NodeStatus {
        match self {
            BehaviorNode::Leaf { action, .. } => action(),

            BehaviorNode::Sequence(children) => {
                for child in children {
                    match child.tick() {
                        NodeStatus::Success => continue,
                        other => return other,
                    }
                }
                NodeStatus::Success
            }

            BehaviorNode::Selector(children) => {
                for child in children {
                    match child.tick() {
                        NodeStatus::Failure => continue,
                        other => return other,
                    }
                }
                NodeStatus::Failure
            }
        }
    }

    /// Return the name of this node, if it is a [`BehaviorNode::Leaf`].
    ///
    /// Composite nodes return `None`.
    pub fn name(&self) -> Option<&str> {
        match self {
            BehaviorNode::Leaf { name, .. } => Some(name),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leaf_returns_its_status() {
        let ok = BehaviorNode::leaf("ok", || NodeStatus::Success);
        let fail = BehaviorNode::leaf("fail", || NodeStatus::Failure);
        let running = BehaviorNode::leaf("running", || NodeStatus::Running);

        assert_eq!(ok.tick(), NodeStatus::Success);
        assert_eq!(fail.tick(), NodeStatus::Failure);
        assert_eq!(running.tick(), NodeStatus::Running);
    }

    #[test]
    fn sequence_succeeds_when_all_children_succeed() {
        let tree = BehaviorNode::sequence(vec![
            BehaviorNode::leaf("a", || NodeStatus::Success),
            BehaviorNode::leaf("b", || NodeStatus::Success),
            BehaviorNode::leaf("c", || NodeStatus::Success),
        ]);
        assert_eq!(tree.tick(), NodeStatus::Success);
    }

    #[test]
    fn sequence_fails_on_first_failure() {
        let tree = BehaviorNode::sequence(vec![
            BehaviorNode::leaf("a", || NodeStatus::Success),
            BehaviorNode::leaf("b", || NodeStatus::Failure),
            BehaviorNode::leaf("c", || NodeStatus::Success),
        ]);
        assert_eq!(tree.tick(), NodeStatus::Failure);
    }

    #[test]
    fn sequence_propagates_running() {
        let tree = BehaviorNode::sequence(vec![
            BehaviorNode::leaf("a", || NodeStatus::Success),
            BehaviorNode::leaf("b", || NodeStatus::Running),
            BehaviorNode::leaf("c", || NodeStatus::Success),
        ]);
        assert_eq!(tree.tick(), NodeStatus::Running);
    }

    #[test]
    fn sequence_empty_succeeds() {
        let tree = BehaviorNode::sequence(vec![]);
        assert_eq!(tree.tick(), NodeStatus::Success);
    }

    #[test]
    fn selector_succeeds_on_first_success() {
        let tree = BehaviorNode::selector(vec![
            BehaviorNode::leaf("a", || NodeStatus::Failure),
            BehaviorNode::leaf("b", || NodeStatus::Success),
            BehaviorNode::leaf("c", || NodeStatus::Failure),
        ]);
        assert_eq!(tree.tick(), NodeStatus::Success);
    }

    #[test]
    fn selector_fails_when_all_children_fail() {
        let tree = BehaviorNode::selector(vec![
            BehaviorNode::leaf("a", || NodeStatus::Failure),
            BehaviorNode::leaf("b", || NodeStatus::Failure),
        ]);
        assert_eq!(tree.tick(), NodeStatus::Failure);
    }

    #[test]
    fn selector_propagates_running() {
        let tree = BehaviorNode::selector(vec![
            BehaviorNode::leaf("a", || NodeStatus::Failure),
            BehaviorNode::leaf("b", || NodeStatus::Running),
        ]);
        assert_eq!(tree.tick(), NodeStatus::Running);
    }

    #[test]
    fn selector_empty_fails() {
        let tree = BehaviorNode::selector(vec![]);
        assert_eq!(tree.tick(), NodeStatus::Failure);
    }

    #[test]
    fn nested_sequence_in_selector() {
        // selector([seq([ok, ok]), seq([ok, fail])]) -> Success
        let tree = BehaviorNode::selector(vec![
            BehaviorNode::sequence(vec![
                BehaviorNode::leaf("a", || NodeStatus::Success),
                BehaviorNode::leaf("b", || NodeStatus::Success),
            ]),
            BehaviorNode::sequence(vec![
                BehaviorNode::leaf("c", || NodeStatus::Success),
                BehaviorNode::leaf("d", || NodeStatus::Failure),
            ]),
        ]);
        assert_eq!(tree.tick(), NodeStatus::Success);
    }

    #[test]
    fn leaf_name_is_accessible() {
        let node = BehaviorNode::leaf("my_action", || NodeStatus::Success);
        assert_eq!(node.name(), Some("my_action"));
    }

    #[test]
    fn composite_name_is_none() {
        let seq = BehaviorNode::sequence(vec![]);
        let sel = BehaviorNode::selector(vec![]);
        assert_eq!(seq.name(), None);
        assert_eq!(sel.name(), None);
    }
}
