//! Transform Frame (TF) Engine.
//!
//! Maintains a directed graph of named reference frames and the 3-D rigid-body
//! transforms (translation + quaternion rotation) that relate them.  Given any
//! two frame names the engine can compose a chain of transforms via BFS to
//! produce the combined `Transform3D`.
//!
//! # Example
//!
//! ```rust
//! use mechos_perception::transform::{TfEngine, Transform3D, Vec3, Quaternion};
//!
//! let mut tf = TfEngine::new();
//!
//! // robot_base is 1 m forward of world origin, same orientation.
//! tf.set_transform("world", "robot_base",
//!     Transform3D::new(Vec3::new(1.0, 0.0, 0.0), Quaternion::identity()));
//!
//! // camera is 0.5 m forward of robot_base, same orientation.
//! tf.set_transform("robot_base", "camera",
//!     Transform3D::new(Vec3::new(0.5, 0.0, 0.0), Quaternion::identity()));
//!
//! let t = tf.lookup("world", "camera").unwrap();
//! assert!((t.translation.x - 1.5).abs() < 1e-5);
//! ```

use std::collections::{HashMap, HashSet, VecDeque};

// ────────────────────────────────────────────────────────────────────────────
// Primitive types
// ────────────────────────────────────────────────────────────────────────────

/// A 3-D translation vector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    /// Create a new vector.
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// The zero vector.
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

/// A unit quaternion representing a 3-D rotation (w, x, y, z convention).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    /// Create a quaternion.  The caller is responsible for providing a unit
    /// quaternion (|q| = 1).
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// The identity rotation (no rotation).
    pub fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0)
    }

    /// Hamilton product: compose two rotations.
    pub fn mul(self, rhs: Self) -> Self {
        Self::new(
            self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        )
    }

    /// Conjugate (== inverse for a unit quaternion).
    pub fn conjugate(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Rotate a vector by this quaternion: p' = q * p * q*.
    pub fn rotate(self, v: Vec3) -> Vec3 {
        // Express v as a pure quaternion.
        let p = Self::new(0.0, v.x, v.y, v.z);
        let rotated = self.mul(p).mul(self.conjugate());
        Vec3::new(rotated.x, rotated.y, rotated.z)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Transform3D
// ────────────────────────────────────────────────────────────────────────────

/// A rigid-body 3-D transform: translation followed by rotation.
///
/// Represents the pose of frame B relative to frame A: to convert a point
/// expressed in frame B into frame A, rotate it by `rotation` then add
/// `translation`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform3D {
    pub translation: Vec3,
    pub rotation: Quaternion,
}

impl Transform3D {
    /// Create a transform from a translation and rotation.
    pub fn new(translation: Vec3, rotation: Quaternion) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    /// The identity transform (no translation, no rotation).
    pub fn identity() -> Self {
        Self::new(Vec3::zero(), Quaternion::identity())
    }

    /// Compose two transforms: `self` applied first, then `other`.
    ///
    /// If `self` = T_A_B and `other` = T_B_C, the result is T_A_C.
    pub fn compose(self, other: Self) -> Self {
        // Rotate other's translation by self's rotation, then add.
        let translated = self.translation.add(self.rotation.rotate(other.translation));
        let rotated = self.rotation.mul(other.rotation);
        Self::new(translated, rotated)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// TfEngine
// ────────────────────────────────────────────────────────────────────────────

/// A directed graph of named reference frames and the [`Transform3D`]s that
/// relate them.
///
/// Frames are identified by arbitrary string names (e.g. `"world"`,
/// `"robot_base"`, `"camera"`).  Edges are directional: adding
/// `"A" → "B"` does not automatically create the inverse.
///
/// [`TfEngine::lookup`] performs BFS to find the shortest path from source
/// to target and returns the composed transform.
#[derive(Debug, Default)]
pub struct TfEngine {
    /// `edges[from][to] = Transform3D`
    edges: HashMap<String, HashMap<String, Transform3D>>,
}

impl TfEngine {
    /// Create an empty TF engine.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register or update the transform from `parent_frame` to `child_frame`.
    pub fn set_transform(
        &mut self,
        parent_frame: &str,
        child_frame: &str,
        transform: Transform3D,
    ) {
        self.edges
            .entry(parent_frame.to_string())
            .or_default()
            .insert(child_frame.to_string(), transform);
    }

    /// Compute the composed [`Transform3D`] that maps points in `source_frame`
    /// into `target_frame`.
    ///
    /// Returns `None` if no path exists between the two frames.
    pub fn lookup(&self, source_frame: &str, target_frame: &str) -> Option<Transform3D> {
        if source_frame == target_frame {
            return Some(Transform3D::identity());
        }

        // BFS over the directed graph; each queue item carries the composed
        // transform accumulated from source_frame to the current node.
        let mut queue: VecDeque<(String, Transform3D)> = VecDeque::new();
        let mut visited: HashSet<String> = HashSet::new();

        queue.push_back((source_frame.to_string(), Transform3D::identity()));
        visited.insert(source_frame.to_string());

        while let Some((current, accumulated)) = queue.pop_front() {
            if let Some(neighbours) = self.edges.get(&current) {
                for (next, edge_tf) in neighbours {
                    if visited.contains(next) {
                        continue;
                    }
                    let composed = accumulated.compose(*edge_tf);
                    if next == target_frame {
                        return Some(composed);
                    }
                    visited.insert(next.clone());
                    queue.push_back((next.clone(), composed));
                }
            }
        }

        None
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_1_SQRT_2;

    // ── Quaternion ──────────────────────────────────────────────────────────

    #[test]
    fn quaternion_identity_rotate_is_noop() {
        let q = Quaternion::identity();
        let v = Vec3::new(1.0, 2.0, 3.0);
        let r = q.rotate(v);
        assert!((r.x - 1.0).abs() < 1e-5);
        assert!((r.y - 2.0).abs() < 1e-5);
        assert!((r.z - 3.0).abs() < 1e-5);
    }

    #[test]
    fn quaternion_90deg_yaw_rotates_x_to_y() {
        // 90° rotation around Z axis: (cos45°, 0, 0, sin45°)
        let q = Quaternion::new(FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2);
        let v = Vec3::new(1.0, 0.0, 0.0);
        let r = q.rotate(v);
        assert!((r.x).abs() < 1e-5, "x should be ~0, got {}", r.x);
        assert!((r.y - 1.0).abs() < 1e-5, "y should be ~1, got {}", r.y);
        assert!((r.z).abs() < 1e-5);
    }

    #[test]
    fn quaternion_conjugate_is_inverse() {
        let q = Quaternion::new(FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2);
        let prod = q.mul(q.conjugate());
        // q * q* should be identity (w≈1, x≈y≈z≈0)
        assert!((prod.w - 1.0).abs() < 1e-5);
        assert!(prod.x.abs() < 1e-5);
        assert!(prod.y.abs() < 1e-5);
        assert!(prod.z.abs() < 1e-5);
    }

    // ── Transform3D ─────────────────────────────────────────────────────────

    #[test]
    fn transform_identity_compose_is_noop() {
        let t = Transform3D::new(Vec3::new(1.0, 2.0, 3.0), Quaternion::identity());
        let composed = Transform3D::identity().compose(t);
        assert!((composed.translation.x - 1.0).abs() < 1e-5);
        assert!((composed.translation.y - 2.0).abs() < 1e-5);
        assert!((composed.translation.z - 3.0).abs() < 1e-5);
    }

    #[test]
    fn transform_compose_translations_add() {
        let t1 = Transform3D::new(Vec3::new(1.0, 0.0, 0.0), Quaternion::identity());
        let t2 = Transform3D::new(Vec3::new(2.0, 0.0, 0.0), Quaternion::identity());
        let composed = t1.compose(t2);
        assert!((composed.translation.x - 3.0).abs() < 1e-5);
    }

    // ── TfEngine ────────────────────────────────────────────────────────────

    #[test]
    fn lookup_same_frame_returns_identity() {
        let tf = TfEngine::new();
        let t = tf.lookup("world", "world").unwrap();
        assert_eq!(t, Transform3D::identity());
    }

    #[test]
    fn lookup_direct_edge() {
        let mut tf = TfEngine::new();
        let expected = Transform3D::new(Vec3::new(1.0, 0.0, 0.0), Quaternion::identity());
        tf.set_transform("world", "robot_base", expected);

        let t = tf.lookup("world", "robot_base").unwrap();
        assert!((t.translation.x - 1.0).abs() < 1e-5);
    }

    #[test]
    fn lookup_composed_chain() {
        let mut tf = TfEngine::new();
        tf.set_transform(
            "world",
            "robot_base",
            Transform3D::new(Vec3::new(1.0, 0.0, 0.0), Quaternion::identity()),
        );
        tf.set_transform(
            "robot_base",
            "camera",
            Transform3D::new(Vec3::new(0.5, 0.0, 0.0), Quaternion::identity()),
        );

        let t = tf.lookup("world", "camera").unwrap();
        assert!((t.translation.x - 1.5).abs() < 1e-5);
    }

    #[test]
    fn lookup_no_path_returns_none() {
        let mut tf = TfEngine::new();
        tf.set_transform(
            "world",
            "robot_base",
            Transform3D::new(Vec3::new(1.0, 0.0, 0.0), Quaternion::identity()),
        );
        assert!(tf.lookup("robot_base", "world").is_none());
        assert!(tf.lookup("world", "ghost_frame").is_none());
    }

    #[test]
    fn set_transform_overrides_previous() {
        let mut tf = TfEngine::new();
        tf.set_transform(
            "world",
            "sensor",
            Transform3D::new(Vec3::new(1.0, 0.0, 0.0), Quaternion::identity()),
        );
        tf.set_transform(
            "world",
            "sensor",
            Transform3D::new(Vec3::new(5.0, 0.0, 0.0), Quaternion::identity()),
        );

        let t = tf.lookup("world", "sensor").unwrap();
        assert!((t.translation.x - 5.0).abs() < 1e-5);
    }

    #[test]
    fn lookup_respects_rotation_in_chain() {
        // robot_base is at world origin, rotated 90° around Z.
        // camera is 1 m forward in robot_base frame (local +X).
        // After the 90° yaw, camera's world position should be (0, 1, 0).
        let q90z = Quaternion::new(FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2);
        let mut tf = TfEngine::new();
        tf.set_transform(
            "world",
            "robot_base",
            Transform3D::new(Vec3::zero(), q90z),
        );
        tf.set_transform(
            "robot_base",
            "camera",
            Transform3D::new(Vec3::new(1.0, 0.0, 0.0), Quaternion::identity()),
        );

        let t = tf.lookup("world", "camera").unwrap();
        assert!(t.translation.x.abs() < 1e-5, "x={}", t.translation.x);
        assert!((t.translation.y - 1.0).abs() < 1e-5, "y={}", t.translation.y);
        assert!(t.translation.z.abs() < 1e-5);
    }
}
