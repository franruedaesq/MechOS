//! Spatial Query & Collision Engine.
//!
//! Partitions 3-D space using a recursive **Octree** so that the robot
//! (or its planning LLM) can quickly answer queries like "is this path
//! clear?" or "are there any obstacles within this volume?".
//!
//! # Key types
//!
//! | Type | Role |
//! |------|------|
//! | [`Point3`]   | A 3-D coordinate.                                      |
//! | [`Aabb`]     | An axis-aligned bounding box.                          |
//! | [`Octree`]   | Spatial index; insert points, query for collisions.    |
//!
//! # Example
//!
//! ```rust
//! use mechos_perception::octree::{Octree, Aabb, Point3};
//!
//! let bounds = Aabb::new(Point3::new(-10.0, -10.0, -10.0),
//!                        Point3::new( 10.0,  10.0,  10.0));
//! let mut tree = Octree::new(bounds, 8);
//!
//! tree.insert(Point3::new(1.0, 2.0, 3.0));
//!
//! // Exact point query.
//! assert!(tree.contains(Point3::new(1.0, 2.0, 3.0)));
//! assert!(!tree.contains(Point3::new(0.0, 0.0, 0.0)));
//!
//! // Bounding-box overlap query (collision check).
//! let probe = Aabb::new(Point3::new(0.5, 1.5, 2.5),
//!                       Point3::new(1.5, 2.5, 3.5));
//! assert!(tree.query_aabb(&probe));
//! ```

// ────────────────────────────────────────────────────────────────────────────
// Point3
// ────────────────────────────────────────────────────────────────────────────

/// A point in 3-D space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point3 {
    /// Create a new point.
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Aabb
// ────────────────────────────────────────────────────────────────────────────

/// An axis-aligned bounding box, defined by its minimum and maximum corners.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min: Point3,
    pub max: Point3,
}

impl Aabb {
    /// Create a bounding box from its two opposite corners.
    ///
    /// The constructor normalises the corners so that `min ≤ max` per axis.
    pub fn new(a: Point3, b: Point3) -> Self {
        Self {
            min: Point3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)),
            max: Point3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)),
        }
    }

    /// Return the centre point of the box.
    pub fn centre(&self) -> Point3 {
        Point3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }

    /// True when the point lies inside or on the boundary of the box.
    pub fn contains_point(&self, p: Point3) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    /// True when `other` overlaps (intersects or touches) this box.
    pub fn overlaps(&self, other: &Aabb) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Octree
// ────────────────────────────────────────────────────────────────────────────

/// A recursive spatial index that subdivides 3-D space into eight octants.
///
/// Points are stored in the deepest node whose bounding box still contains
/// them.  Subdividing stops when either
/// - the number of points in a node is ≤ `capacity`, or
/// - `max_depth` levels have already been created.
///
/// Construct with [`Octree::new`], insert obstacles with
/// [`Octree::insert`], then query with [`Octree::contains`] or
/// [`Octree::query_aabb`].
#[derive(Debug)]
pub struct Octree {
    root: OctreeNode,
    max_depth: usize,
}

impl Octree {
    /// Create an empty octree covering `bounds`.
    ///
    /// - `capacity` – maximum points per leaf before subdivision is attempted.
    pub fn new(bounds: Aabb, capacity: usize) -> Self {
        Self {
            root: OctreeNode::new(bounds, capacity),
            max_depth: 8,
        }
    }

    /// Create an empty octree with an explicit maximum subdivision depth.
    pub fn with_max_depth(bounds: Aabb, capacity: usize, max_depth: usize) -> Self {
        Self {
            root: OctreeNode::new(bounds, capacity),
            max_depth,
        }
    }

    /// Insert a point into the tree.
    ///
    /// Points outside the root bounding box are silently ignored.
    pub fn insert(&mut self, point: Point3) {
        self.root.insert(point, self.max_depth, 0);
    }

    /// Return the total number of points stored in the tree.
    pub fn len(&self) -> usize {
        self.root.count()
    }

    /// True when the tree contains no points.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// True when the tree contains a point equal to `p`.
    pub fn contains(&self, p: Point3) -> bool {
        self.root.contains(p)
    }

    /// True when any point in the tree lies inside `region`.
    ///
    /// Used for collision detection: pass the robot's swept volume (or any
    /// query box) to check whether any obstacle occupies that space.
    pub fn query_aabb(&self, region: &Aabb) -> bool {
        self.root.query_aabb(region)
    }

    /// Export all points currently stored in the tree.
    ///
    /// This is used for Octree map sharing: a robot serialises its spatial map
    /// as a flat list of points, broadcasts them over the fleet network, and
    /// peer robots call [`merge`][Self::merge] to fuse the data into their own
    /// trees.  Only the points are exported; the tree structure is not.
    pub fn export_points(&self) -> Vec<Point3> {
        let mut points = Vec::new();
        self.root.collect_points(&mut points);
        points
    }

    /// Merge a set of points (e.g. received from a peer robot's exported map)
    /// into this tree.
    ///
    /// Points that fall outside the tree's root bounding box are silently
    /// ignored, consistent with [`insert`][Self::insert].
    pub fn merge(&mut self, points: &[Point3]) {
        for &p in points {
            self.insert(p);
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// OctreeNode – internal implementation
// ────────────────────────────────────────────────────────────────────────────

#[derive(Debug)]
struct OctreeNode {
    bounds: Aabb,
    capacity: usize,
    /// Points stored at this node (only non-empty when the node is a leaf).
    points: Vec<Point3>,
    /// Eight children; `None` while this node is a leaf.
    children: Option<Box<[OctreeNode; 8]>>,
}

impl OctreeNode {
    fn new(bounds: Aabb, capacity: usize) -> Self {
        Self {
            bounds,
            capacity,
            points: Vec::new(),
            children: None,
        }
    }

    fn is_leaf(&self) -> bool {
        self.children.is_none()
    }

    fn count(&self) -> usize {
        if self.is_leaf() {
            self.points.len()
        } else if let Some(children) = &self.children {
            children.iter().map(|c| c.count()).sum()
        } else {
            unreachable!("non-leaf OctreeNode must have children")
        }
    }

    fn insert(&mut self, point: Point3, max_depth: usize, depth: usize) {
        if !self.bounds.contains_point(point) {
            return;
        }

        if self.is_leaf() {
            self.points.push(point);
            // Subdivide when over capacity and depth budget remains.
            if self.points.len() > self.capacity && depth < max_depth {
                self.subdivide(max_depth, depth);
            }
        } else if let Some(children) = self.children.as_mut() {
            for child in children.iter_mut() {
                if child.bounds.contains_point(point) {
                    child.insert(point, max_depth, depth + 1);
                    return;
                }
            }
        }
    }

    fn contains(&self, p: Point3) -> bool {
        if !self.bounds.contains_point(p) {
            return false;
        }
        if self.is_leaf() {
            self.points.contains(&p)
        } else if let Some(children) = &self.children {
            children.iter().any(|c| c.contains(p))
        } else {
            unreachable!("non-leaf OctreeNode must have children")
        }
    }

    fn query_aabb(&self, region: &Aabb) -> bool {
        if !self.bounds.overlaps(region) {
            return false;
        }
        if self.is_leaf() {
            self.points.iter().any(|p| region.contains_point(*p))
        } else if let Some(children) = &self.children {
            children.iter().any(|c| c.query_aabb(region))
        } else {
            unreachable!("non-leaf OctreeNode must have children")
        }
    }

    /// Collect all stored points into `out` (depth-first traversal).
    fn collect_points(&self, out: &mut Vec<Point3>) {
        if self.is_leaf() {
            out.extend_from_slice(&self.points);
        } else if let Some(children) = &self.children {
            for child in children.iter() {
                child.collect_points(out);
            }
        }
    }

    /// Split this leaf into eight children and redistribute existing points.
    fn subdivide(&mut self, max_depth: usize, depth: usize) {
        let c = self.bounds.centre();
        let min = self.bounds.min;
        let max = self.bounds.max;

        // Build the eight octant AABBs around the centre point.
        let octants = [
            Aabb::new(min, c),
            Aabb::new(Point3::new(c.x, min.y, min.z), Point3::new(max.x, c.y, c.z)),
            Aabb::new(Point3::new(min.x, c.y, min.z), Point3::new(c.x, max.y, c.z)),
            Aabb::new(Point3::new(c.x, c.y, min.z), Point3::new(max.x, max.y, c.z)),
            Aabb::new(Point3::new(min.x, min.y, c.z), Point3::new(c.x, c.y, max.z)),
            Aabb::new(Point3::new(c.x, min.y, c.z), Point3::new(max.x, c.y, max.z)),
            Aabb::new(Point3::new(min.x, c.y, c.z), Point3::new(c.x, max.y, max.z)),
            Aabb::new(c, max),
        ];

        let cap = self.capacity;
        let mut children = Box::new(octants.map(|b| OctreeNode::new(b, cap)));

        // Redistribute points that were in this leaf into the children.
        let points = std::mem::take(&mut self.points);
        for p in points {
            for child in children.iter_mut() {
                if child.bounds.contains_point(p) {
                    child.insert(p, max_depth, depth + 1);
                    break;
                }
            }
        }

        self.children = Some(children);
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_tree(capacity: usize) -> Octree {
        Octree::new(
            Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)),
            capacity,
        )
    }

    // ── Aabb ────────────────────────────────────────────────────────────────

    #[test]
    fn aabb_contains_interior_point() {
        let b = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
        assert!(b.contains_point(Point3::new(1.0, 1.0, 1.0)));
    }

    #[test]
    fn aabb_contains_boundary_point() {
        let b = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        assert!(b.contains_point(Point3::new(0.0, 0.0, 0.0)));
        assert!(b.contains_point(Point3::new(1.0, 1.0, 1.0)));
    }

    #[test]
    fn aabb_excludes_exterior_point() {
        let b = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        assert!(!b.contains_point(Point3::new(2.0, 0.0, 0.0)));
    }

    #[test]
    fn aabb_overlaps_touching_boxes() {
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let b = Aabb::new(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 1.0, 1.0));
        assert!(a.overlaps(&b));
    }

    #[test]
    fn aabb_no_overlap_separated_boxes() {
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let b = Aabb::new(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 1.0, 1.0));
        assert!(!a.overlaps(&b));
    }

    #[test]
    fn aabb_normalises_min_max() {
        let b = Aabb::new(Point3::new(2.0, 2.0, 2.0), Point3::new(0.0, 0.0, 0.0));
        assert_eq!(b.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(b.max, Point3::new(2.0, 2.0, 2.0));
    }

    // ── Octree – basic insert/query ──────────────────────────────────────────

    #[test]
    fn empty_tree_contains_nothing() {
        let tree = unit_tree(4);
        assert!(tree.is_empty());
        assert!(!tree.contains(Point3::new(0.5, 0.5, 0.5)));
    }

    #[test]
    fn insert_and_contains() {
        let mut tree = unit_tree(4);
        tree.insert(Point3::new(0.5, 0.5, 0.5));
        assert!(tree.contains(Point3::new(0.5, 0.5, 0.5)));
        assert!(!tree.contains(Point3::new(0.1, 0.1, 0.1)));
    }

    #[test]
    fn insert_outside_bounds_is_ignored() {
        let mut tree = unit_tree(4);
        tree.insert(Point3::new(5.0, 5.0, 5.0)); // outside [0,1]^3
        assert!(tree.is_empty());
    }

    #[test]
    fn len_tracks_insertions() {
        let mut tree = unit_tree(10);
        for i in 1..=5 {
            tree.insert(Point3::new(i as f32 * 0.1, 0.0, 0.0));
        }
        assert_eq!(tree.len(), 5);
    }

    // ── Octree – subdivision ─────────────────────────────────────────────────

    #[test]
    fn subdivision_preserves_all_points() {
        // capacity=2 → subdivision triggered on 3rd insert
        let mut tree = unit_tree(2);
        let pts = [
            Point3::new(0.1, 0.1, 0.1),
            Point3::new(0.9, 0.9, 0.9),
            Point3::new(0.2, 0.8, 0.3),
            Point3::new(0.7, 0.2, 0.6),
        ];
        for &p in &pts {
            tree.insert(p);
        }
        assert_eq!(tree.len(), 4);
        for &p in &pts {
            assert!(tree.contains(p), "missing {:?}", p);
        }
    }

    // ── Octree – collision / AABB query ─────────────────────────────────────

    #[test]
    fn query_aabb_detects_contained_point() {
        let mut tree = unit_tree(4);
        tree.insert(Point3::new(0.5, 0.5, 0.5));

        let probe = Aabb::new(Point3::new(0.4, 0.4, 0.4), Point3::new(0.6, 0.6, 0.6));
        assert!(tree.query_aabb(&probe));
    }

    #[test]
    fn query_aabb_misses_when_no_point_inside() {
        let mut tree = unit_tree(4);
        tree.insert(Point3::new(0.9, 0.9, 0.9));

        let probe = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(0.5, 0.5, 0.5));
        assert!(!tree.query_aabb(&probe));
    }

    #[test]
    fn query_aabb_path_clear_scenario() {
        // Simulate a corridor with an obstacle at (5, 0, 0).
        let bounds =
            Aabb::new(Point3::new(-10.0, -10.0, -10.0), Point3::new(10.0, 10.0, 10.0));
        let mut tree = Octree::new(bounds, 4);
        tree.insert(Point3::new(5.0, 0.0, 0.0)); // obstacle

        // Path box that does NOT include the obstacle.
        let clear_path =
            Aabb::new(Point3::new(0.0, -0.5, -0.5), Point3::new(4.0, 0.5, 0.5));
        assert!(!tree.query_aabb(&clear_path), "path should be clear");

        // Path box that DOES include the obstacle.
        let blocked_path =
            Aabb::new(Point3::new(0.0, -0.5, -0.5), Point3::new(6.0, 0.5, 0.5));
        assert!(tree.query_aabb(&blocked_path), "path should be blocked");
    }

    #[test]
    fn many_insertions_with_subdivision() {
        let bounds =
            Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(100.0, 100.0, 100.0));
        let mut tree = Octree::new(bounds, 4);

        // Insert a 5×5×5 grid of points.
        let mut count = 0usize;
        for ix in 0..5 {
            for iy in 0..5 {
                for iz in 0..5 {
                    tree.insert(Point3::new(
                        ix as f32 * 10.0 + 5.0,
                        iy as f32 * 10.0 + 5.0,
                        iz as f32 * 10.0 + 5.0,
                    ));
                    count += 1;
                }
            }
        }

        assert_eq!(tree.len(), count);
        assert!(tree.contains(Point3::new(5.0, 5.0, 5.0)));
        assert!(tree.contains(Point3::new(45.0, 45.0, 45.0)));
    }

    // ── export_points / merge ────────────────────────────────────────────────

    #[test]
    fn export_points_returns_all_inserted_points() {
        let mut tree = unit_tree(4);
        let pts = [
            Point3::new(0.1, 0.1, 0.1),
            Point3::new(0.9, 0.9, 0.9),
            Point3::new(0.5, 0.5, 0.5),
        ];
        for &p in &pts {
            tree.insert(p);
        }
        let exported = tree.export_points();
        assert_eq!(exported.len(), pts.len());
        for &p in &pts {
            assert!(exported.contains(&p), "exported must contain {:?}", p);
        }
    }

    #[test]
    fn export_points_empty_tree_returns_empty_vec() {
        let tree = unit_tree(4);
        assert!(tree.export_points().is_empty());
    }

    #[test]
    fn merge_fuses_peer_map_into_local_tree() {
        // Simulate Robot A's spatial map.
        let bounds =
            Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 10.0, 10.0));
        let mut robot_a = Octree::new(bounds, 4);
        robot_a.insert(Point3::new(1.0, 1.0, 1.0));
        robot_a.insert(Point3::new(2.0, 2.0, 2.0));

        // Robot B starts with its own local observation.
        let mut robot_b = Octree::new(bounds, 4);
        robot_b.insert(Point3::new(8.0, 8.0, 8.0));

        // Robot B merges Robot A's exported map.
        let peer_points = robot_a.export_points();
        robot_b.merge(&peer_points);

        // Robot B's tree now contains all three points.
        assert_eq!(robot_b.len(), 3);
        assert!(robot_b.contains(Point3::new(1.0, 1.0, 1.0)));
        assert!(robot_b.contains(Point3::new(2.0, 2.0, 2.0)));
        assert!(robot_b.contains(Point3::new(8.0, 8.0, 8.0)));
    }

    #[test]
    fn merge_ignores_out_of_bounds_points() {
        let bounds =
            Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let mut tree = Octree::new(bounds, 4);
        // Points from a peer with a larger world – those outside our bounds must be ignored.
        let peer_points = vec![
            Point3::new(0.5, 0.5, 0.5), // inside
            Point3::new(5.0, 5.0, 5.0), // outside
        ];
        tree.merge(&peer_points);
        assert_eq!(tree.len(), 1);
        assert!(tree.contains(Point3::new(0.5, 0.5, 0.5)));
    }
}

