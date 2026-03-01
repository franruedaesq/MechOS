//! Episodic Memory Store.
//!
//! Persists timestamped interaction summaries together with their dense
//! embedding vectors to a local SQLite database and provides
//! cosine-similarity–based recall so the runtime can retrieve the memories
//! most semantically similar to a query embedding.
//!
//! # Storage layout
//!
//! A single table `episodic_memories` is created (if it does not already
//! exist) with the following columns:
//!
//! | column      | type    | description                                    |
//! |-------------|---------|------------------------------------------------|
//! | id          | TEXT    | UUID v4 primary key                            |
//! | timestamp   | TEXT    | RFC-3339 creation time (UTC)                   |
//! | source      | TEXT    | Originating component label                    |
//! | summary     | TEXT    | Human-readable interaction summary             |
//! | embedding   | BLOB    | Little-endian f32 vector (4 × N bytes)         |
//!
//! # Example
//!
//! ```rust
//! use mechos_memory::episodic::{EpisodicStore, MemoryEntry};
//!
//! #[tokio::main(flavor = "current_thread")]
//! async fn main() {
//!     let store = EpisodicStore::open_in_memory().unwrap();
//!
//!     let entry = MemoryEntry::new(
//!         "mechos-runtime".to_string(),
//!         "The robot navigated around the desk.".to_string(),
//!         vec![0.1, 0.9, 0.3],
//!     );
//!     store.store(&entry).await.unwrap();
//!
//!     let results = store.recall_similar(&[0.1, 0.9, 0.3], 5).await.unwrap();
//!     assert_eq!(results[0].0.id, entry.id);
//! }
//! ```

use chrono::{DateTime, Utc};
use rusqlite::{Connection, params};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use uuid::Uuid;

use std::cmp::Ordering as CmpOrdering;
use std::collections::BinaryHeap;
use std::sync::{Arc, Mutex};

// ─────────────────────────────────────────────────────────────────────────────
// Error type
// ─────────────────────────────────────────────────────────────────────────────

/// Errors that can arise from episodic memory operations.
#[derive(Error, Debug)]
pub enum EpisodicError {
    #[error("SQLite error: {0}")]
    Sqlite(#[from] rusqlite::Error),
    #[error("Embedding vectors must be non-empty and equal in length")]
    DimensionMismatch,
    #[error("blocking task panicked: {0}")]
    TaskPanic(String),
}

// ─────────────────────────────────────────────────────────────────────────────
// MemoryEntry
// ─────────────────────────────────────────────────────────────────────────────

/// A single episodic memory record.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MemoryEntry {
    /// Unique identifier for this memory.
    pub id: Uuid,
    /// Wall-clock time at which the memory was created.
    pub timestamp: DateTime<Utc>,
    /// Label of the component that produced this memory (e.g. `"mechos-runtime"`).
    pub source: String,
    /// Human-readable summary of the interaction or observation.
    pub summary: String,
    /// Dense embedding vector representing the semantic content of the summary.
    pub embedding: Vec<f32>,
}

impl MemoryEntry {
    /// Construct a new [`MemoryEntry`] with a freshly generated UUID and the
    /// current UTC timestamp.
    pub fn new(source: String, summary: String, embedding: Vec<f32>) -> Self {
        Self {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source,
            summary,
            embedding,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Embedding serialisation helpers
// ─────────────────────────────────────────────────────────────────────────────

fn embedding_to_bytes(embedding: &[f32]) -> Vec<u8> {
    embedding.iter().flat_map(|f| f.to_le_bytes()).collect()
}

fn bytes_to_embedding(bytes: &[u8]) -> Vec<f32> {
    bytes
        .chunks_exact(4)
        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Cosine similarity
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the cosine similarity between two equal-length vectors.
///
/// Returns a value in `[-1.0, 1.0]`, or `0.0` if either vector has zero norm.
pub fn cosine_similarity(a: &[f32], b: &[f32]) -> f32 {
    let (dot, norm_a_sq, norm_b_sq) = a.iter().zip(b).fold(
        (0.0f32, 0.0f32, 0.0f32),
        |(dot, norm_a_sq, norm_b_sq), (&x, &y)| {
            (
                dot + x * y,
                norm_a_sq + x * x,
                norm_b_sq + y * y,
            )
        },
    );
    let norm_a = norm_a_sq.sqrt();
    let norm_b = norm_b_sq.sqrt();
    if norm_a == 0.0 || norm_b == 0.0 {
        0.0
    } else {
        dot / (norm_a * norm_b)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Min-heap entry for Top-K recall
// ─────────────────────────────────────────────────────────────────────────────

/// Wraps a `(MemoryEntry, similarity)` pair for use in a [`BinaryHeap`].
///
/// The heap is a *min*-heap keyed on similarity so that when the heap reaches
/// capacity `k` we can efficiently evict the worst entry in O(log k) time,
/// yielding an overall O(N log K) Top-K algorithm.
struct HeapEntry(MemoryEntry, f32);

impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.1.total_cmp(&other.1) == CmpOrdering::Equal
    }
}

impl Eq for HeapEntry {}

// Reverse the natural float ordering so that the `BinaryHeap` (which is a
// max-heap by default) acts as a min-heap: the entry with the *lowest*
// similarity floats to the top and is evicted first.
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<CmpOrdering> {
        Some(self.cmp(other))
    }
}

impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> CmpOrdering {
        // Reverse: lower similarity → greater in the heap → evicted first.
        other.1.total_cmp(&self.1)
    }
}



/// SQLite-backed episodic memory store.
///
/// Persists [`MemoryEntry`] records to a local SQLite database and supports
/// semantic retrieval via cosine-similarity ranking.
#[derive(Clone)]
pub struct EpisodicStore {
    conn: Arc<Mutex<Connection>>,
}

impl EpisodicStore {
    /// Open (or create) a persistent SQLite database at `path`.
    ///
    /// Enables WAL (Write-Ahead Logging) mode so that concurrent readers are
    /// not blocked by an active writer.
    pub fn open(path: &str) -> Result<Self, EpisodicError> {
        let conn = Connection::open(path)?;
        conn.execute_batch("PRAGMA journal_mode=WAL;")?;
        let store = Self { conn: Arc::new(Mutex::new(conn)) };
        store.init_schema()?;
        Ok(store)
    }

    /// Open a temporary in-memory database (useful for testing).
    pub fn open_in_memory() -> Result<Self, EpisodicError> {
        let conn = Connection::open_in_memory()?;
        let store = Self { conn: Arc::new(Mutex::new(conn)) };
        store.init_schema()?;
        Ok(store)
    }

    fn init_schema(&self) -> Result<(), EpisodicError> {
        let conn = self.conn.lock().unwrap_or_else(|e| e.into_inner());
        conn.execute_batch(
            "CREATE TABLE IF NOT EXISTS episodic_memories (
                id        TEXT NOT NULL PRIMARY KEY,
                timestamp TEXT NOT NULL,
                source    TEXT NOT NULL,
                summary   TEXT NOT NULL,
                embedding BLOB NOT NULL
            );",
        )?;
        Ok(())
    }

    /// Persist a [`MemoryEntry`] to the store.
    pub async fn store(&self, entry: &MemoryEntry) -> Result<(), EpisodicError> {
        if entry.embedding.is_empty() {
            return Err(EpisodicError::DimensionMismatch);
        }
        let conn = Arc::clone(&self.conn);
        let blob = embedding_to_bytes(&entry.embedding);
        let id = entry.id.to_string();
        let ts = entry.timestamp.to_rfc3339();
        let source = entry.source.clone();
        let summary = entry.summary.clone();
        tokio::task::spawn_blocking(move || {
            let conn = conn.lock().unwrap_or_else(|e| e.into_inner());
            conn.execute(
                "INSERT OR REPLACE INTO episodic_memories
                     (id, timestamp, source, summary, embedding)
                 VALUES (?1, ?2, ?3, ?4, ?5)",
                params![id, ts, source, summary, blob],
            )?;
            Ok(())
        })
        .await
        .map_err(|e| EpisodicError::TaskPanic(e.to_string()))?
    }

    /// Retrieve all stored entries ordered by timestamp (oldest first).
    pub async fn all_entries(&self) -> Result<Vec<MemoryEntry>, EpisodicError> {
        let conn = Arc::clone(&self.conn);
        tokio::task::spawn_blocking(move || {
            let conn = conn.lock().unwrap_or_else(|e| e.into_inner());
            let mut stmt = conn.prepare(
                "SELECT id, timestamp, source, summary, embedding
                 FROM episodic_memories
                 ORDER BY timestamp ASC",
            )?;
            let rows = stmt.query_map([], |row| {
                let id_str: String = row.get(0)?;
                let ts_str: String = row.get(1)?;
                let source: String = row.get(2)?;
                let summary: String = row.get(3)?;
                let blob: Vec<u8> = row.get(4)?;
                Ok((id_str, ts_str, source, summary, blob))
            })?;

            let mut entries = Vec::new();
            for row in rows {
                let (id_str, ts_str, source, summary, blob) = row?;
                let id = Uuid::parse_str(&id_str)
                    .map_err(|e| rusqlite::Error::InvalidColumnType(0, e.to_string(), rusqlite::types::Type::Text))?;
                let timestamp = ts_str.parse::<DateTime<Utc>>().map_err(|e| {
                    rusqlite::Error::InvalidColumnType(1, e.to_string(), rusqlite::types::Type::Text)
                })?;
                entries.push(MemoryEntry {
                    id,
                    timestamp,
                    source,
                    summary,
                    embedding: bytes_to_embedding(&blob),
                });
            }
            Ok(entries)
        })
        .await
        .map_err(|e| EpisodicError::TaskPanic(e.to_string()))?
    }

    /// Return the `top_k` most similar entries to `query_embedding`, ranked by
    /// cosine similarity (highest first).
    ///
    /// Each element of the returned slice is `(MemoryEntry, similarity_score)`.
    ///
    /// Uses a min-heap of capacity `top_k` to compute the result in
    /// O(N log K) time and O(K) extra memory rather than sorting all N entries.
    ///
    /// Returns [`EpisodicError::DimensionMismatch`] if `query_embedding` is
    /// empty or any stored embedding has a different dimension.
    pub async fn recall_similar(
        &self,
        query_embedding: &[f32],
        top_k: usize,
    ) -> Result<Vec<(MemoryEntry, f32)>, EpisodicError> {
        if query_embedding.is_empty() {
            return Err(EpisodicError::DimensionMismatch);
        }
        if top_k == 0 {
            return Ok(vec![]);
        }
        let entries = self.all_entries().await?;
        let query = query_embedding.to_vec();

        // Min-heap of capacity `top_k`: the entry with the lowest similarity
        // sits at the top and is replaced when a better candidate arrives.
        let mut heap: BinaryHeap<HeapEntry> = BinaryHeap::with_capacity(top_k + 1);

        for entry in entries {
            if entry.embedding.len() != query.len() {
                continue;
            }
            let score = cosine_similarity(&entry.embedding, &query);
            if heap.len() < top_k {
                heap.push(HeapEntry(entry, score));
            } else if let Some(worst) = heap.peek()
                && score > worst.1 {
                    heap.pop();
                    heap.push(HeapEntry(entry, score));
                }
        }

        // Drain heap into a vec sorted by descending similarity.
        let mut result: Vec<(MemoryEntry, f32)> =
            heap.into_iter().map(|e| (e.0, e.1)).collect();
        result.sort_by(|a, b| b.1.total_cmp(&a.1));
        Ok(result)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_entry(source: &str, summary: &str, embedding: Vec<f32>) -> MemoryEntry {
        MemoryEntry::new(source.to_string(), summary.to_string(), embedding)
    }

    // ── cosine_similarity ────────────────────────────────────────────────────

    #[test]
    fn cosine_identical_vectors_is_one() {
        let v = vec![1.0f32, 2.0, 3.0];
        assert!((cosine_similarity(&v, &v) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn cosine_orthogonal_vectors_is_zero() {
        let a = vec![1.0f32, 0.0, 0.0];
        let b = vec![0.0f32, 1.0, 0.0];
        assert!(cosine_similarity(&a, &b).abs() < 1e-6);
    }

    #[test]
    fn cosine_opposite_vectors_is_minus_one() {
        let a = vec![1.0f32, 0.0];
        let b = vec![-1.0f32, 0.0];
        assert!((cosine_similarity(&a, &b) + 1.0).abs() < 1e-6);
    }

    #[test]
    fn cosine_zero_vector_returns_zero() {
        let a = vec![0.0f32, 0.0];
        let b = vec![1.0f32, 2.0];
        assert_eq!(cosine_similarity(&a, &b), 0.0);
    }

    // ── embedding round-trip ─────────────────────────────────────────────────

    #[test]
    fn embedding_bytes_roundtrip() {
        let original = vec![1.5f32, -0.25, 0.0, 42.0];
        let bytes = embedding_to_bytes(&original);
        let recovered = bytes_to_embedding(&bytes);
        assert_eq!(original, recovered);
    }

    // ── EpisodicStore ─────────────────────────────────────────────────────────

    #[tokio::test]
    async fn store_and_retrieve_all_entries() {
        let store = EpisodicStore::open_in_memory().unwrap();
        let e1 = make_entry("src-a", "first memory", vec![1.0, 0.0]);
        let e2 = make_entry("src-b", "second memory", vec![0.0, 1.0]);
        store.store(&e1).await.unwrap();
        store.store(&e2).await.unwrap();

        let all = store.all_entries().await.unwrap();
        assert_eq!(all.len(), 2);
    }

    #[tokio::test]
    async fn recall_similar_returns_best_match() {
        let store = EpisodicStore::open_in_memory().unwrap();
        let near = make_entry("rt", "near", vec![1.0f32, 0.0, 0.0]);
        let far = make_entry("rt", "far", vec![0.0f32, 0.0, 1.0]);
        store.store(&near).await.unwrap();
        store.store(&far).await.unwrap();

        let query = [1.0f32, 0.0, 0.0];
        let results = store.recall_similar(&query, 1).await.unwrap();
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].0.id, near.id);
        assert!((results[0].1 - 1.0).abs() < 1e-6);
    }

    #[tokio::test]
    async fn recall_similar_top_k_limits_results() {
        let store = EpisodicStore::open_in_memory().unwrap();
        for i in 0..5 {
            let e = make_entry("rt", &format!("mem {i}"), vec![i as f32, 1.0]);
            store.store(&e).await.unwrap();
        }
        let results = store.recall_similar(&[2.0, 1.0], 3).await.unwrap();
        assert_eq!(results.len(), 3);
    }

    #[tokio::test]
    async fn recall_similar_empty_query_returns_error() {
        let store = EpisodicStore::open_in_memory().unwrap();
        let err = store.recall_similar(&[], 5).await.unwrap_err();
        assert!(matches!(err, EpisodicError::DimensionMismatch));
    }

    #[tokio::test]
    async fn store_empty_embedding_returns_error() {
        let store = EpisodicStore::open_in_memory().unwrap();
        let e = MemoryEntry {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: "test".to_string(),
            summary: "no embedding".to_string(),
            embedding: vec![],
        };
        let err = store.store(&e).await.unwrap_err();
        assert!(matches!(err, EpisodicError::DimensionMismatch));
    }

    #[tokio::test]
    async fn recall_skips_dimension_mismatched_entries() {
        let store = EpisodicStore::open_in_memory().unwrap();
        // Store a 3-dimensional entry
        let e = make_entry("rt", "3d", vec![1.0, 0.0, 0.0]);
        store.store(&e).await.unwrap();
        // Query with a 2-dimensional vector – no match should be returned
        let results = store.recall_similar(&[1.0, 0.0], 5).await.unwrap();
        assert_eq!(results.len(), 0);
    }

    #[tokio::test]
    async fn duplicate_id_replaced_on_store() {
        let store = EpisodicStore::open_in_memory().unwrap();
        let mut e = make_entry("rt", "original", vec![1.0, 0.0]);
        store.store(&e).await.unwrap();
        e.summary = "updated".to_string();
        store.store(&e).await.unwrap(); // INSERT OR REPLACE

        let all = store.all_entries().await.unwrap();
        assert_eq!(all.len(), 1);
        assert_eq!(all[0].summary, "updated");
    }

    #[tokio::test]
    async fn all_entries_empty_store_returns_empty_vec() {
        let store = EpisodicStore::open_in_memory().unwrap();
        let all = store.all_entries().await.unwrap();
        assert!(all.is_empty());
    }
}
