//! Distributed Fleet Task Board.
//!
//! Provides a shared task queue that allows multiple robots in a fleet to
//! coordinate work without duplication.  When Robot A claims a task, it is
//! locked so that Robot B moves on to an unclaimed task instead.
//!
//! # Storage layout
//!
//! A single SQLite table `fleet_tasks` is created (if it does not already
//! exist) with the following columns:
//!
//! | column       | type | description                                         |
//! |--------------|------|-----------------------------------------------------|
//! | id           | TEXT | UUID v4 primary key                                 |
//! | title        | TEXT | Short human-readable task name                      |
//! | description  | TEXT | Full task description                               |
//! | status       | TEXT | One of `"open"`, `"claimed"`, `"completed"`         |
//! | claimed_by   | TEXT | Robot ID that holds the claim (NULL when unclaimed) |
//! | created_at   | TEXT | RFC-3339 creation timestamp (UTC)                   |
//! | updated_at   | TEXT | RFC-3339 last-update timestamp (UTC)                |
//!
//! # Example
//!
//! ```rust
//! use mechos_memory::task_board::TaskBoard;
//!
//! #[tokio::main(flavor = "current_thread")]
//! async fn main() {
//!     let board = TaskBoard::open_in_memory().unwrap();
//!
//!     let id = board.post("Move Box 1", "Move the red box from shelf A to shelf B.").await.unwrap();
//!
//!     // Robot Alpha claims the task.
//!     board.claim(&id, "robot_alpha").await.unwrap();
//!
//!     // Robot Beta sees no available tasks (all claimed).
//!     let available = board.list_available().await.unwrap();
//!     assert!(available.is_empty());
//!
//!     // Robot Alpha completes the task.
//!     board.complete(&id, "robot_alpha").await.unwrap();
//! }
//! ```

use chrono::Utc;
use rusqlite::{Connection, params};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use uuid::Uuid;

use std::sync::{Arc, Mutex};

// ─────────────────────────────────────────────────────────────────────────────
// Error type
// ─────────────────────────────────────────────────────────────────────────────

/// Errors that can arise from task board operations.
#[derive(Error, Debug)]
pub enum TaskBoardError {
    #[error("SQLite error: {0}")]
    Sqlite(#[from] rusqlite::Error),
    #[error("Task not found: {0}")]
    NotFound(String),
    #[error("Task is already claimed by another robot")]
    AlreadyClaimed,
    #[error("Task is not currently claimed by {0}")]
    NotClaimed(String),
    #[error("Task is already completed")]
    AlreadyCompleted,
    #[error("blocking task panicked: {0}")]
    TaskPanic(String),
}

// ─────────────────────────────────────────────────────────────────────────────
// TaskStatus
// ─────────────────────────────────────────────────────────────────────────────

/// The lifecycle state of a fleet task.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum TaskStatus {
    /// The task has been posted and is available to be claimed.
    Open,
    /// The task has been claimed by a robot and is in progress.
    Claimed,
    /// The task has been completed.
    Completed,
}

impl TaskStatus {
    fn as_str(&self) -> &'static str {
        match self {
            TaskStatus::Open => "open",
            TaskStatus::Claimed => "claimed",
            TaskStatus::Completed => "completed",
        }
    }

    fn from_str(s: &str) -> Option<Self> {
        match s {
            "open" => Some(TaskStatus::Open),
            "claimed" => Some(TaskStatus::Claimed),
            "completed" => Some(TaskStatus::Completed),
            _ => None,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TaskEntry
// ─────────────────────────────────────────────────────────────────────────────

/// A single task on the shared Fleet Task Board.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskEntry {
    /// Unique identifier for this task.
    pub id: String,
    /// Short human-readable task name (e.g. `"Move Box 1"`).
    pub title: String,
    /// Full description of what needs to be done.
    pub description: String,
    /// Current lifecycle state of the task.
    pub status: TaskStatus,
    /// The robot ID that has claimed this task, if any.
    pub claimed_by: Option<String>,
    /// RFC-3339 timestamp when the task was posted.
    pub created_at: String,
    /// RFC-3339 timestamp when the task was last updated.
    pub updated_at: String,
}

// ─────────────────────────────────────────────────────────────────────────────
// TaskBoard
// ─────────────────────────────────────────────────────────────────────────────

/// SQLite-backed shared task queue for fleet coordination.
///
/// Multiple robots in a fleet can share a single [`TaskBoard`] (backed by the
/// same SQLite file on a shared filesystem, or by an in-process in-memory
/// database for testing) to claim tasks without duplication.
#[derive(Clone)]
pub struct TaskBoard {
    conn: Arc<Mutex<Connection>>,
}

impl TaskBoard {
    /// Open (or create) a persistent SQLite task board at `path`.
    ///
    /// Enables WAL (Write-Ahead Logging) mode so that concurrent readers are
    /// not blocked by an active writer.
    pub fn open(path: &str) -> Result<Self, TaskBoardError> {
        let conn = Connection::open(path)?;
        conn.execute_batch("PRAGMA journal_mode=WAL;")?;
        let board = Self { conn: Arc::new(Mutex::new(conn)) };
        board.init_schema()?;
        Ok(board)
    }

    /// Open a temporary in-memory task board (useful for testing).
    pub fn open_in_memory() -> Result<Self, TaskBoardError> {
        let conn = Connection::open_in_memory()?;
        let board = Self { conn: Arc::new(Mutex::new(conn)) };
        board.init_schema()?;
        Ok(board)
    }

    fn init_schema(&self) -> Result<(), TaskBoardError> {
        let conn = self.conn.lock().unwrap_or_else(|e| e.into_inner());
        conn.execute_batch(
            "CREATE TABLE IF NOT EXISTS fleet_tasks (
                id          TEXT NOT NULL PRIMARY KEY,
                title       TEXT NOT NULL,
                description TEXT NOT NULL,
                status      TEXT NOT NULL DEFAULT 'open',
                claimed_by  TEXT,
                created_at  TEXT NOT NULL,
                updated_at  TEXT NOT NULL
            );",
        )?;
        Ok(())
    }

    /// Post a new task to the board and return its UUID.
    ///
    /// The task starts with [`TaskStatus::Open`] and is immediately available
    /// for any robot to claim.
    pub async fn post(&self, title: &str, description: &str) -> Result<String, TaskBoardError> {
        let conn = Arc::clone(&self.conn);
        let title = title.to_owned();
        let description = description.to_owned();
        tokio::task::spawn_blocking(move || {
            let conn = conn.lock().unwrap_or_else(|e| e.into_inner());
            let id = Uuid::new_v4().to_string();
            let now = Utc::now().to_rfc3339();
            let status = TaskStatus::Open.as_str();
            conn.execute(
                "INSERT INTO fleet_tasks (id, title, description, status, claimed_by, created_at, updated_at)
                 VALUES (?1, ?2, ?3, ?4, NULL, ?5, ?6)",
                params![id, title, description, status, now, now],
            )?;
            Ok(id)
        })
        .await
        .map_err(|e| TaskBoardError::TaskPanic(e.to_string()))?
    }

    /// Claim a task on behalf of `robot_id`.
    ///
    /// Returns [`TaskBoardError::AlreadyClaimed`] if another robot already
    /// holds the task, and [`TaskBoardError::AlreadyCompleted`] if the task
    /// has already been finished.
    pub async fn claim(&self, task_id: &str, robot_id: &str) -> Result<(), TaskBoardError> {
        let conn = Arc::clone(&self.conn);
        let task_id = task_id.to_owned();
        let robot_id = robot_id.to_owned();
        tokio::task::spawn_blocking(move || {
            let conn = conn.lock().unwrap_or_else(|e| e.into_inner());
            let entry = get_entry(&conn, &task_id)?;
            match entry.status {
                TaskStatus::Claimed => return Err(TaskBoardError::AlreadyClaimed),
                TaskStatus::Completed => return Err(TaskBoardError::AlreadyCompleted),
                TaskStatus::Open => {}
            }
            let now = Utc::now().to_rfc3339();
            let status = TaskStatus::Claimed.as_str();
            conn.execute(
                "UPDATE fleet_tasks SET status = ?1, claimed_by = ?2, updated_at = ?3
                 WHERE id = ?4",
                params![status, robot_id, now, task_id],
            )?;
            Ok(())
        })
        .await
        .map_err(|e| TaskBoardError::TaskPanic(e.to_string()))?
    }

    /// Mark a task as completed by `robot_id`.
    ///
    /// Returns [`TaskBoardError::NotClaimed`] if `robot_id` does not hold the
    /// claim, preventing a robot from completing another robot's task.
    pub async fn complete(&self, task_id: &str, robot_id: &str) -> Result<(), TaskBoardError> {
        let conn = Arc::clone(&self.conn);
        let task_id = task_id.to_owned();
        let robot_id = robot_id.to_owned();
        tokio::task::spawn_blocking(move || {
            let conn = conn.lock().unwrap_or_else(|e| e.into_inner());
            let entry = get_entry(&conn, &task_id)?;
            if entry.status == TaskStatus::Completed {
                return Err(TaskBoardError::AlreadyCompleted);
            }
            if entry.claimed_by.as_deref() != Some(&robot_id) {
                return Err(TaskBoardError::NotClaimed(robot_id));
            }
            let now = Utc::now().to_rfc3339();
            let status = TaskStatus::Completed.as_str();
            conn.execute(
                "UPDATE fleet_tasks SET status = ?1, updated_at = ?2 WHERE id = ?3",
                params![status, now, task_id],
            )?;
            Ok(())
        })
        .await
        .map_err(|e| TaskBoardError::TaskPanic(e.to_string()))?
    }

    /// Fetch a single task by its UUID.
    pub async fn get(&self, task_id: &str) -> Result<TaskEntry, TaskBoardError> {
        let conn = Arc::clone(&self.conn);
        let task_id = task_id.to_owned();
        tokio::task::spawn_blocking(move || {
            let conn = conn.lock().unwrap_or_else(|e| e.into_inner());
            get_entry(&conn, &task_id)
        })
        .await
        .map_err(|e| TaskBoardError::TaskPanic(e.to_string()))?
    }

    /// Return all tasks with [`TaskStatus::Open`], ordered by creation time
    /// (oldest first).
    pub async fn list_available(&self) -> Result<Vec<TaskEntry>, TaskBoardError> {
        self.list_by_status(TaskStatus::Open.as_str()).await
    }

    /// Return all tasks regardless of status, ordered by creation time.
    pub async fn list_all(&self) -> Result<Vec<TaskEntry>, TaskBoardError> {
        let conn = Arc::clone(&self.conn);
        tokio::task::spawn_blocking(move || {
            let conn = conn.lock().unwrap_or_else(|e| e.into_inner());
            let mut stmt = conn.prepare(
                "SELECT id, title, description, status, claimed_by, created_at, updated_at
                 FROM fleet_tasks ORDER BY created_at ASC",
            )?;
            let rows = stmt.query_map([], row_to_entry)?;
            rows.collect::<Result<Vec<_>, _>>().map_err(TaskBoardError::Sqlite)
        })
        .await
        .map_err(|e| TaskBoardError::TaskPanic(e.to_string()))?
    }

    async fn list_by_status(&self, status: &str) -> Result<Vec<TaskEntry>, TaskBoardError> {
        let conn = Arc::clone(&self.conn);
        let status = status.to_owned();
        tokio::task::spawn_blocking(move || {
            let conn = conn.lock().unwrap_or_else(|e| e.into_inner());
            let mut stmt = conn.prepare(
                "SELECT id, title, description, status, claimed_by, created_at, updated_at
                 FROM fleet_tasks WHERE status = ?1 ORDER BY created_at ASC",
            )?;
            let rows = stmt.query_map(params![status], row_to_entry)?;
            rows.collect::<Result<Vec<_>, _>>().map_err(TaskBoardError::Sqlite)
        })
        .await
        .map_err(|e| TaskBoardError::TaskPanic(e.to_string()))?
    }
}

fn get_entry(conn: &Connection, task_id: &str) -> Result<TaskEntry, TaskBoardError> {
    let mut stmt = conn.prepare(
        "SELECT id, title, description, status, claimed_by, created_at, updated_at
         FROM fleet_tasks WHERE id = ?1",
    )?;
    let mut rows = stmt.query_map(params![task_id], row_to_entry)?;
    rows.next()
        .ok_or_else(|| TaskBoardError::NotFound(task_id.to_string()))?
        .map_err(TaskBoardError::Sqlite)
}

fn row_to_entry(row: &rusqlite::Row<'_>) -> rusqlite::Result<TaskEntry> {
    let id: String = row.get(0)?;
    let title: String = row.get(1)?;
    let description: String = row.get(2)?;
    let status_str: String = row.get(3)?;
    let claimed_by: Option<String> = row.get(4)?;
    let created_at: String = row.get(5)?;
    let updated_at: String = row.get(6)?;
    let status = TaskStatus::from_str(&status_str).ok_or_else(|| {
        rusqlite::Error::InvalidColumnType(3, status_str, rusqlite::types::Type::Text)
    })?;
    Ok(TaskEntry {
        id,
        title,
        description,
        status,
        claimed_by,
        created_at,
        updated_at,
    })
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_board() -> TaskBoard {
        TaskBoard::open_in_memory().unwrap()
    }

    #[tokio::test]
    async fn post_creates_open_task() {
        let board = make_board();
        let id = board.post("Move Box 1", "Move the red box.").await.unwrap();
        let task = board.get(&id).await.unwrap();
        assert_eq!(task.title, "Move Box 1");
        assert_eq!(task.description, "Move the red box.");
        assert_eq!(task.status, TaskStatus::Open);
        assert!(task.claimed_by.is_none());
    }

    #[tokio::test]
    async fn claim_locks_task_for_robot() {
        let board = make_board();
        let id = board.post("Deliver Package", "Take package to room 5.").await.unwrap();
        board.claim(&id, "robot_alpha").await.unwrap();
        let task = board.get(&id).await.unwrap();
        assert_eq!(task.status, TaskStatus::Claimed);
        assert_eq!(task.claimed_by.as_deref(), Some("robot_alpha"));
    }

    #[tokio::test]
    async fn claim_by_second_robot_is_rejected() {
        let board = make_board();
        let id = board.post("Task A", "Do something.").await.unwrap();
        board.claim(&id, "robot_alpha").await.unwrap();
        let err = board.claim(&id, "robot_bravo").await.unwrap_err();
        assert!(matches!(err, TaskBoardError::AlreadyClaimed));
    }

    #[tokio::test]
    async fn complete_by_claimer_succeeds() {
        let board = make_board();
        let id = board.post("Task B", "Do another thing.").await.unwrap();
        board.claim(&id, "robot_alpha").await.unwrap();
        board.complete(&id, "robot_alpha").await.unwrap();
        let task = board.get(&id).await.unwrap();
        assert_eq!(task.status, TaskStatus::Completed);
    }

    #[tokio::test]
    async fn complete_by_non_claimer_is_rejected() {
        let board = make_board();
        let id = board.post("Task C", "Yet another task.").await.unwrap();
        board.claim(&id, "robot_alpha").await.unwrap();
        let err = board.complete(&id, "robot_bravo").await.unwrap_err();
        assert!(matches!(err, TaskBoardError::NotClaimed(_)));
    }

    #[tokio::test]
    async fn complete_already_completed_is_rejected() {
        let board = make_board();
        let id = board.post("Task D", "Do it once.").await.unwrap();
        board.claim(&id, "robot_alpha").await.unwrap();
        board.complete(&id, "robot_alpha").await.unwrap();
        let err = board.complete(&id, "robot_alpha").await.unwrap_err();
        assert!(matches!(err, TaskBoardError::AlreadyCompleted));
    }

    #[tokio::test]
    async fn list_available_returns_only_open_tasks() {
        let board = make_board();
        let id1 = board.post("Task 1", "Open task.").await.unwrap();
        let id2 = board.post("Task 2", "Will be claimed.").await.unwrap();
        board.claim(&id2, "robot_alpha").await.unwrap();
        let available = board.list_available().await.unwrap();
        assert_eq!(available.len(), 1);
        assert_eq!(available[0].id, id1);
    }

    #[tokio::test]
    async fn list_all_returns_all_tasks() {
        let board = make_board();
        board.post("T1", "desc1").await.unwrap();
        let id2 = board.post("T2", "desc2").await.unwrap();
        board.claim(&id2, "robot_alpha").await.unwrap();
        let all = board.list_all().await.unwrap();
        assert_eq!(all.len(), 2);
    }

    #[tokio::test]
    async fn get_nonexistent_task_returns_not_found() {
        let board = make_board();
        let err = board.get("nonexistent-id").await.unwrap_err();
        assert!(matches!(err, TaskBoardError::NotFound(_)));
    }

    #[tokio::test]
    async fn claim_completed_task_is_rejected() {
        let board = make_board();
        let id = board.post("Task E", "One time task.").await.unwrap();
        board.claim(&id, "robot_alpha").await.unwrap();
        board.complete(&id, "robot_alpha").await.unwrap();
        let err = board.claim(&id, "robot_bravo").await.unwrap_err();
        assert!(matches!(err, TaskBoardError::AlreadyCompleted));
    }

    #[tokio::test]
    async fn task_entry_serializes_to_json() {
        let board = make_board();
        let id = board.post("Serialization test", "Test JSON.").await.unwrap();
        let task = board.get(&id).await.unwrap();
        let json = serde_json::to_string(&task).unwrap();
        assert!(json.contains("Serialization test"));
        assert!(json.contains("open"));
    }
}
