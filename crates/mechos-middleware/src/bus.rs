//! Headless, typed, topic-based publish/subscribe event bus.
//!
//! Uses [`tokio::sync::broadcast`] channels under the hood so that every
//! subscriber receives every message without any single subscriber blocking
//! the others.

use mechos_types::{Event, MechError};
use tokio::sync::broadcast;

/// Default channel capacity (number of buffered events before old ones are
/// dropped for slow subscribers).
const DEFAULT_CAPACITY: usize = 256;

/// Shared event bus. Clone it cheaply – all clones share the same underlying
/// broadcast channel.
#[derive(Clone, Debug)]
pub struct EventBus {
    sender: broadcast::Sender<Event>,
}

impl EventBus {
    /// Create a new bus with the given channel capacity.
    pub fn new(capacity: usize) -> Self {
        let (sender, _) = broadcast::channel(capacity);
        Self { sender }
    }

    /// Publish an event to all active subscribers.
    ///
    /// Returns the number of receivers that received the event, or a
    /// [`MechError::Serialization`] error if the channel is closed.
    pub fn publish(&self, event: Event) -> Result<usize, MechError> {
        self.sender.send(event).map_err(|e| {
            MechError::Serialization(format!("event bus send error: {e}"))
        })
    }

    /// Subscribe to all events on the bus.
    ///
    /// The caller should wrap the returned receiver with a [`TopicSubscriber`]
    /// to filter by source/topic, or consume it directly for every event.
    pub fn subscribe(&self) -> broadcast::Receiver<Event> {
        self.sender.subscribe()
    }

    /// Convenience: subscribe and return a [`TopicSubscriber`] filtered to
    /// the given topic prefix.
    pub fn subscribe_topic(&self, topic: impl Into<String>) -> TopicSubscriber {
        TopicSubscriber {
            topic: topic.into(),
            receiver: self.sender.subscribe(),
        }
    }
}

impl Default for EventBus {
    fn default() -> Self {
        Self::new(DEFAULT_CAPACITY)
    }
}

/// A subscriber that only delivers events whose `source` field starts with the
/// given topic prefix.
pub struct TopicSubscriber {
    topic: String,
    receiver: broadcast::Receiver<Event>,
}

impl TopicSubscriber {
    /// Wait for the next event that matches this subscriber's topic filter.
    ///
    /// Returns `None` when the bus is closed and no further events will arrive.
    pub async fn recv(&mut self) -> Option<Event> {
        loop {
            match self.receiver.recv().await {
                Ok(event) if event.source.starts_with(&self.topic) => {
                    return Some(event);
                }
                Ok(_) => continue,
                Err(broadcast::error::RecvError::Lagged(n)) => {
                    // Subscriber fell behind; log and continue.
                    eprintln!("[mechos-middleware] TopicSubscriber lagged by {n} events");
                    continue;
                }
                Err(broadcast::error::RecvError::Closed) => return None,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mechos_types::{EventPayload, TelemetryData};
    use uuid::Uuid;
    use chrono::Utc;

    fn make_event(source: &str) -> Event {
        Event {
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            source: source.to_string(),
            payload: EventPayload::Telemetry(TelemetryData {
                position_x: 1.0,
                position_y: 2.0,
                heading_rad: 0.0,
                battery_percent: 90,
            }),
        }
    }

    #[tokio::test]
    async fn publish_and_receive() {
        let bus = EventBus::default();
        let mut rx = bus.subscribe();

        let event = make_event("mechos-middleware::test");
        bus.publish(event.clone()).unwrap();

        let received = rx.recv().await.unwrap();
        assert_eq!(received.id, event.id);
        assert_eq!(received.source, event.source);
    }

    #[tokio::test]
    async fn topic_subscriber_filters() {
        let bus = EventBus::default();
        let mut sub = bus.subscribe_topic("sensor");

        // Publish an event that should NOT match.
        bus.publish(make_event("actuator::motor")).unwrap();
        // Publish an event that SHOULD match.
        let good = make_event("sensor/lidar");
        bus.publish(good.clone()).unwrap();

        let received = sub.recv().await.unwrap();
        assert_eq!(received.id, good.id);
    }

    #[tokio::test]
    async fn multiple_subscribers_receive_same_event() {
        let bus = EventBus::default();
        let mut rx1 = bus.subscribe();
        let mut rx2 = bus.subscribe();

        let event = make_event("ros2::odom");
        bus.publish(event.clone()).unwrap();

        assert_eq!(rx1.recv().await.unwrap().id, event.id);
        assert_eq!(rx2.recv().await.unwrap().id, event.id);
    }

    #[test]
    fn publish_no_subscribers_returns_error() {
        let bus = EventBus::default();
        // No active receivers – send returns Err.
        let result = bus.publish(make_event("test"));
        assert!(result.is_err());
    }
}
