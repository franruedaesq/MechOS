//! OpenTelemetry pipeline initialisation for MechOS.
//!
//! Call [`init_tracing`] once at process startup to wire up the `tracing`
//! subscriber with an optional OTLP span exporter.
//!
//! # Environment variables
//!
//! | Variable | Effect |
//! |---|---|
//! | `OTEL_EXPORTER_OTLP_ENDPOINT` | OTLP collector base URL (e.g. `http://localhost:4318`). When set the OTLP HTTP exporter is activated. |
//! | `RUST_LOG` | Log filter (default `"info"`). |
//! | `MECHOS_LOG_FORMAT=json` | Emit newline-delimited JSON logs. |
//!
//! # Example
//!
//! ```rust,no_run
//! // Hold the guard for the entire lifetime of the process.
//! let _guard = mechos_runtime::telemetry::init_tracing("mechos");
//! ```

use opentelemetry::trace::TracerProvider as _;
use opentelemetry_otlp::WithExportConfig;
use opentelemetry_sdk::{trace::SdkTracerProvider, Resource};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

/// Initialise the global `tracing` subscriber with an optional OTLP exporter.
///
/// When `OTEL_EXPORTER_OTLP_ENDPOINT` is set an OTLP/HTTP span exporter is
/// configured and all tracing spans (including those created with
/// `#[instrument]` in `mechos-kernel` and `mechos-hal`) are forwarded to the
/// collector.
///
/// When the env-var is absent the function falls back to a plain
/// `tracing-subscriber` console formatter without any OTel export.
///
/// The returned [`TracerProviderGuard`] **must** be held for the lifetime of
/// the process; dropping it flushes all pending span batches.
pub fn init_tracing(service_name: &str) -> TracerProviderGuard {
    let log_level = std::env::var("RUST_LOG").unwrap_or_else(|_| "info".to_string());
    let env_filter = EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| EnvFilter::new(log_level));
    let use_json = std::env::var("MECHOS_LOG_FORMAT").as_deref() == Ok("json");

    let provider = build_provider(service_name);

    if let Some(ref p) = provider {
        let tracer = p.tracer("mechos");
        let otel_layer = tracing_opentelemetry::layer().with_tracer(tracer);
        if use_json {
            tracing_subscriber::registry()
                .with(env_filter)
                .with(otel_layer)
                .with(tracing_subscriber::fmt::layer().json())
                .init();
        } else {
            tracing_subscriber::registry()
                .with(env_filter)
                .with(otel_layer)
                .with(tracing_subscriber::fmt::layer().compact())
                .init();
        }
    } else if use_json {
        tracing_subscriber::registry()
            .with(env_filter)
            .with(tracing_subscriber::fmt::layer().json())
            .init();
    } else {
        tracing_subscriber::registry()
            .with(env_filter)
            .with(tracing_subscriber::fmt::layer().compact())
            .init();
    }

    TracerProviderGuard(provider)
}

// ─────────────────────────────────────────────────────────────────────────────
// RAII guard
// ─────────────────────────────────────────────────────────────────────────────

/// RAII guard that shuts down the OTel [`SdkTracerProvider`] on drop.
///
/// Dropping this guard calls [`SdkTracerProvider::shutdown`], flushing all
/// pending spans before the process exits.  Hold an instance of this type
/// in `main` for the entire program lifetime.
pub struct TracerProviderGuard(Option<SdkTracerProvider>);

impl Drop for TracerProviderGuard {
    fn drop(&mut self) {
        if let Some(provider) = self.0.take() {
            if let Err(e) = provider.shutdown() {
                eprintln!("[mechos] OpenTelemetry provider shutdown error: {e}");
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Build an [`SdkTracerProvider`] when `OTEL_EXPORTER_OTLP_ENDPOINT` is set.
///
/// Returns `None` when the env-var is absent or the exporter cannot be
/// initialised (the error is printed to stderr and the caller falls back to
/// plain tracing-subscriber output).
fn build_provider(service_name: &str) -> Option<SdkTracerProvider> {
    let endpoint = std::env::var("OTEL_EXPORTER_OTLP_ENDPOINT").ok()?;

    let exporter = opentelemetry_otlp::SpanExporter::builder()
        .with_http()
        .with_endpoint(endpoint)
        .build()
        .map_err(|e| eprintln!("[mechos] OTLP exporter init failed: {e}"))
        .ok()?;

    let resource = Resource::builder()
        .with_service_name(service_name.to_string())
        .build();

    Some(
        SdkTracerProvider::builder()
            .with_resource(resource)
            // Use the simple (synchronous) exporter so that no Tokio runtime
            // needs to be running at init time.  The MechOS CLI creates its
            // Tokio runtime only after calling `init_tracing`, making a batch
            // exporter (which internally spawns tasks) unsafe to use here.
            .with_simple_exporter(exporter)
            .build(),
    )
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify that `build_provider` returns `None` when no endpoint is set.
    #[test]
    fn build_provider_returns_none_without_endpoint() {
        // Ensure the env-var is unset for this test.
        // SAFETY: single-threaded test; no other thread reads this env-var.
        unsafe { std::env::remove_var("OTEL_EXPORTER_OTLP_ENDPOINT") };
        assert!(
            build_provider("test-service").is_none(),
            "expected None when OTEL_EXPORTER_OTLP_ENDPOINT is absent"
        );
    }

    /// Verify that `TracerProviderGuard` drops without panicking when it holds
    /// no provider.
    #[test]
    fn tracer_provider_guard_drop_with_none_is_safe() {
        let guard = TracerProviderGuard(None);
        drop(guard); // must not panic
    }
}
