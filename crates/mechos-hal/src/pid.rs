//! Generic PID (Proportional–Integral–Derivative) controller.
//!
//! The controller computes a corrective output that drives a measured value
//! toward a desired set-point.  It is deliberately hardware-agnostic: the
//! caller supplies the measurement and elapsed time, and receives the computed
//! output which it can apply to any actuator.
//!
//! # Example
//!
//! ```rust
//! use mechos_hal::pid::PidController;
//!
//! let mut pid = PidController::new(1.0, 0.1, 0.05);
//! pid.set_set_point(90.0); // target position in degrees or any unit
//!
//! let output = pid.update(0.0, 0.01); // measurement=0, dt=10 ms
//! assert!(output > 0.0); // controller drives measurement toward set-point
//! ```

/// A tunable PID controller for closed-loop feedback control.
///
/// Gains and output limits are configurable after construction via
/// [`PidController::set_gains`] and [`PidController::set_output_limits`].
#[derive(Debug, Clone)]
pub struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    set_point: f32,
    integral: f32,
    last_error: Option<f32>,
    output_min: f32,
    output_max: f32,
}

impl PidController {
    /// Create a new controller with the given gains.
    ///
    /// Output is unclamped by default (`f32::NEG_INFINITY` to `f32::INFINITY`).
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            set_point: 0.0,
            integral: 0.0,
            last_error: None,
            output_min: f32::NEG_INFINITY,
            output_max: f32::INFINITY,
        }
    }

    /// Update the proportional, integral, and derivative gains.
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Change the desired set-point value.
    pub fn set_set_point(&mut self, set_point: f32) {
        self.set_point = set_point;
    }

    /// Return the current set-point.
    pub fn set_point(&self) -> f32 {
        self.set_point
    }

    /// Clamp the controller output to `[min, max]`.
    ///
    /// Integral wind-up is also clamped to this range.
    pub fn set_output_limits(&mut self, min: f32, max: f32) {
        self.output_min = min;
        self.output_max = max;
    }

    /// Compute the next controller output.
    ///
    /// - `measurement` – the current measured value of the process variable.
    /// - `dt` – elapsed time since the last call, in seconds (must be > 0).
    ///
    /// Returns the clamped control output.  Returns `0.0` without updating
    /// internal state if `dt` is not positive.
    pub fn update(&mut self, measurement: f32, dt: f32) -> f32 {
        if dt <= 0.0 {
            return 0.0;
        }

        let error = self.set_point - measurement;

        // Proportional term.
        let p = self.kp * error;

        // Integral term with anti-windup clamping.
        self.integral += error * dt;
        let i_raw = self.ki * self.integral;
        let i = i_raw.clamp(self.output_min, self.output_max);
        // Back-calculate integral to prevent wind-up beyond limits.
        if self.ki.abs() > f32::EPSILON {
            self.integral = i / self.ki;
        }

        // Derivative term (backward difference).
        let d = match self.last_error {
            Some(prev) => self.kd * (error - prev) / dt,
            None => 0.0,
        };
        self.last_error = Some(error);

        (p + i + d).clamp(self.output_min, self.output_max)
    }

    /// Reset internal state (integral accumulator and derivative memory).
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proportional_only_drives_toward_set_point() {
        let mut pid = PidController::new(2.0, 0.0, 0.0);
        pid.set_set_point(10.0);

        // error = 10.0 - 0.0 = 10.0 → output = 2.0 * 10.0 = 20.0
        let output = pid.update(0.0, 0.1);
        assert!((output - 20.0).abs() < 1e-4);
    }

    #[test]
    fn output_is_zero_at_set_point() {
        let mut pid = PidController::new(1.0, 0.0, 0.0);
        pid.set_set_point(5.0);
        let output = pid.update(5.0, 0.1);
        assert!(output.abs() < 1e-6);
    }

    #[test]
    fn output_clamped_to_limits() {
        let mut pid = PidController::new(100.0, 0.0, 0.0);
        pid.set_set_point(1.0);
        pid.set_output_limits(-1.0, 1.0);

        let output = pid.update(0.0, 0.01);
        assert!(output <= 1.0);
        assert!(output >= -1.0);
    }

    #[test]
    fn integral_accumulates_over_time() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        pid.set_set_point(1.0);

        // After two steps of dt=0.5 with constant error=1:
        // integral ≈ 0.5 + 0.5 = 1.0 → output ≈ 1.0 * 1.0 = 1.0
        pid.update(1.0, 0.5); // error = 0
        // reset and try with constant nonzero error
        let mut pid2 = PidController::new(0.0, 1.0, 0.0);
        pid2.set_set_point(2.0);
        pid2.update(1.0, 0.5); // integral += 1.0 * 0.5 = 0.5
        let out = pid2.update(1.0, 0.5); // integral += 0.5 → 1.0, output = 1.0
        assert!((out - 1.0).abs() < 1e-4);
    }

    #[test]
    fn reset_clears_state() {
        let mut pid = PidController::new(1.0, 1.0, 1.0);
        pid.set_set_point(5.0);
        pid.update(0.0, 0.1);
        pid.reset();

        // After reset, derivative term is absent (treated as first call).
        // integral is also zeroed.
        let out_after_reset = pid.update(0.0, 0.1);
        let mut fresh = PidController::new(1.0, 1.0, 1.0);
        fresh.set_set_point(5.0);
        let out_fresh = fresh.update(0.0, 0.1);
        assert!((out_after_reset - out_fresh).abs() < 1e-6);
    }

    #[test]
    fn set_gains_updates_behavior() {
        let mut pid = PidController::new(1.0, 0.0, 0.0);
        pid.set_set_point(10.0);
        pid.set_gains(3.0, 0.0, 0.0);
        let output = pid.update(0.0, 0.1);
        // error = 10.0 → output = 3.0 * 10.0 = 30.0
        assert!((output - 30.0).abs() < 1e-4);
    }

    #[test]
    fn non_positive_dt_returns_zero_without_side_effects() {
        let mut pid = PidController::new(1.0, 1.0, 1.0);
        pid.set_set_point(5.0);
        // A non-positive dt must return 0 and leave state unchanged.
        assert_eq!(pid.update(0.0, 0.0), 0.0);
        assert_eq!(pid.update(0.0, -0.1), 0.0);
        // After the no-op calls, a valid update should behave as if first call.
        let mut fresh = PidController::new(1.0, 1.0, 1.0);
        fresh.set_set_point(5.0);
        assert!((pid.update(0.0, 0.1) - fresh.update(0.0, 0.1)).abs() < 1e-6);
    }
}
