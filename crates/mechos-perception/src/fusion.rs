//! Sensor Fusion Engine.
//!
//! Combines heterogeneous sensor data streams (Odometry + IMU) into a single,
//! unified [`FusedState`] estimate using a complementary filter.
//!
//! The filter blends:
//! - **Odometry** – absolute position and heading derived from wheel encoders
//!   or similar dead-reckoning; low-frequency, globally consistent but subject
//!   to drift.
//! - **IMU** – gyroscope angular velocity; high-frequency and locally accurate
//!   but unbounded drift over time.
//!
//! The complementary filter formula for heading is:
//! ```text
//! heading_fused = α * (heading_odom + ω_imu * dt) + (1 − α) * heading_odom
//! ```
//! where α ∈ [0, 1] controls how much the IMU integration is trusted.
//!
//! # Example
//!
//! ```rust
//! use mechos_perception::fusion::{SensorFusion, OdometryData, ImuData};
//!
//! let mut fusion = SensorFusion::new(0.98);
//!
//! fusion.update_odometry(OdometryData {
//!     position_x: 1.0, position_y: 0.0,
//!     heading_rad: 0.1,
//!     velocity_x: 0.5, velocity_y: 0.0,
//! });
//!
//! fusion.update_imu(ImuData {
//!     angular_velocity_z: 0.2,
//!     linear_accel_x: 0.1,
//!     linear_accel_y: 0.0,
//! });
//!
//! let state = fusion.fused_state(0.01);
//! assert!((state.position_x - 1.0).abs() < 1e-5);
//! ```

// ────────────────────────────────────────────────────────────────────────────
// Input types
// ────────────────────────────────────────────────────────────────────────────

/// A single odometry measurement (e.g. from wheel encoders or visual
/// odometry).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OdometryData {
    /// Robot X position in the world frame (metres).
    pub position_x: f32,
    /// Robot Y position in the world frame (metres).
    pub position_y: f32,
    /// Robot heading angle, measured counter-clockwise from +X (radians).
    pub heading_rad: f32,
    /// Linear velocity along the robot's X axis (m/s).
    pub velocity_x: f32,
    /// Linear velocity along the robot's Y axis (m/s).
    pub velocity_y: f32,
}

/// A single IMU measurement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuData {
    /// Angular velocity around the vertical (Z) axis (rad/s).
    pub angular_velocity_z: f32,
    /// Linear acceleration along the robot's X axis (m/s²).
    pub linear_accel_x: f32,
    /// Linear acceleration along the robot's Y axis (m/s²).
    pub linear_accel_y: f32,
}

// ────────────────────────────────────────────────────────────────────────────
// Output type
// ────────────────────────────────────────────────────────────────────────────

/// The fused state estimate produced by [`SensorFusion`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FusedState {
    /// Estimated X position in the world frame (metres).
    pub position_x: f32,
    /// Estimated Y position in the world frame (metres).
    pub position_y: f32,
    /// Fused heading angle (radians).
    pub heading_rad: f32,
    /// Estimated linear velocity along the robot's X axis (m/s).
    pub velocity_x: f32,
    /// Estimated linear velocity along the robot's Y axis (m/s).
    pub velocity_y: f32,
}

// ────────────────────────────────────────────────────────────────────────────
// SensorFusion
// ────────────────────────────────────────────────────────────────────────────

/// Complementary filter that fuses [`OdometryData`] and [`ImuData`] into a
/// single [`FusedState`].
///
/// Construct with [`SensorFusion::new`], feed measurements via
/// [`SensorFusion::update_odometry`] and [`SensorFusion::update_imu`], then
/// call [`SensorFusion::fused_state`] with the elapsed time `dt` to obtain
/// the current estimate.
#[derive(Debug)]
pub struct SensorFusion {
    /// Complementary filter coefficient (0–1).  Higher values trust the IMU
    /// gyroscope more for heading estimation.
    alpha: f32,
    last_odometry: Option<OdometryData>,
    last_imu: Option<ImuData>,
}

impl SensorFusion {
    /// Create a new fusion engine.
    ///
    /// `alpha` is the complementary filter coefficient (clamped to `[0, 1]`).
    /// A value of `0.98` is typical for fusing a slow odometry update with a
    /// 100 Hz IMU.
    pub fn new(alpha: f32) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            last_odometry: None,
            last_imu: None,
        }
    }

    /// Feed a new odometry measurement into the filter.
    pub fn update_odometry(&mut self, data: OdometryData) {
        self.last_odometry = Some(data);
    }

    /// Feed a new IMU measurement into the filter.
    pub fn update_imu(&mut self, data: ImuData) {
        self.last_imu = Some(data);
    }

    /// Compute the current fused state estimate.
    ///
    /// `dt` is the time elapsed since the last call (seconds, must be ≥ 0).
    ///
    /// - Position and velocity are taken directly from the most recent
    ///   odometry reading (or zero if none has been received yet).
    /// - Heading is blended: the IMU-integrated heading prediction
    ///   (`heading_odom + ω * dt`) is weighted by `alpha`; the raw odometry
    ///   heading is weighted by `(1 − alpha)`.
    pub fn fused_state(&self, dt: f32) -> FusedState {
        let dt = dt.max(0.0);

        let (pos_x, pos_y, odom_heading, vel_x, vel_y) = match &self.last_odometry {
            Some(o) => (o.position_x, o.position_y, o.heading_rad, o.velocity_x, o.velocity_y),
            None => (0.0, 0.0, 0.0, 0.0, 0.0),
        };

        let heading = match &self.last_imu {
            Some(imu) => {
                let imu_predicted = odom_heading + imu.angular_velocity_z * dt;
                self.alpha * imu_predicted + (1.0 - self.alpha) * odom_heading
            }
            None => odom_heading,
        };

        FusedState {
            position_x: pos_x,
            position_y: pos_y,
            heading_rad: heading,
            velocity_x: vel_x,
            velocity_y: vel_y,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn odom(px: f32, py: f32, h: f32) -> OdometryData {
        OdometryData {
            position_x: px,
            position_y: py,
            heading_rad: h,
            velocity_x: 0.0,
            velocity_y: 0.0,
        }
    }

    fn imu(wz: f32) -> ImuData {
        ImuData {
            angular_velocity_z: wz,
            linear_accel_x: 0.0,
            linear_accel_y: 0.0,
        }
    }

    #[test]
    fn no_measurements_returns_zero_state() {
        let fusion = SensorFusion::new(0.98);
        let state = fusion.fused_state(0.01);
        assert_eq!(state.position_x, 0.0);
        assert_eq!(state.heading_rad, 0.0);
    }

    #[test]
    fn odometry_only_passes_through() {
        let mut fusion = SensorFusion::new(0.98);
        fusion.update_odometry(odom(3.0, 4.0, 1.0));

        let state = fusion.fused_state(0.01);
        assert!((state.position_x - 3.0).abs() < 1e-5);
        assert!((state.position_y - 4.0).abs() < 1e-5);
        // No IMU → heading comes purely from odometry.
        assert!((state.heading_rad - 1.0).abs() < 1e-5);
    }

    #[test]
    fn imu_only_returns_zero_position() {
        let mut fusion = SensorFusion::new(0.98);
        fusion.update_imu(imu(1.0));

        let state = fusion.fused_state(0.01);
        assert_eq!(state.position_x, 0.0);
        // No odometry → odom_heading = 0, IMU predicts 0 + 1.0 * 0.01 = 0.01.
        // Fused: 0.98 * 0.01 + 0.02 * 0 = 0.0098.
        assert!((state.heading_rad - 0.0098).abs() < 1e-5);
    }

    #[test]
    fn fused_heading_blends_imu_and_odom() {
        let mut fusion = SensorFusion::new(0.5);
        fusion.update_odometry(odom(0.0, 0.0, 1.0)); // heading = 1.0 rad
        fusion.update_imu(imu(0.0)); // zero angular velocity

        // With ω=0 and dt=0.1: imu_predicted = 1.0; fused = 0.5*1.0 + 0.5*1.0 = 1.0
        let state = fusion.fused_state(0.1);
        assert!((state.heading_rad - 1.0).abs() < 1e-5);
    }

    #[test]
    fn imu_integration_with_nonzero_omega() {
        let mut fusion = SensorFusion::new(1.0); // pure IMU
        fusion.update_odometry(odom(0.0, 0.0, 0.0));
        fusion.update_imu(imu(2.0)); // 2 rad/s

        // dt = 0.5 s → imu_predicted = 0 + 2.0 * 0.5 = 1.0
        // alpha = 1.0 → fused = 1.0 * 1.0 + 0.0 * 0.0 = 1.0
        let state = fusion.fused_state(0.5);
        assert!((state.heading_rad - 1.0).abs() < 1e-5);
    }

    #[test]
    fn alpha_clamped_to_unit_interval() {
        let fusion_high = SensorFusion::new(5.0);
        assert!((fusion_high.alpha - 1.0).abs() < 1e-5);

        let fusion_low = SensorFusion::new(-1.0);
        assert!((fusion_low.alpha - 0.0).abs() < 1e-5);
    }

    #[test]
    fn update_replaces_previous_measurement() {
        let mut fusion = SensorFusion::new(0.0); // pure odometry heading
        fusion.update_odometry(odom(1.0, 0.0, 0.5));
        fusion.update_odometry(odom(2.0, 0.0, 1.5)); // newer reading

        let state = fusion.fused_state(0.0);
        assert!((state.position_x - 2.0).abs() < 1e-5);
        assert!((state.heading_rad - 1.5).abs() < 1e-5);
    }

    #[test]
    fn negative_dt_treated_as_zero() {
        let mut fusion = SensorFusion::new(1.0);
        fusion.update_odometry(odom(0.0, 0.0, 0.0));
        fusion.update_imu(imu(10.0));

        // Negative dt → dt is clamped to 0 → imu_predicted = 0 + 10.0*0 = 0.
        let state = fusion.fused_state(-1.0);
        assert!((state.heading_rad).abs() < 1e-5);
    }

    #[test]
    fn velocity_propagated_from_odometry() {
        let mut fusion = SensorFusion::new(0.98);
        fusion.update_odometry(OdometryData {
            position_x: 0.0,
            position_y: 0.0,
            heading_rad: 0.0,
            velocity_x: 1.2,
            velocity_y: 0.3,
        });

        let state = fusion.fused_state(0.01);
        assert!((state.velocity_x - 1.2).abs() < 1e-5);
        assert!((state.velocity_y - 0.3).abs() < 1e-5);
    }
}
