//! Generic `Camera` trait and supporting types for image-capture hardware.

use mechos_types::MechError;

/// A raw image frame returned by a camera driver.
#[derive(Debug, Clone)]
pub struct CameraFrame {
    /// Frame width in pixels.
    pub width: u32,
    /// Frame height in pixels.
    pub height: u32,
    /// Raw pixel data (e.g. RGB24 or greyscale).
    pub data: Vec<u8>,
}

/// A camera or image-capture device.
///
/// Drivers implement this trait and register themselves with a
/// [`HardwareRegistry`][crate::registry::HardwareRegistry].
pub trait Camera: Send + Sync {
    /// Stable identifier for this camera, e.g. `"front_rgb"`.
    fn id(&self) -> &str;

    /// Capture and return the next available frame.
    ///
    /// # Errors
    ///
    /// Returns [`MechError::HardwareFault`] if the frame cannot be captured
    /// (e.g. the device is disconnected or the buffer is unavailable).
    fn capture(&mut self) -> Result<CameraFrame, MechError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockCamera {
        id: String,
    }

    impl Camera for MockCamera {
        fn id(&self) -> &str {
            &self.id
        }

        fn capture(&mut self) -> Result<CameraFrame, MechError> {
            Ok(CameraFrame {
                width: 2,
                height: 2,
                data: vec![0u8; 4 * 3], // 2×2 RGB24
            })
        }
    }

    #[test]
    fn mock_camera_capture() {
        let mut cam = MockCamera {
            id: "front_rgb".to_string(),
        };
        assert_eq!(cam.id(), "front_rgb");
        let frame = cam.capture().unwrap();
        assert_eq!(frame.width, 2);
        assert_eq!(frame.height, 2);
        assert_eq!(frame.data.len(), 12);
    }
}
