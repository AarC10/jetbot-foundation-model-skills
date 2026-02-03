## Subscribed Topics

- `image_raw` (sensor_msgs/Image): Raw camera images
- `camera_info` (sensor_msgs/CameraInfo): Camera calibration information

## Published Topics

- `image_rect` (sensor_msgs/Image): Rectified camera images
- `camera_info_rect` (sensor_msgs/CameraInfo): Updated camera info for rectified images

## Parameters

- `input_image_topic` (string, default: "image_raw"): Topic name for input images
- `input_camera_info_topic` (string, default: "camera_info"): Topic name for input camera info
- `output_image_topic` (string, default: "image_rect"): Topic name for output rectified images
- `output_camera_info_topic` (string, default: "camera_info_rect"): Topic name for output rectified camera info
- `alpha` (double, default: 0.0): Free scaling parameter for getOptimalNewCameraMatrix
  - 0.0 = maximize valid pixel area (may crop image)
  - 1.0 = retain all source image pixels (may include black borders)
- `interpolate` (bool, default: true): Enable bilinear interpolation during rectification

## Implementation Details

### Rectification Process

1. Receives camera calibration info (K, D, R, P matrices)
2. Computes optimal new camera matrix using alpha parameter
3. Initializes undistortion and rectification maps
4. For each incoming image:
   - Applies remap using pre-computed maps
   - Publishes rectified image
   - Publishes updated camera info (with zero distortion)

