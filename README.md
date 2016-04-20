# Camera Pose Calibration

## Table of contents
- [Description](#description)
- [Assumptions](#assumptions)
- [Usage](#usage)

## Description

This package calculates the pose of a camera to a fixed frame.
The tf between the camera and fixed frame is optionally published on the ROS server.
This package works only for the asymmetric circle pattern because the pose of an asymmetric circle pattern is uniquely defined.
Other standard calibration patterns such as chessboard pattern and circles pattern do not have this property.
The tf tree is connected by recognizing the pose of the asymmetric circles calibration pattern with an image and a point cloud.

This package contains only a subset of the functionality of other calibration packages and is not meant as a replacement for - or addition to - these packages.
There are other, more advanced, calibration packages available than this package.
A good example is industrial_extrinsic_cal.
The only advantage of this package is that it requires minimal configuration and is easy to set up.
Especially if your camera driver gives you an image and a point cloud, which is often the case for time-of-flight cameras.

## Assumptions
- The tf between camera and asymmetric circle pattern is static (does not change over time)
- The tf of the asymetric circle pattern and the fixed frame is published on the ROS server
- The asymmetric circle pattern can be detected in the image, and point cloud data is available for the corresponding points

## Usage

#### Defining the calibration_tag frame for asymmetric circles calibration plate

The asymmetric cicles calibration node in this package assumes this definition of the calibration tag to be present as a tf transform and connected to the fixed frame.

Make sure the calibration_tag conforms to the definition given below, or you might get unexpected results.
Imagine the square which circumscribes the calibration plate.
Position the calibration plate on a table such that the two corner points of this square which have calibration dots on them are towards you.
The lower left dot then represents the zero point of the calibration_tag frame.

- The X axis points to the right.
- The Y axis points forwards away from you.
- The Z-axis points upwards, out of the calibration plate.

The tf from calibration tag to the fixed frame could for example be written in the URDF to get the tf on the ROS server.

#### Services

Three services are provided:
All three of them perform the same calibration.
They differ only in the way the image and point cloud is received.
- /calibrate_call: The image and point cloud data is given in the service call request.
- /calibrate_file: The image and point cloud path is given, the calibration is performed with these files.
- /calibrate_topic: The image and point cloud are received from the ROS server.

Configuration parameters are given in the service call request.
The detectopm of the asymmetric circle pattern will commence after calling the service.
The pose of the camera to the fixed frame will then be returned and optionally published.

Remaining service call request arguments:
- tag frame: Name of the asymmetric circle frame on the ROS server
- target frame: Tf frame to connect camera to
- point_cloud_scale_x: Scale factor between image and point cloud (default 1)
- point_cloud_scale_y: Scale factor between image and point cloud (default 1)
- pattern: Pattern msg type with distance and size of the asymmetric circles pattern.
- neighbor_distance: Find neighboring pixels within a certain distance in pixels and average their x, y, z values
- valid_pattern_ratio_threshold: Acceptance threshold for the ratio of valid points to NaN values

#### Planned features

- Correction for perspective in determining the blob center.
- Create services for calibrating with two intensity images and the reprojection matrix

