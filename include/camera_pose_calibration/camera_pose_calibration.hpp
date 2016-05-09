/*
 * Copyright 2016 Delft Robotics B.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstddef>

namespace camera_pose_calibration {

struct CalibrationInformation {
	/// List of coordinates in image for where points are found.
	std::vector<cv::Point2f> image_points;

	/// Points of the detected pattern points.
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;

	/// Points of the detected pattern projected on a plane.
	pcl::PointCloud<pcl::PointXYZ>::Ptr projected_source_cloud;

	/// Plane coefficients for the fitted plane.
	pcl::ModelCoefficients::Ptr plane_coefficients;

	/// Target model to find the isometry to.
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;

	/// Indices of invalid points.
	std::vector<size_t> nan_indices;
};

/// Finds the isometry for the asymmetric calibration pattern in the image and pointcloud.
Eigen::Isometry3d findCalibration(
	cv::Mat const & image,                      ///< The image in which the calibration pattern is visible.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  ///< The corresponding (registered) pointcloud.
	cv::Size const pattern_size,                ///< Size of the asymmetric calibration pattern.
	double pattern_distance,                    ///< Distance between the center of the points on the pattern.
	double neighbor_distance = 0.01,            ///< Distance in meters to detected pattern points to which neighbors are searched and averaged. A distance of <= 0 means use no neighbors.
	double valid_pattern_ratio_threshold = 0.7, ///< When the ratio of valid_points / (pattern_width * pattern_height) is below this threshold, there are not enough points to estimate an isometry.
	double point_cloud_scale_x = 1.0,           ///< Scale in x used to transform points from intensity to pointcloud frame.
	double point_cloud_scale_y = 1.0,           ///< Scale in y used to transform points from intensity to pointcloud frame.
	CalibrationInformation * debug_information = NULL ///< Extra debugging information.
);

/// Finds the isometry for the asymmetric calibration pattern, given two (undistorted) stereo images and a reprojection matrix.
Eigen::Isometry3d findCalibration(
	cv::Mat const & left_image,                 ///< The (undistorted) left image in which the calibration pattern is visible.
	cv::Mat const & right_image,                ///< The (undistorted) right image in which the calibration pattern is visible.
	Eigen::Matrix4d const & reprojection,       ///< The reprojection matrix.
	cv::Size const pattern_size,                ///< Size of the asymmetric calibration pattern.
	double pattern_distance                     ///< Distance between the center of the points on the pattern.
);


}
