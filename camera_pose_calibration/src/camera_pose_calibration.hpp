#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <dr_opencv/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/optional.hpp>

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

/// Returns the plane parameters of the fit to the input point cloud
pcl::ModelCoefficients::Ptr fitPointsToPlane(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud   ///< Point cloud
);

/// Returns the indices with nan value for x, y or z
std::vector<size_t> findNan(
	pcl::PointCloud<pcl::PointXYZ> const & cloud     ///< Point cloud
);

/// Defines the asymmetric circles calibration pattern
pcl::PointCloud<pcl::PointXYZ>::Ptr generateAsymmetricCircles(
	double distance,                            ///< Distance between adjacent circles
	size_t pattern_height,                      ///< Number of circles in vertical direction
	size_t pattern_width                        ///< Number of circles in horizontal direction
);

/// Fits a plane to a point cloud and gives the projected point cloud and the plane coefficients
void projectCloudOnPlane(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,       ///< Point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected,  ///< Point cloud projected to the fitted plane
	pcl::ModelCoefficients::ConstPtr plane_coefficients   ///< Plane coefficients
);

///< Erases the indices from cloud
void eraseIndices(
	std::vector<size_t> indices,                ///< Indices to erase
	pcl::PointCloud<pcl::PointXYZ> & cloud      ///< Point cloud to erase indices from
);

/// Finds the isometry between two point clouds
Eigen::Isometry3d findIsometry(
	pcl::PointCloud<pcl::PointXYZ>::Ptr source, ///< Source point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr target  ///< Target point cloud
);

/// Finds the isometry for the asymmetric calibration pattern in the image and pointcloud.
Eigen::Isometry3d findCalibrationIsometry(
	cv::Mat const & image,                      ///< The image in which the calibration pattern is visible.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  ///< The corresponding (registered) pointcloud.
	int pattern_width,                          ///< Width of the asymmetric calibration pattern.
	int pattern_height,                         ///< Height of the asymmetric calibration pattern.
	double pattern_distance,                    ///< Distance between the center of the points on the pattern.
	double neighbor_distance = 0.01,            ///< Distance in meters to detected pattern points to which neighbors are searched and averaged. A distance of <= 0 means use no neighbors.
	double valid_pattern_ratio_threshold = 0.7, ///< When the ratio of valid_points / (pattern_width * pattern_height) is below this threshold, there are not enough points to estimate an isometry.
	double point_cloud_scale_x = 1.0,           ///< Scale in x used to transform points from intensity to pointcloud frame.
	double point_cloud_scale_y = 1.0,           ///< Scale in y used to transform points from intensity to pointcloud frame.
	std::shared_ptr<CalibrationInformation> const debug_information = nullptr ///< Extra debugging information.
);

}
