#pragma once

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/optional.hpp>

namespace camera_pose_calibration {

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

}
