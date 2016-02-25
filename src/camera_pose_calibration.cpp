#include "camera_pose_calibration_impl.hpp"
#include "camera_pose_calibration.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/project_inliers.h>

namespace camera_pose_calibration {

pcl::ModelCoefficients::Ptr fitPointsToPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	// optional
	seg.setOptimizeCoefficients(true);

	// mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) {
		throw std::runtime_error("Could not estimate a planar model for the given pointcloud.");
	}

	return coefficients;
}

std::vector<size_t> findNan(pcl::PointCloud<pcl::PointXYZ> const & cloud) {
	std::vector<size_t> nan_indices;
	for (size_t i = 0; i < cloud.size(); i++) {
		if (std::isnan(cloud.at(i).x) ||
			std::isnan(cloud.at(i).y) ||
			std::isnan(cloud.at(i).z)) {
			nan_indices.push_back(i);
		}
	}
	return nan_indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generateAsymmetricCircles(
	double distance,
	size_t pattern_height,
	size_t pattern_width
) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t j = 0; j < pattern_height; j++) {
		for (size_t i = 0; i < pattern_width; i++) {
			pcl::PointXYZ point;
			double offset = (j % 2 == 0 ? 0 : distance / 2);
			point.x = j * 0.5 * distance;
			point.y = i * distance + offset;
			point.z = 0;
			cloud->push_back(point);
		}
	}
	return cloud;
}

void projectCloudOnPlane(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected,
	pcl::ModelCoefficients::ConstPtr plane_coefficients
) {
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(plane_coefficients);
	proj.filter(*cloud_projected);
}

void eraseIndices(std::vector<size_t> indices, pcl::PointCloud<pcl::PointXYZ> & cloud) {
	// sort in descending order to keep indices matching the updated cloud
	std::sort(indices.begin(), indices.end(), std::greater<size_t>());
	for (size_t i = 0; i < indices.size(); i++) {
		cloud.erase(cloud.begin() + indices.at(i));
	}
}

Eigen::Isometry3d findIsometry(
	pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target
) {
	Eigen::Matrix4f transformation;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
	svd.estimateRigidTransformation(*source, *target, transformation);
	return Eigen::Isometry3d(transformation.cast<double>());
}

Eigen::Isometry3d findCalibration(
	cv::Mat const & image,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	cv::Size const pattern_size,
	double pattern_distance,
	double neighbor_distance,
	double valid_pattern_ratio_threshold,
	double point_cloud_scale_x,
	double point_cloud_scale_y,
	CalibrationInformation * debug_information
) {
	if (point_cloud_scale_x == 0) point_cloud_scale_x = 1;
	if (point_cloud_scale_y == 0) point_cloud_scale_y = 1;

	// find pattern
	std::vector<cv::Point2f> image_points;
	if (!cv::findCirclesGrid(image, pattern_size, image_points, cv::CALIB_CB_ASYMMETRIC_GRID)) {
		throw std::runtime_error("Failed to find calibration pattern.");
	}

	if (debug_information) {
		debug_information->image_points = image_points;
	}

	// get the (x, y, z) points of the calibration pattern
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
	kd_tree.setInputCloud(cloud);
	for (cv::Point const & p : image_points) {
		if (p.x < 0 || p.y < 0) {
			throw std::runtime_error("Found invalid image point for calibration pattern point.");
		}

		pcl::PointXYZ average = cloud->at(p.x * point_cloud_scale_x, p.y * point_cloud_scale_y);
		if (std::isnan(average.x) || std::isnan(average.y) || std::isnan(average.z)) {
			source_cloud->push_back(average);
			continue;
		}

		// find neighboring pixels and average them
		if (neighbor_distance > 0) {
			std::vector<int> neighbor_indices;
			std::vector<float> neighbor_squared_distances;
			kd_tree.radiusSearch(average, neighbor_distance, neighbor_indices, neighbor_squared_distances);
			for (int index : neighbor_indices) {
				average.x += cloud->points[index].x;
				average.y += cloud->points[index].y;
				average.z += cloud->points[index].z;
			}
			average.x /= neighbor_indices.size() + 1;
			average.y /= neighbor_indices.size() + 1;
			average.z /= neighbor_indices.size() + 1;
		}

		source_cloud->push_back(average);
	}

	// create target (expected) model
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = generateAsymmetricCircles(pattern_distance, pattern_size.height, pattern_size.width);

	if (debug_information) {
		debug_information->source_cloud = source_cloud;
		debug_information->target_cloud = target_cloud;
	}

	// remove NaN's
	std::vector<size_t> nan_indices = findNan(*source_cloud);

	if (double(nan_indices.size()) / pattern_size.area() > 1.f - valid_pattern_ratio_threshold) {
		throw std::runtime_error("Found too many invalid (NaN) points to find isometry.");
	}

	eraseIndices(nan_indices, *source_cloud);
	eraseIndices(nan_indices, *target_cloud);

	if (debug_information) {
		debug_information->nan_indices = nan_indices;
	}

	// project calibration points to plane to reduce noise
	pcl::ModelCoefficients::Ptr plane_coefficients = fitPointsToPlane(source_cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr projected_source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	projectCloudOnPlane(source_cloud, projected_source_cloud, plane_coefficients);

	if (debug_information) {
		debug_information->plane_coefficients = plane_coefficients;
		debug_information->projected_source_cloud = projected_source_cloud;
	}

	// find actual isometry from target (calibration tag) to source (camera)
	Eigen::Isometry3d isometry = findIsometry(projected_source_cloud, target_cloud);

	return isometry;
}

Eigen::Isometry3d findCalibration(
	cv::Mat const & left_image,
	cv::Mat const & right_image,
	Eigen::Matrix4d const & reprojection,
	cv::Size pattern_size,
	double pattern_distance
) {
	// find pattern
	std::vector<cv::Point2f> left_points, right_points;

	bool detected_patterns =
		cv::findCirclesGrid(left_image,  pattern_size, left_points,  cv::CALIB_CB_ASYMMETRIC_GRID) &&
		cv::findCirclesGrid(right_image, pattern_size, right_points, cv::CALIB_CB_ASYMMETRIC_GRID);

	if (!detected_patterns) {
		throw std::runtime_error("Failed to find calibration patterns.");
	}

	// get the (x, y, z) points of the calibration pattern
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < pattern_size.area(); ++i) {
		Eigen::Vector4d point{
			left_points[i].x, left_points[i].y, left_points[i].x - right_points[i].x, 1
		};
		Eigen::Vector3d result = (reprojection * point).hnormalized();
		source_cloud->push_back(pcl::PointXYZ(result[0], result[1], result[2]));
	}

	// create target (expected) model
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = generateAsymmetricCircles(pattern_distance, pattern_size.height, pattern_size.width);

	// project calibration points to plane to reduce noise
	pcl::ModelCoefficients::Ptr plane_coefficients = fitPointsToPlane(source_cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr projected_source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	projectCloudOnPlane(source_cloud, projected_source_cloud, plane_coefficients);

	// find actual isometry from target (calibration tag) to source (camera)
	Eigen::Isometry3d isometry = findIsometry(projected_source_cloud, target_cloud);

	return isometry;
}

}
