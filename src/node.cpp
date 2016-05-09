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

#include "node.hpp"

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/bind.hpp>

#include <boost/thread/future.hpp>

namespace camera_pose_calibration {

namespace {
	template<typename T>
	T getParam(ros::NodeHandle & node, std::string const & name, T const & default_value) {
		T result;
		node.param(name, result, default_value);
		return result;
	}

	/// Topic to read point cloud from.
	char const * cloud_topic = "points_registered";

	/// Topic to read image from.
	char const * image_topic = "image_color";
}

CameraPoseCalibrationNode::CameraPoseCalibrationNode() :
	image_transport(node_handle),
	calibrated(false)
{
	// initialize ros communication
	cloud_publisher                    = node_handle.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud", 1, true);
	target_cloud_publisher             = node_handle.advertise<pcl::PointCloud<pcl::PointXYZ> >("target", 1, true);
	transformed_target_cloud_publisher = node_handle.advertise<pcl::PointCloud<pcl::PointXYZ> >("transformed_target", 1, true);
	source_cloud_publisher             = node_handle.advertise<pcl::PointCloud<pcl::PointXYZ> >("source", 1, true);
	projected_source_cloud_publisher   = node_handle.advertise<pcl::PointCloud<pcl::PointXYZ> >("projected_source", 1, true);
	calibration_plane_marker_publisher = node_handle.advertise<visualization_msgs::Marker>("calibration_plane", 1, true);
	detected_pattern_publisher         = image_transport.advertise("detected_pattern", 1, true);

	calibrate_server_call  = node_handle.advertiseService("calibrate_call",  &CameraPoseCalibrationNode::onCalibrateCall,  this);
	calibrate_server_topic = node_handle.advertiseService("calibrate_topic", &CameraPoseCalibrationNode::onCalibrateTopic, this);
	calibrate_server_file  = node_handle.advertiseService("calibrate_file",  &CameraPoseCalibrationNode::onCalibrateFile,  this);

	// parameters
	publish_transform = getParam(node_handle, "publish_transform", false);
	publish_rate      = getParam(node_handle, "publish_rate", 1);

	if (publish_transform) {
		tf_timer = node_handle.createTimer(publish_rate, &CameraPoseCalibrationNode::onTfTimeout, this);
	}
}

visualization_msgs::Marker CameraPoseCalibrationNode::createCalibrationPlaneMarker(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
	uint8_t pattern_width,
	uint8_t pattern_height
) {
	// create the calibration plane marker
	visualization_msgs::Marker calibration_plane;
	calibration_plane.header.frame_id = points->header.frame_id;
	calibration_plane.ns              = "calibration";
	calibration_plane.type            = visualization_msgs::Marker::TRIANGLE_LIST;
	calibration_plane.action          = visualization_msgs::Marker::ADD;
	calibration_plane.color.r         = 1.0;
	calibration_plane.color.g         = 1.0;
	calibration_plane.color.a         = 1.0;
	calibration_plane.scale.x         = 1.0;
	calibration_plane.scale.y         = 1.0;
	calibration_plane.scale.z         = 1.0;
	calibration_plane.frame_locked    = true;
	calibration_plane.id              = 0;

	// triangle 1
	geometry_msgs::Point triangle_point;
	triangle_point.x = points->points[0].x;
	triangle_point.y = points->points[0].y;
	triangle_point.z = points->points[0].z;
	calibration_plane.points.push_back(triangle_point);
	triangle_point.x = points->points[(pattern_height - 1) * pattern_width].x;
	triangle_point.y = points->points[(pattern_height - 1) * pattern_width].y;
	triangle_point.z = points->points[(pattern_height - 1) * pattern_width].z;
	calibration_plane.points.push_back(triangle_point);
	triangle_point.x = points->points[pattern_width - 1].x;
	triangle_point.y = points->points[pattern_width - 1].y;
	triangle_point.z = points->points[pattern_width - 1].z;
	calibration_plane.points.push_back(triangle_point);

	// triangle 2
	triangle_point.x = points->points[pattern_height * pattern_width - 1].x;
	triangle_point.y = points->points[pattern_height * pattern_width - 1].y;
	triangle_point.z = points->points[pattern_height * pattern_width - 1].z;
	calibration_plane.points.push_back(triangle_point);
	triangle_point.x = points->points[pattern_width - 1].x;
	triangle_point.y = points->points[pattern_width - 1].y;
	triangle_point.z = points->points[pattern_width - 1].z;
	calibration_plane.points.push_back(triangle_point);
	triangle_point.x = points->points[(pattern_height - 1) * pattern_width].x;
	triangle_point.y = points->points[(pattern_height - 1) * pattern_width].y;
	triangle_point.z = points->points[(pattern_height - 1) * pattern_width].z;
	calibration_plane.points.push_back(triangle_point);

	return calibration_plane;
}

void CameraPoseCalibrationNode::onTfTimeout(ros::TimerEvent const &) {
	if (calibrated) {
		calibration_transform.stamp_ = ros::Time::now();
		transform_broadcaster.sendTransform(calibration_transform);
	}
}

bool CameraPoseCalibrationNode::calibrate(
	sensor_msgs::Image const & sensor_msgs_image,
	sensor_msgs::PointCloud2 const & sensor_msgs_cloud,
	std::string const & tag_frame,
	std::string const & target_frame,
	double const & point_cloud_scale_x,
	double const & point_cloud_scale_y,
	camera_pose_calibration::PatternParameters const & pattern,
	geometry_msgs::Transform & transform
) {
	ROS_INFO_STREAM("Received calibration request from '" << sensor_msgs_cloud.header.frame_id << "' to '" << target_frame << "'.");

	// extract image
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(sensor_msgs_image, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR_STREAM("Failed to convert sensor_msgs/Image to cv::Mat. Error: " << e.what());
		return false;
	}
	cv::Mat image = cv_ptr->image;

	// extract pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(sensor_msgs_cloud, *cloud);
	if (sensor_msgs_cloud.header.frame_id == "") {
		ROS_ERROR_STREAM("Found empty frame_id in given pointcloud.");
		return false;
	}

	// find calibration plate in image, and find isometry from camera to calibration plate
	CalibrationInformation debug_information;
	Eigen::Isometry3d camera_to_tag;
	try {
		camera_to_tag = camera_pose_calibration::findCalibration(
			image,
			cloud,
			cv::Size(pattern.pattern_width, pattern.pattern_height),
			pattern.pattern_distance,
			pattern.neighbor_distance,
			pattern.valid_pattern_ratio_threshold,
			point_cloud_scale_x,
			point_cloud_scale_y,
			&debug_information
		);
	} catch (std::exception const & e) {
		ROS_ERROR_STREAM("Failed to find isometry. Error: " << e.what());
		detected_pattern_publisher.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
		return false;
	}

	// get the current time for publishing stamps
	ros::Time now = sensor_msgs_cloud.header.stamp;

	// find transformation from tag frame to the frame to which we want to calibrate
	Eigen::Isometry3d tag_to_target;
	if (tag_frame == target_frame) {
		tag_to_target = Eigen::Isometry3d::Identity();
	} else {
		try {
			tf::StampedTransform transform;
			if (!transform_listener.waitForTransform(target_frame, tag_frame, now, ros::Duration(1.0))) {
				ROS_ERROR_STREAM("Failed to find transform from " << tag_frame << " to " << target_frame);
				return false;
			}
			transform_listener.lookupTransform(target_frame, tag_frame, now, transform);

			tf::transformTFToEigen(transform, tag_to_target);
		} catch (tf::TransformException const & e) {
			ROS_ERROR_STREAM("Failed to find transform. Error: " << e.what());
			return false;
		}
	}

	// transform for target to camera frame
	Eigen::Isometry3d camera_to_target = tag_to_target * camera_to_tag;

	// clone image and draw detection
	cv::Mat detected_pattern = image.clone();
	cv::drawChessboardCorners(detected_pattern, cv::Size(pattern.pattern_width, pattern.pattern_height), cv::Mat(debug_information.image_points), true);

	// transform the target (model) to the target frame to verify a correct calibration
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_target_cloud(new pcl::PointCloud<pcl::PointXYZ>(*debug_information.target_cloud));
	pcl::transformPointCloud(*debug_information.target_cloud, *transformed_target_cloud, Eigen::Affine3d(camera_to_tag.inverse()));

	// copy result to response
	tf::transformEigenToMsg(camera_to_target, transform);

	// publish result if necessary
	calibrated = true;
	tf::Transform tf_camera_to_target;
	transformEigenToTF(camera_to_target, tf_camera_to_target);
	calibration_transform = tf::StampedTransform(tf_camera_to_target, now, target_frame, cloud->header.frame_id);
	if (publish_transform) {
		transform_broadcaster.sendTransform(calibration_transform);
	}

	// copy headers to publishing pointclouds
	pcl_conversions::toPCL(now, cloud->header.stamp);
	transformed_target_cloud->header                  = cloud->header;
	debug_information.projected_source_cloud->header = cloud->header;
	debug_information.source_cloud->header           = cloud->header;
	debug_information.target_cloud->header           = cloud->header;
	debug_information.target_cloud->header.frame_id  = tag_frame;

	// publish calibration plane
	visualization_msgs::Marker calibration_plane = createCalibrationPlaneMarker(debug_information.projected_source_cloud, pattern.pattern_width, pattern.pattern_height);
	calibration_plane.header.stamp = now;
	calibration_plane_marker_publisher.publish(calibration_plane);

	// publish the debug information
	cloud_publisher.publish(cloud);
	target_cloud_publisher.publish(debug_information.target_cloud);
	transformed_target_cloud_publisher.publish(transformed_target_cloud);
	source_cloud_publisher.publish(debug_information.source_cloud);
	projected_source_cloud_publisher.publish(debug_information.projected_source_cloud);
	detected_pattern_publisher.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", detected_pattern).toImageMsg());

	ROS_INFO_STREAM(cloud->header.frame_id << " to " << tag_frame << " :\n" << camera_to_tag.matrix() );
	ROS_INFO_STREAM(cloud->header.frame_id << " to " << target_frame << " :\n" << camera_to_target.matrix() );

	return true;
}

bool CameraPoseCalibrationNode::onCalibrateFile(camera_pose_calibration::CalibrateFile::Request & req, camera_pose_calibration::CalibrateFile::Response & res) {
	// load the image
	std::string image_path = req.image;
	std_msgs::Header header;
	header.frame_id = req.camera_frame;
	cv_bridge::CvImage image_msg(header, sensor_msgs::image_encodings::BGR8, cv::imread(image_path));
	if (!image_msg.image.data) {
		ROS_ERROR_STREAM("Failed to read image from " << image_path << ".");
		return false;
	}

	sensor_msgs::Image image;
	image_msg.toImageMsg(image);

	// load the pointcloud
	std::string cloud_path = req.cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile(cloud_path, *pcl_cloud) == -1) {
		ROS_ERROR_STREAM("Failed to read pointcloud " << cloud_path << ".");
		return false;
	}

	pcl_cloud->header.frame_id   = req.camera_frame;
	pcl_conversions::toPCL(ros::Time::now(), pcl_cloud->header.stamp);

	sensor_msgs::PointCloud2 cloud;
	pcl::toROSMsg(*pcl_cloud, cloud);

	return calibrate(
		image,
		cloud,
		req.tag_frame,
		req.target_frame,
		req.point_cloud_scale_x,
		req.point_cloud_scale_y,
		req.pattern,
		res.transform
	);
}

typedef std::pair<sensor_msgs::Image::ConstPtr, sensor_msgs::PointCloud2::ConstPtr> InputData;

void synchronizationCallback(boost::promise<InputData> & promise, sensor_msgs::Image::ConstPtr image, sensor_msgs::PointCloud2::ConstPtr cloud) {
	promise.set_value(InputData(image, cloud));
}

bool CameraPoseCalibrationNode::onCalibrateTopic(camera_pose_calibration::CalibrateTopic::Request & req, camera_pose_calibration::CalibrateTopic::Response & res) {
	// Use a specific callback queue for the data topics.
	ros::CallbackQueue queue;

	// Synchronize image and point cloud from topic
	message_filters::Subscriber<sensor_msgs::Image> image_sub(node_handle, image_topic, 1, ros::TransportHints(), &queue);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(node_handle, cloud_topic, 1, ros::TransportHints(), &queue);
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(image_sub, cloud_sub, 10);


	// Use a promise/future for communicating the data between threads.
	boost::promise<InputData> promise;
	boost::unique_future<InputData> future = promise.get_future();
	sync.registerCallback(boost::bind(&synchronizationCallback, boost::ref(promise), _1, _2));

	// Run a background spinner for the callback queue.
	ros::AsyncSpinner spinner(1, &queue);
	spinner.start();

	// Wait for the future.
	while (true) {
		if (!node_handle.ok()) return false;
		if (future.wait_for(boost::chrono::milliseconds(100)) == boost::future_status::ready) break;
	}

	// Stop the spinner and copy the result to the request.
	spinner.stop();
	InputData data = future.get();
	sensor_msgs::Image image       = *data.first;
	sensor_msgs::PointCloud2 cloud = *data.second;

	if (image.data.empty() || cloud.data.empty()) {
		ROS_ERROR_STREAM("No image and/or point cloud received.");
		return false;
	}

	// Get all other calibration information from the service call request
	return calibrate(
		image,
		cloud,
		req.tag_frame,
		req.target_frame,
		req.point_cloud_scale_x,
		req.point_cloud_scale_y,
		req.pattern,
		res.transform
	);
}

/// Calibrates the camera given the information in the request.
bool CameraPoseCalibrationNode::onCalibrateCall(camera_pose_calibration::CalibrateCall::Request & req, camera_pose_calibration::CalibrateCall::Response & res) {
	return calibrate(
		req.image,
		req.cloud,
		req.tag_frame,
		req.target_frame,
		req.point_cloud_scale_x,
		req.point_cloud_scale_y,
		req.pattern,
		res.transform
	);
}


}

