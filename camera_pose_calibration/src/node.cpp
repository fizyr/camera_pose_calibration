#include "node.hpp"

#include <dr_opencv/opencv.hpp>

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

#include <future>
#include <tuple>

namespace camera_pose_calibration {

using namespace dr;

CameraPoseCalibrationNode::CameraPoseCalibrationNode() :
	image_transport(node_handle_),
	calibrated(false)
{
	// initialize ros communication
	cloud_publisher                    = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1, true);
	target_cloud_publisher             = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ>>("target", 1, true);
	transformed_target_cloud_publisher = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ>>("transformed_target", 1, true);
	source_cloud_publisher             = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ>>("source", 1, true);
	projected_source_cloud_publisher   = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ>>("projected_source", 1, true);
	calibration_plane_marker_publisher = node_handle_.advertise<visualization_msgs::Marker>("calibration_plane", 1, true);
	detected_pattern_publisher         = image_transport.advertise("detected_pattern", 1, true);

	calibrate_server       = node_handle_.advertiseService("calibrate",       &CameraPoseCalibrationNode::onCalibrate,      this);
	calibrate_server_topic = node_handle_.advertiseService("calibrate_topic", &CameraPoseCalibrationNode::onCalibrateTopic, this);

	// parameters
	publish_transform = getParam("publish_transform", false, false);
	publish_rate      = getParam("publish_rate", 1, false);

	if (publish_transform) {
		tf_timer = node_handle_.createTimer(publish_rate, &CameraPoseCalibrationNode::onTfTimeout, this);
	}

	parseInput();
}

/// Parses ROS parameters and calls the calibration service.
void CameraPoseCalibrationNode::parseInput() {
	// no manual input, only through service calls
	bool manual_input = getParam("manual", false);
	if (!manual_input) return;

	// construct the request
	dr_msgs::Calibrate::Request req;
	req.pattern_width                 = getParam("pattern_width", 3);
	req.pattern_height                = getParam("pattern_height", 9);
	req.pattern_distance              = getParam("pattern_distance", 0.04);
	req.neighbor_distance             = getParam("neighbor_distance", 0.01);
	req.valid_pattern_ratio_threshold = getParam("valid_pattern_ratio_threshold", 0.7);
	req.tag_frame                     = getParam<std::string>("tag_frame", "calibration_tag");
	req.target_frame                  = getParam<std::string>("target_frame", "calibration_tag");
	std::string camera_frame          = getParam<std::string>("camera_frame", "camera_link");

	// load the image
	std::string image_path = getParam<std::string>("image_path", "intensity.png");
	std_msgs::Header header;
	header.frame_id = camera_frame;
	cv_bridge::CvImage image_msg(header, sensor_msgs::image_encodings::BGR8, cv::imread(image_path));
	if (!image_msg.image.data) {
		DR_ERROR("Failed to read image from " << image_path << ".");
		return;
	}

	image_msg.toImageMsg(req.image);

	// load the pointcloud
	std::string cloud_path = getParam<std::string>("cloud_path", "cloud.pcd");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile(cloud_path, *cloud) == -1) {
		DR_ERROR("Failed to read pointcloud " << cloud_path << ".");
		return;
	}

	cloud->header.frame_id   = camera_frame;
	pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);

	pcl::toROSMsg(*cloud, req.cloud);

	// call the service
	dr_msgs::Calibrate::Response res;
	onCalibrate(req, res);
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

bool CameraPoseCalibrationNode::onCalibrateTopic(dr_msgs::CalibrateTopic::Request & req, dr_msgs::CalibrateTopic::Response & res) {
	// Use a specific callback queue for the data topics.
	ros::CallbackQueue queue;

	// Synchronize image and point cloud from topic
	message_filters::Subscriber<sensor_msgs::Image> image_sub(node_handle_, req.image_topic, 1, ros::TransportHints(), &queue);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(node_handle_, req.cloud_topic, 1, ros::TransportHints(), &queue);
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(image_sub, cloud_sub, 10);

	// Use a promise/future for communicating the data between threads.
	std::promise<std::tuple<sensor_msgs::Image, sensor_msgs::PointCloud2>> promise;
	auto future = promise.get_future();
	auto callback = std::bind([&promise] (sensor_msgs::Image const & image, sensor_msgs::PointCloud2 const & cloud) {
		promise.set_value(std::make_tuple(image, cloud));
	});
	sync.registerCallback(callback);

	// Run a background spinner for the callback queue.
	ros::AsyncSpinner spinner(1, &queue);
	spinner.start();

	// Wait for the future.
	while (true) {
		if (!node_handle_.ok()) return false;
		future.wait_for(std::chrono::milliseconds(100));
		if (future.valid()) break;
	}

	dr_msgs::Calibrate::Request calibrate_request;
	dr_msgs::Calibrate::Response calibrate_response;

	// Stop the spinner and copy the result to the request.
	spinner.stop();
	std::tie(calibrate_request.image, calibrate_request.cloud) = future.get();

	if (!calibrate_request.image.data.size() > 0 || !calibrate_request.cloud.data.size() > 0) {
		DR_ERROR("No image and/or point cloud received.");
		return false;
	}

	// Get all other calibration information from the service call request
	calibrate_request.pattern_width                 = req.pattern_width;
	calibrate_request.pattern_height                = req.pattern_height;
	calibrate_request.pattern_distance              = req.pattern_distance;
	calibrate_request.neighbor_distance             = req.neighbor_distance;
	calibrate_request.valid_pattern_ratio_threshold = req.valid_pattern_ratio_threshold;
	calibrate_request.tag_frame                     = req.tag_frame;
	calibrate_request.target_frame                  = req.target_frame;
	calibrate_request.point_cloud_scale_x           = req.point_cloud_scale_x;
	calibrate_request.point_cloud_scale_y           = req.point_cloud_scale_y;

	if (!onCalibrate(calibrate_request, calibrate_response)) {
		return false;
	}

	res.transform = calibrate_response.transform;
	return true;
}


/// Calibrates the camera given the information in the request.
bool CameraPoseCalibrationNode::onCalibrate(dr_msgs::Calibrate::Request & req, dr_msgs::Calibrate::Response & res) {
	DR_INFO("Received calibration request from '" << req.cloud.header.frame_id << "' to '" << req.target_frame << "'.");

	// extract image
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		DR_ERROR("Failed to convert sensor_msgs/Image to cv::Mat. Error: " << e.what());
		return false;
	}
	cv::Mat image = cv_ptr->image;

	// extract pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(req.cloud, *cloud);
	if (req.cloud.header.frame_id == "") {
		DR_ERROR("Found empty frame_id in given pointcloud.");
		return false;
	}

	if (req.point_cloud_scale_x == 0) req.point_cloud_scale_x = 1;
	if (req.point_cloud_scale_y == 0) req.point_cloud_scale_y = 1;

	// find calibration plate in image
	std::shared_ptr<dr::CalibrationInformation> debug_information(new dr::CalibrationInformation);
	Eigen::Isometry3d camera_to_tag;
	try {
		camera_to_tag = dr::findCalibrationIsometry(
			image,
			cloud,
			req.pattern_width,
			req.pattern_height,
			req.pattern_distance,
			req.neighbor_distance,
			req.valid_pattern_ratio_threshold,
			req.point_cloud_scale_x,
			req.point_cloud_scale_y,
			debug_information
		);
	} catch (std::exception const & e) {
		DR_ERROR("Failed to find isometry. Error: " << e.what());
		detected_pattern_publisher.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
		return false;
	}

	// get the current time for publishing stamps
	ros::Time now = req.cloud.header.stamp;

	// find transformation from tag frame to the frame to which we want to calibrate
	Eigen::Isometry3d tag_to_target;
	if (req.tag_frame == req.target_frame) {
		tag_to_target = Eigen::Isometry3d::Identity();
	} else {
		try {
			tf::StampedTransform transform;
			if (!transform_listener.waitForTransform(req.target_frame, req.tag_frame, now, ros::Duration(1.0))) {
				DR_ERROR("Failed to find transform from " << req.tag_frame << " to " << req.target_frame);
				return false;
			}
			transform_listener.lookupTransform(req.target_frame, req.tag_frame, now, transform);

			tf::transformTFToEigen(transform, tag_to_target);
		} catch (tf::TransformException const & e) {
			DR_ERROR("Failed to find transform. Error: " << e.what());
			return false;
		}
	}

	// transform for target to camera frame
	Eigen::Isometry3d camera_to_target = tag_to_target * camera_to_tag;

	// clone image and draw detection
	cv::Mat detected_pattern = image.clone();
	cv::drawChessboardCorners(detected_pattern, cv::Size(req.pattern_width, req.pattern_height), cv::Mat(debug_information->image_points), true);

	// transform the target (model) to the target frame to verify a correct calibration
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_target_cloud(new pcl::PointCloud<pcl::PointXYZ>(*debug_information->target_cloud));
	pcl::transformPointCloud(*debug_information->target_cloud, *transformed_target_cloud, Eigen::Affine3d(camera_to_tag.inverse()));

	// copy result to response
	tf::transformEigenToMsg(camera_to_target, res.transform);

	// publish result if necessary
	calibrated = true;
	tf::Transform tf_camera_to_target;
	transformEigenToTF(camera_to_target, tf_camera_to_target);
	calibration_transform = tf::StampedTransform(tf_camera_to_target, now, req.target_frame, cloud->header.frame_id);
	if (publish_transform) {
		transform_broadcaster.sendTransform(calibration_transform);
	}

	// copy headers to publishing pointclouds
	pcl_conversions::toPCL(now, cloud->header.stamp);
	transformed_target_cloud->header                  = cloud->header;
	debug_information->projected_source_cloud->header = cloud->header;
	debug_information->source_cloud->header           = cloud->header;
	debug_information->target_cloud->header           = cloud->header;
	debug_information->target_cloud->header.frame_id  = req.tag_frame;

	// publish calibration plane
	visualization_msgs::Marker calibration_plane = createCalibrationPlaneMarker(debug_information->projected_source_cloud, req.pattern_width, req.pattern_height);
	calibration_plane.header.stamp = now;
	calibration_plane_marker_publisher.publish(calibration_plane);

	// publish the debug information
	cloud_publisher.publish(cloud);
	target_cloud_publisher.publish(debug_information->target_cloud);
	transformed_target_cloud_publisher.publish(transformed_target_cloud);
	source_cloud_publisher.publish(debug_information->source_cloud);
	projected_source_cloud_publisher.publish(debug_information->projected_source_cloud);
	detected_pattern_publisher.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", detected_pattern).toImageMsg());

	DR_INFO(cloud->header.frame_id << " to " << req.tag_frame << " :\n" << camera_to_tag.matrix() );
	DR_INFO(cloud->header.frame_id << " to " << req.target_frame << " :\n" << camera_to_target.matrix() );

	return true;
}


}

