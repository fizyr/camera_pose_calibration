#include "camera_pose_calibration.hpp"

#include <opencv2/opencv.hpp>

#include <camera_pose_calibration/CalibrateFile.h>
#include <camera_pose_calibration/CalibrateCall.h>
#include <camera_pose_calibration/CalibrateTopic.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>


namespace camera_pose_calibration {

class CameraPoseCalibrationNode {
public:
	CameraPoseCalibrationNode();

protected:
	/// Node handle
	ros::NodeHandle node_handle;

	/// Image transport object used for publishing images.
	image_transport::ImageTransport image_transport;

	/// Image transport publisher for publishing the detected pattern.
	image_transport::Publisher detected_pattern_publisher;

	/// Listens to transforms from tf to transform calibration from the correct target to the camera.
	tf::TransformListener transform_listener;

	/// Publishes tf transforms.
	tf::TransformBroadcaster transform_broadcaster;

	/// Publishes the original pointcloud.
	ros::Publisher cloud_publisher;

	/// Publishes the expected (generated) pattern.
	ros::Publisher target_cloud_publisher;

	/// Publishes the expected pattern transformed using the found calibration transform.
	ros::Publisher transformed_target_cloud_publisher;

	/// Publishes the detected pattern from the source cloud.
	ros::Publisher source_cloud_publisher;

	/// Publishes the detected pattern but projected on a fitted plane.
	ros::Publisher projected_source_cloud_publisher;

	/// Publishes a marker for showing the plane of the calibration points.
	ros::Publisher calibration_plane_marker_publisher;

	/// Service server for calibration with all data in the service call.
	ros::ServiceServer calibrate_server_call;

	/// Service server for calibration with image and cloud from topics.
	ros::ServiceServer calibrate_server_topic;

	/// Service server for calibration with image and cloud from file.
	ros::ServiceServer calibrate_server_file;

	/// Timer to periodically republish TF transforms.
	ros::Timer tf_timer;

	/// The found calibration transformation.
	tf::StampedTransform calibration_transform;

	/// Determines whether to publish the transform over tf.
	bool publish_transform;

	/// Rate with which to publish transforms.
	double publish_rate;

	/// Determines if we have calibrated (and thus should publish the transform).
	bool calibrated;

	/// Parses ROS parameters and calls the calibration service.
	void parseInput();

	/// Create a marker for the detected calibration plane.
	visualization_msgs::Marker createCalibrationPlaneMarker(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points, uint8_t pattern_width, uint8_t pattern_height);

	/// Called when the calibrated TF transforms should be republished.
	void onTfTimeout(ros::TimerEvent const &);

	/// Calibrates and publishes the tf and debug information
	bool calibrate(
		sensor_msgs::Image const & sensor_msgs_image,
		sensor_msgs::PointCloud2 const & sensor_msgs_cloud,
		std::string const & tag_frame,
		std::string const & target_frame,
		double const & point_cloud_scale_x,
		double const & point_cloud_scale_y,
		camera_pose_calibration::PatternParameters const & pattern,
		geometry_msgs::Transform & transform
	);

	/// Calibrates the camera given the image and point cloud by a ROS topic, and all other information in the request.
	bool onCalibrateFile(camera_pose_calibration::CalibrateFile::Request & req, camera_pose_calibration::CalibrateFile::Response & res);

	/// Calibrates the camera given the image and point cloud by a ROS topic, and all other information in the request.
	bool onCalibrateTopic(camera_pose_calibration::CalibrateTopic::Request & req, camera_pose_calibration::CalibrateTopic::Response & res);

	/// Calibrates the camera given all the information in the request.
	bool onCalibrateCall(camera_pose_calibration::CalibrateCall::Request & req, camera_pose_calibration::CalibrateCall::Response & res);
};

}
