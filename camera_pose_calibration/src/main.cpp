#include "node.hpp"

int main(int argc, char * argv[]) {
	ros::init(argc, argv, ROS_PACKAGE_NAME);
	camera_pose_calibration::CameraPoseCalibrationNode node;
	ROS_INFO_STREAM(ROS_PACKAGE_NAME << " node initialized.");
	ros::spin();
}
