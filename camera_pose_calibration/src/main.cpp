#include "node.hpp"

int main(int argc, char * argv[]) {
	ros::init(argc, argv, ROS_PACKAGE_NAME);

	camera_pose_calibration::CameraPoseCalibrationNode node;
	ros::spin();
}
