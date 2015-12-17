#include "node.hpp"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace camera_pose_calibration {

class CameraPoseCalibrationNodelet : public nodelet::Nodelet {
public:
	virtual void onInit() {
		NODELET_DEBUG("Initializing nodelet...");
	};

private:
	CameraPoseCalibrationNode circles_calibration_node;
};

}

PLUGINLIB_EXPORT_CLASS(camera_pose_calibration::CameraPoseCalibrationNodelet, nodelet::Nodelet)
