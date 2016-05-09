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
