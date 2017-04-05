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

// An example of cablibrating camera position from a live camera stream.

#include "camera_pose_calibration/CalibrateFile.h"
#include "geometry_msgs/Pose.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "string.h"
#include "tf/transform_datatypes.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_cam_cal");

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<camera_pose_calibration::CalibrateFile>("calibrate_file");

  camera_pose_calibration::CalibrateFile srv;
  
  // Get the filepaths to image & pointcloud datafiles
  std::string path = ros::package::getPath("camera_pose_calibration");

  // Fill out the service request
  srv.request.cloud = path + "/data/example/cal.pcd";
  srv.request.image = path + "/data/example/cal.jpg";
  srv.request.tag_frame = "calibration_circles_frame"; // This is defined in a URDF. It's fixed
  srv.request.camera_frame = "camera_ee_depth_frame"; // Unknown pose. Find its relation to target_frame
  srv.request.target_frame = "right_ur5_r200_link"; // This is defined in a URDF. It's fixed
  srv.request.point_cloud_scale_x = 1;
  srv.request.point_cloud_scale_y = 1;

  camera_pose_calibration::PatternParameters circle_pattern;
  // See http://answers.ros.org/question/258014/parameters-for-camera_pose_calibration/
  circle_pattern.pattern_width = 4; // 4 circles wide
  circle_pattern.pattern_height = 11; // 11 circles long
  circle_pattern.pattern_distance = 0.02; // Distance between circle centers. Can be in [m] or [cm], as long as all are consistent
  circle_pattern.neighbor_distance = 0.001; // Distance between pixels of image
  circle_pattern.valid_pattern_ratio_threshold = 0.00001; // Min. ratio of valid pixels to NaN's
  srv.request.pattern = circle_pattern;

  if (!client.call(srv))
  {
    ROS_ERROR("Failed to call service calibrate_file");
    return 1;
  }

  // Normalize the quaternion. Would be nice if srv.response.transform.rotation.normalize() worked
  tf::Quaternion quat(srv.response.transform.rotation.x, srv.response.transform.rotation.y, srv.response.transform.rotation.z, srv.response.transform.rotation.z);

  quat.normalize();

  ROS_INFO_STREAM("Camera translation: " << std::endl << srv.response.transform.translation );
  ROS_INFO_STREAM("Camera rotation (quaternion): " << std::endl << quat.x() <<"  "<< quat.y() <<"  "<< quat.z() <<"  "<< quat.w() );

  return 0;
}
