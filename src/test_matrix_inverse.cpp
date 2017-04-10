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

// Test whether matrix inversion is correct.

#include "ros/ros.h"
//#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_matrix_inversion");
  ros::NodeHandle n;

  Eigen::Isometry3d matrix;

  matrix = Eigen::Translation3d(1., 1., 1.)*Eigen::Quaterniond(0.92971116937058, 0.1766746401234857, -0.22478416240547922, 0.23215359875840932);

  ROS_INFO_STREAM("Original matrix: " << std::endl << matrix.matrix() );
  ROS_INFO_STREAM("Inverted matrix (method 1): " << std::endl << matrix.inverse().matrix() );
  ROS_INFO_STREAM("Inverted matrix (method 2): " << std::endl << matrix.matrix().inverse() );

  return 0;
}
