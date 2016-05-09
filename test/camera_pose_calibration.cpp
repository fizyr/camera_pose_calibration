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

#include "camera_pose_calibration_impl.hpp"
#include <gtest/gtest.h>

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace camera_pose_calibration {

/// Test if the isometry between two equal point clouds returns the identity transformation
TEST(CameraPoseCalibration, findIsometry_identity) {
	// Define cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->push_back(pcl::PointXYZ(0,0,0));
	cloud->push_back(pcl::PointXYZ(0,1,0));
	cloud->push_back(pcl::PointXYZ(0,0,1));

	// Calculate expected result and actual result
	Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d actual = findIsometry(cloud, cloud);

	for (int row=0; row<actual.matrix().rows(); row++) {
		for (int col=0; col<actual.matrix().cols(); col++) {
			EXPECT_NEAR(expected(row,col), actual(row,col), 6E-8);
		}
	}
}

/// Test if the isometry between two translated point clouds returns the translation transformation
TEST(CameraPoseCalibration, findIsometry_translation) {
	// Define source
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	source->push_back(pcl::PointXYZ(0,0,0));
	source->push_back(pcl::PointXYZ(1,0,0));
	source->push_back(pcl::PointXYZ(0,1,0));
	source->push_back(pcl::PointXYZ(0,0,1));

	// Define target
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	target->push_back(pcl::PointXYZ(0,0,2));
	target->push_back(pcl::PointXYZ(1,0,2));
	target->push_back(pcl::PointXYZ(0,1,2));
	target->push_back(pcl::PointXYZ(0,0,3));

	// Expected result
	Eigen::Isometry3d expected = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0,0,2);

	// Calculate isometry with function to be unit tested
	Eigen::Isometry3d actual = findIsometry(source, target);

	for (int row=0; row<actual.matrix().rows(); row++) {
		for (int col=0; col<actual.matrix().cols(); col++) {
			EXPECT_NEAR(expected(row,col), actual(row,col), 6E-8);
		}
	}
}

}
