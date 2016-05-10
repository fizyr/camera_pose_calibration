cmake_minimum_required(VERSION 2.8.3)
project(camera_pose_calibration)

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	cv_bridge
	eigen_conversions
	image_transport
	message_generation
	nodelet
	tf
	tf_conversions
	pcl_ros
	roscpp
	sensor_msgs
	visualization_msgs
)

find_package(Boost REQUIRED COMPONENT chrono thread)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_message_files(
	DIRECTORY msg
)

add_service_files(
	DIRECTORY srv
)

generate_messages(
	DEPENDENCIES sensor_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
)

include_directories(
	include/${PROJECT_NAME}
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/camera_pose_calibration.cpp)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${Eigen_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME}_nodelet SHARED src/nodelet.cpp src/node.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet
	${PROJECT_NAME}
)
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet)

catkin_add_gtest(${PROJECT_NAME}_test test/camera_pose_calibration.cpp)
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
