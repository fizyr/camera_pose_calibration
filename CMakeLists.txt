cmake_minimum_required(VERSION 2.8.3)
project(camera_pose_calibration)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	pcl_ros
	cv_bridge
	cmake_modules
	eigen_conversions
	sensor_msgs
	image_transport
	tf
	tf_conversions
	visualization_msgs
	nodelet
	dr_base
	message_generation
)

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
	DEPENDENCIES geometry_msgs sensor_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
)

dr_include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/camera_pose_calibration.cpp)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
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


install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
