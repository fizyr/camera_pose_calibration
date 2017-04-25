cmake_minimum_required(VERSION 3.1)
project(camera_pose_calibration)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED on)
set(CMAKE_CXX_EXTENSIONS off)
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	eigen_conversions
	geometry_msgs
	image_transport
	message_generation
	nodelet
	pcl_ros
	roscpp
	sensor_msgs
	tf
	tf_conversions
	visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

add_definitions(${PCL_DEFINITIONS})

add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)
generate_messages(DEPENDENCIES
	geometry_msgs
	sensor_msgs
)

catkin_package()

include_directories(
	include/${PROJECT_NAME}
SYSTEM
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${EIGEN3_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME}_nodelet SHARED
	src/node.cpp
	src/nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet
	${PROJECT_NAME}
)
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet)

add_executable(example_cam_cal src/example_cam_cal.cpp)
target_link_libraries(example_cam_cal ${PROJECT_NAME}_nodelet)

if(CATKIN_ENABLE_TESTING)
	catkin_add_gtest(${PROJECT_NAME}_test test/${PROJECT_NAME}.cpp)
	target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
