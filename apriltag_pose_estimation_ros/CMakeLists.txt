cmake_minimum_required(VERSION 2.8)
project(apriltag_pose_estimation_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    image_transport
    geometry_msgs
    nav_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
endif()

find_package(apriltag REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${apriltag_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
/usr/local/include/apriltag
${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp 
    sensor_msgs 
    std_msgs
    geometry_msgs
    cv_bridge
    image_transport
    nav_msgs
)

add_library(${PROJECT_NAME} SHARED
${apriltag_LIBRARIES}
src/apriltag_pose_rospipeline.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
/usr/local/lib/libapriltag.so
/usr/local/lib/libapriltag.so.3
/usr/local/lib/libapriltag.so.3.3.0
)

add_executable(apriltag_pose
src/apriltag_pose_rospipeline.cpp)
target_link_libraries(apriltag_pose 
${PROJECT_NAME}
${catkin_LIBRARIES}
)



