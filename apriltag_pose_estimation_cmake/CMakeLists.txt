cmake_minimum_required(VERSION 2.8)
project(apriltag_pose_estimation_cmake)

add_compile_options(-std=c++11)

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


add_library(${PROJECT_NAME} SHARED
${apriltag_LIBRARIES}
src/apriltag_pose_cmakepipeline.cpp
)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
/usr/local/lib/libapriltag.so
/usr/local/lib/libapriltag.so.3
/usr/local/lib/libapriltag.so.3.3.0
)

add_executable(apriltag_pose
src/apriltag_pose_cmakepipeline.cpp)
target_link_libraries(apriltag_pose 
${PROJECT_NAME}
)



