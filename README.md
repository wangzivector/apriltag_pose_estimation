# Apriltag Pose Estimation: Quick-setup for ROS-image-topic <u>_or_</u> USB-Cam
This repository contains two _individual_ implementations to estimate pose(6 DoFs) of Apriltags, using _ROS_ <u>or</u> _CMake_ .
Two folders are two **individal** versions, one for **ROS with image topic**, another for **Cmake with OpenCV**. 

## Feature
1. Two versions for ROS wrapper or Cmake tool:
   - [apriltag_pose_estimation_ros](./apriltag_pose_estimation_ros): **ROS with image topic**, just simply specify the ros_image_topic inside .yaml file in ROS version, to get apriltag pose.
   - [apriltag_pose_estimation_cmake](./apriltag_pose_estimation_cmake): **Cmake with OpenCV**, For non-ROS users who can use Cmake with opencv/VideoCapture() to fetch images, to get apriltag pose.

2. Both projects are **C++** written with **OpenCV** and **Eigen** libraries. ROS version is tested in ROS-Melodic on Ubuntu 18.04.

3. Note that both versions are based on official library of [Apriltag](https://github.com/AprilRobotics/apriltag), which we need to make installed on system as described below. THe official library also has its pose estimation example. Here we show **hand-on/light-weight wrappers** of how to implement it.


## Usage
### 1. Dependency
   - Install <u>**Eigen3**</u> and <u>**OpenCV**</u> on Ubuntu, if dont have:
```bash
sudo apt install libeigen3-dev # eigen 3 
sudo apt install libopencv-dev # opencv
```

   - Install Official <u>**Apriltag**</u> library:
```bash
git clone https://github.com/AprilRobotics/apriltag
cd apriltag && cmake .
sudo make install
```

### 2. Complie code and Run

- Clone this respository to your machine
```bash
cd anywhere_you_wwant/
git clone https://github.com/wangzivector/apriltag_pose_estimation
```
---
#### 2.1 For **ROS user**: 
1. Copy the corresponding version folder to catkin space catkin make them, specifically:

   ```bash
   cp -r apriltag_pose_estimation/apriltag_pose_estimation_ros/  ~/catkin_ws/src
   cd ~/catkin_ws 
   catkin_make # or catkin build
   ``` 

2. Check and modify the configuration file `camera_parameter.yaml`. Include the used `tag_family` type, `tagsize`, and `camera specifications` listed in the file (Obtain camera specifications see details in Maintenance). 

   - Modify ros image topic in `camera_parameter.yaml`, the image topic should publish RGB image for rosnode to subscribe: 
      ```yaml
      camera_name: /camera/color/image_raw
      ```
    > (Of course, you need to publish the image yourself in advance(In ROS, use rospack like `cv_camera`, `usb_cam` are good). Otherwise (For those only have usb camera and use ROS), just plug the usb-cam and change the `camera_name` to `usb_cam`, like this: `camera_name: usb_cam`. It will work as the cmake version. And! it will help to publish tag-drawed images to topic `/apriltag/taged_image`, which you can view in Rviz).  

   - Modify the following yaml file directory in `apriltag_pose_rospipeline.cpp`, if needed, and re-catkin_make the package:
      ```cpp
      #define FILENAME "/home/smart/catkin_ws/src/apriltag_pose_estimation_ros/camera_parameter.yaml"
      ```

3. Run the ros node: 
```bash
rosrun apriltag_pose_estimation_ros apriltag_pose
```
    
4. There are three output topics, check the detection result using Rviz: 
    - `apriltag/pose`: 6-Dofs pose result.
    - `apriltag/odom`: Same as the `pose`, but can be viewed as fancy coordinate in Rviz.
    - `apriltag/taged_image`: Undistorted Image in which tag detection is drawed. 

---

#### 2.2 For **Cmake user**:
1. Copy the `apriltag_pose_estimation_cmake/` folder to Anywhere you want. Then Cmake and make the project:
```bash
cd Anywhere_you_want/apriltag_pose_estimation_cmake
mkdir build && cd build
cmake ..
make
```

> - Check and modify the configuration file `camera_parameter.yaml`. Include the used `tag_family` type, `tagsize`, and `camera specifications` listed in the file (Obtain camera specifications see details in Maintenance).

> - Modify the following .yaml file directory in `apriltag_pose_cmakepipeline.cpp`, if needed, and re-make this project: <br/> `#define FILENAME "./../camera_parameter.yaml"`

2. Plug the usb-cam and run the .exe in `/build`:</br>
```bash
./apriltag_pose
```
It will show the tag_drawed_images in OpenCV window and output the Pose info. in terminal. For further use, go check the code.

# Maintenance
- Camera specifications can be obtained using Matlab calibration tool, details see: [Video](https://www.youtube.com/watch?v=x6YIwoQBBxA) or [Example](https://blog.csdn.net/qq_35451572/article/details/102663396)

- the `apriltag` version in `CmakeLists.txt` may need to change for the installed version, if the cmake raise a complie error. Try to correct the line: `/usr/local/lib/libapriltag.so.3.3.0`  to your installed version.

- This resposity is heavily borrowed from [apriltag_pose_ros](https://github.com/yuannuaa/apriltag_pose_ros), which implemented apriltag pose estimation for UAV with Pose filter. We modify it to quick and easy-to-use versions, as long as you have a general camera and cmake (or ROS) enviroment. 

- We may modify the Pose filter as well in the future, as an independent module for better pose estimation. 
