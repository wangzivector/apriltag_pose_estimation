// Inherent from: https://github.com/yuannuaa/apriltag_pose_ros
#include <iostream>
#include <math.h>
#include "unistd.h"
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include "tag25h9.h"
}

#define FILENAME "./../camera_parameter.yaml"

bool DRAW_TAGED_IMAGE = true;
cv::Mat DrawTagOnImage(cv::Mat image, apriltag_detection_t *det);

int main(int argc, char *argv[])
{
    // Read Yaml file
    // https://docs.opencv.org/3.4/dd/d74/tutorial_file_input_output_with_xml_yml.html
    cv::FileStorage fs;
    fs.open(FILENAME, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        std::cout << "Failed to read Yaml file: " << FILENAME << std::endl;
        return (-1);
    }

    // Initialize tag detector params with options
    apriltag_family_t *tagfamily = NULL;
    std::string _tagfamily = std::string(fs["tag_family"]);
    const char *famname = _tagfamily.c_str();
    if (!strcmp(famname, "tag36h11")) {
        tagfamily = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
        tagfamily = tag25h9_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    // Just modify the 4th params of fun. to change the model
    bool ad_debug = false;
    bool quiet = false;
    int threads = 4; // CPU threads
    double decimate = 2.0; // Decimate input image by this factor
    double blur = 0.0;
    bool refine_edges = true; // Spend more time trying to align edges of tags

    // Create apriltag detector object
    apriltag_detector_t *apriltag_detector = apriltag_detector_create();
    apriltag_detector_add_family(apriltag_detector, tagfamily);
    apriltag_detector->quad_decimate = decimate;
    apriltag_detector->quad_sigma = blur;
    apriltag_detector->nthreads = threads;
    apriltag_detector->debug = ad_debug;
    apriltag_detector->refine_edges = refine_edges;

    // Initialize apriltag pose estimation parameter
    apriltag_detection_info_t detection_info;
    detection_info.tagsize = (double) fs["tagsize"];
    detection_info.fx = (double) fs["fx"];
    detection_info.fy = (double) fs["fy"];
    detection_info.cx = (double) fs["cx"];
    detection_info.cy = (double) fs["cy"];

    cv::Mat camera_matrix, distortion_coeff, rectif_matrix;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> distortion_coeff;
    fs["rectification_matrix"] >> rectif_matrix;

    // Open normal camera if have
    cv::VideoCapture cap(0);
    std::string camera_name = std::string(fs["camera_name"]);
    bool USE_CAP_IMAGE = cap.isOpened() && (camera_name == "usb_cam");
    
    if (!USE_CAP_IMAGE) {
        std::cout << "Failed to open usb camera." << std::endl;
        return (-1);
    }else std::cout << "Open VideoCapture cap(0) done." << std::endl;

    // Loop process initialization
    cv::Mat image_frame, undistorted_image, gray_image;

    std::cout << "Start detecting apriltag..." << std::endl;
    while (true){

        // Try to fetch a image
        cap >> image_frame;

        // Preprocess image to gray and make head
        // Make an image_u8_t header for the Mat data
        cv::undistort(image_frame, undistorted_image, camera_matrix, distortion_coeff); // rectif_matrix
        // cv::imshow("ori", image_frame);
        // cv::waitKey(0);
        // cv::imshow("ori", undistorted_image);
        // cv::waitKey(0);

        // undistorted_image = image_frame;
        cv::cvtColor(undistorted_image, gray_image, cv::COLOR_BGR2GRAY);
        
        image_u8_t image_pack = { 
            .width = gray_image.cols,
            .height = gray_image.rows,
            .stride = gray_image.cols,
            .buf = gray_image.data
        };

        // Make detection of appriltags
        zarray_t *detections = apriltag_detector_detect(apriltag_detector, &image_pack);
        // std::cout << " Tags detected nums: " << zarray_size(detections) << std::endl;
        
        // For each valid tag result
        for (int tag_ind = 0; tag_ind < zarray_size(detections); tag_ind++) {
            // Fetch single tag result 
            apriltag_detection_t *tag_single;
            zarray_get(detections, tag_ind, &tag_single);

	    	// Estimate_tag_pose.
	        detection_info.det = tag_single;
            apriltag_pose_t tag_single_pose;
	        double err = estimate_tag_pose(&detection_info, &tag_single_pose);

	        // Do something with pose.
            Eigen::Matrix<double, 3, 1> Translate_mat;
            Translate_mat <<tag_single_pose.t->data[0], tag_single_pose.t->data[1], tag_single_pose.t->data[2];

            // Get quaternion            
	        double r11 = tag_single_pose.R->data[0];  
	        double r12 = tag_single_pose.R->data[1]; 
	        double r13 = tag_single_pose.R->data[2];
	        double r21 = tag_single_pose.R->data[3];
	        double r22 = tag_single_pose.R->data[4];
	        double r23 = tag_single_pose.R->data[5];
	        double r31 = tag_single_pose.R->data[6];
	        double r32 = tag_single_pose.R->data[7];
	        double r33 = tag_single_pose.R->data[8];

            Eigen::Matrix3d Rotate_mat;
            Rotate_mat <<r11, r12, r13,
                        r21, r22, r23,
                        r31, r32, r33;
            Eigen::Quaternion<double> q_relative(Rotate_mat);
            
            // pose_msg.pose.orientation.w = q_relative.w();
            // pose_msg.pose.orientation.x = q_relative.x();
            // pose_msg.pose.orientation.y = q_relative.y();
            // pose_msg.pose.orientation.z = q_relative.z();

            // Draw each tag on colored image
            if (DRAW_TAGED_IMAGE) undistorted_image = DrawTagOnImage(undistorted_image, tag_single);

	        std::cout << "Pose: " << tag_ind << " Err: " << err 
                << "\nTral:\n" << Translate_mat << "\nRota:\n" << Rotate_mat << std::endl;
        }
        apriltag_detections_destroy(detections);

        // Draw tag image
        if (DRAW_TAGED_IMAGE) {
            cv::imshow("undistorted_image_with tags", undistorted_image);
            cv::waitKey(1);
        }
    }

    // Destroy tag family
    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tagfamily);
    } else if (!strcmp(famname, "tag25h9")) {
        tag25h9_destroy(tagfamily);
    }
    if(cap.isOpened()) cap.release();
    return 0;
}

// Draw tag on color image
cv::Mat DrawTagOnImage(cv::Mat image, apriltag_detection_t *det) {
    cv::line(image, cv::Point(det->p[0][0], det->p[0][1]),
                cv::Point(det->p[1][0], det->p[1][1]),
                cv::Scalar(0, 0xff, 0), 2);
    cv::line(image, cv::Point(det->p[0][0], det->p[0][1]),
                cv::Point(det->p[3][0], det->p[3][1]),
                cv::Scalar(0, 0, 0xff), 2);
    cv::line(image, cv::Point(det->p[1][0], det->p[1][1]),
                cv::Point(det->p[2][0], det->p[2][1]),
                cv::Scalar(0xff, 0, 0), 2);
    cv::line(image, cv::Point(det->p[2][0], det->p[2][1]),
                cv::Point(det->p[3][0], det->p[3][1]),
                cv::Scalar(0xff, 0, 0), 2);
    return image;
}



