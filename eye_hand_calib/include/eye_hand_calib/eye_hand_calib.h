#ifndef EYE_HAND_CALIB
#define EYE_HAND_CALIB

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <vector>
#include <string>

/**
 * Example code for eye-to-hand calibration
 * Applied Robotics
 * Garrison Johnston
 * 
 * Uses data from:
 *    Tabb, A., Ahmad Yousef, K.M. 
 *    Solving the robot-world hand-eye(s) calibration problem with iterative methods. 
 *    Machine Vision and Applications 28, 569–590 (2017). 
 *    https://doi.org/10.1007/s00138-017-0841-7
 */
class EyeHandCalib : public rclcpp::Node
{
public:
    /**
     * Main constructor
     */
    EyeHandCalib();

    /**
     * Destructor
     */
    ~EyeHandCalib() = default;

    /**
     * Calibrate the camera pose 
     */
    void calibrate();
    
private:

    /**
     * sets up the checkerboard 
     */
    void initializeCheckerboard();

    /**
     * Sets up the robot poses (hardcoded for now)
     */
    void initializeRobotPoses();

    /**
     * loads and processes the image
     * @param index Image number [0, NUM_IMAGES)
     */
    bool processImage(int index);

    /**
     * Prints the results
     */
    void printResults();

    // the number of images to be processed
    static constexpr int NUM_IMAGES = 3;

    // Camera intrinsic parameters
    cv::Mat K_;
    cv::Mat D_;

    // Checkerboard corner positions
    std::vector<cv::Point3f> pattern_points_;

    // Inverted robot poses: ^G T_B
    std::vector<cv::Mat> R_gripper_base_;
    std::vector<cv::Mat> t_gripper_base_;

    // Camera measurements: ^C T_P
    std::vector<cv::Mat> R_cam_pattern_;
    std::vector<cv::Mat> t_cam_pattern_;

    // Calibration result: ^B T_C
    cv::Mat R_base_cam_;
    cv::Mat t_base_cam_;

    cv::Size board_size_ = {8, 6};
    double square_size_ = 0.0285;
};

#endif // EYE_HAND_CALIB