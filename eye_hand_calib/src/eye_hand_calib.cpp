#include <eye_hand_calib/eye_hand_calib.h>

EyeHandCalib::EyeHandCalib() : Node("hand_eye_calib")
{
    K_ = (cv::Mat_<double>(3, 3) <<
        900.0, 0.0, 320.0,
        0.0, 900.0, 240.0,
        0.0, 0.0, 1.0);

    D_ = cv::Mat::zeros(5, 1, CV_64F);

    initializeCheckerboard();
    initializeRobotPoses();
}

void EyeHandCalib::initializeCheckerboard()
{
    for (int row = 0; row < board_size_.height; ++row)
    {
        for (int col = 0; col < board_size_.width; ++col)
        {
            pattern_points_.emplace_back(
                col * square_size_,
                row * square_size_,
                0.0f);
        }
    }
}

void EyeHandCalib::initializeRobotPoses()
{
    R_gripper_base_.resize(3);
    R_gripper_base_[0] =
        (cv::Mat_<double>(3, 3) <<
            -0.178668,  0.983906, -0.002544,
             0.104436,  0.016394, -0.994396,
            -0.978351, -0.177932, -0.105684);

    R_gripper_base_[1] =
        (cv::Mat_<double>(3, 3) <<
            -0.098599,  0.995124, -0.002556,
             0.114587,  0.008802, -0.993374,
            -0.988508, -0.098238, -0.114896);
     R_gripper_base_[2] =
        (cv::Mat_<double>(3, 3) <<
             0.092825,  0.995679, -0.002556,
             0.114164, -0.013193, -0.993374,
            -0.989116,  0.091918, -0.114896);

    t_gripper_base_.resize(3);
    t_gripper_base_[0] = 
        (cv::Mat_<double>(3, 1) <<
            -0.525397105,
             0.907647285,
            -0.011588263);
    t_gripper_base_[1] = 
        (cv::Mat_<double>(3, 1) <<
            -0.285919922,
             0.937671026,
            -0.267437553);
    t_gripper_base_[2] = 
        (cv::Mat_<double>(3, 1) <<
             0.288350222,
             0.936404190,
            -0.269260733);
}

bool EyeHandCalib::processImage(int index)
{
    std::string package_path =
        ament_index_cpp::get_package_share_directory(
            "eye_hand_calib");

    std::string filename =
        package_path + "/images/image" +
        std::to_string(index) + ".png";

    cv::Mat image = cv::imread(filename);

    if (image.empty())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not load image %i", index);

        return false;
    }

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Detect checkerboard corners
    std::vector<cv::Point2f> corners;

    bool found = cv::findChessboardCorners(
        gray,
        board_size_,
        corners);

    if (!found)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not find checkerboard corners in image %i",
            index);

        return false;
    }

    // Estimate checkerboard pose relative to camera
    cv::Mat rvec, tvec;

    bool success = cv::solvePnP(
        pattern_points_,
        corners,
        K_,
        D_,
        rvec,
        tvec);

    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not solve PnP for image %i",  index);

        return false;
    }

    // Convert Rodrigues vector to rotation matrix
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Store ^C T_P
    R_cam_pattern_.push_back(R);
    t_cam_pattern_.push_back(tvec);

    return true;
}

void EyeHandCalib::calibrate()
{
    for (int i = 0; i < NUM_IMAGES; ++i)
    {
        if (!processImage(i))
        {
            RCLCPP_ERROR(this->get_logger(), "Image processing failed at image %i", i);
            return;
        }
    }
    cv::calibrateHandEye(
        R_gripper_base_,
        t_gripper_base_,
        R_cam_pattern_,
        t_cam_pattern_,
        R_base_cam_,
        t_base_cam_,
        cv::CALIB_HAND_EYE_PARK);

    printResults();
}

void EyeHandCalib::printResults()
{
    std::cout << "Camera rotation relative to robot base:\n" << R_base_cam_ << std::endl;

    std::cout << "Camera translation relative to robot base:\n" << t_base_cam_ << std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<EyeHandCalib>();

    node->calibrate();

    rclcpp::shutdown();

    return 0;
}