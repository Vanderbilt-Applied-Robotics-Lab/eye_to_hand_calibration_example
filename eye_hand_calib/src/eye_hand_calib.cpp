#include <eye_hand_calib/eye_hand_calib.h>

EyeHandCalib::EyeHandCalib() : Node("hand_eye_calib")
{
    // IMPORTANT:
    // Replace these placeholders with the camera intrinsics
    // and distortion coefficients from the original dataset.
    K_ = (cv::Mat_<double>(3, 3) <<
        900.0, 0.0, 640.0,
        0.0, 900.0, 360.0,
        0.0, 0.0, 1.0);

    D_ = cv::Mat::zeros(5, 1, CV_64F);

    initializeCheckerboard();
    initializeRobotPoses();
}

void EyeHandCalib::initializeCheckerboard()
{
    object_points_.clear();

    for (int row = 0; row < board_size_.height; ++row)
    {
        for (int col = 0; col < board_size_.width; ++col)
        {
            object_points_.emplace_back(
                col * square_size_,
                row * square_size_,
                0.0f);
        }
    }
}

void EyeHandCalib::initializeRobotPoses()
{
    // Original dataset transformations: ^E T_B
    // Translation values converted from mm to m.

    std::vector<cv::Mat> R_original = {
        (cv::Mat_<double>(3, 3) <<
            -0.178668,  0.983906, -0.002544,
             0.104436,  0.016394, -0.994396,
            -0.978351, -0.177932, -0.105684),

        (cv::Mat_<double>(3, 3) <<
            -0.098599,  0.995124, -0.002556,
             0.114587,  0.008802, -0.993374,
            -0.988508, -0.098238, -0.114896),

        (cv::Mat_<double>(3, 3) <<
             0.092825,  0.995679, -0.002556,
             0.114164, -0.013193, -0.993374,
            -0.989116,  0.091918, -0.114896)
    };

    std::vector<cv::Mat> t_original = {
        (cv::Mat_<double>(3, 1) <<
            -0.525397105,
             0.907647285,
            -0.011588263),

        (cv::Mat_<double>(3, 1) <<
            -0.285919922,
             0.937671026,
            -0.267437553),

        (cv::Mat_<double>(3, 1) <<
             0.288350222,
             0.936404190,
            -0.269260733)
    };

    R_base_ee_.clear();
    t_base_ee_.clear();

    // Convert ^E T_B to ^B T_E
    for (size_t i = 0; i < R_original.size(); ++i)
    {
        cv::Mat R_inv = R_original[i].t();
        cv::Mat t_inv = -R_inv * t_original[i];

        R_base_ee_.push_back(R_inv);
        t_base_ee_.push_back(t_inv);
    }
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
        object_points_,
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

    // Store ^C T_T
    R_target_cam_.push_back(R);
    t_target_cam_.push_back(tvec);

    // Store ^B T_E directly.
    // These transformations were already inverted
    // in initializeRobotPoses().
    R_ee_base_.push_back(R_base_ee_[index]);
    t_ee_base_.push_back(t_base_ee_[index]);

    return true;
}

void EyeHandCalib::calibrate()
{
    R_ee_base_.clear();
    t_ee_base_.clear();
    R_target_cam_.clear();
    t_target_cam_.clear();

    for (int i = 0; i < NUM_IMAGES; ++i)
    {
        if (!processImage(i))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Calibration aborted: image processing failed.");

            return;
        }
    }

    if (R_ee_base_.size() < 3)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Insufficient calibration poses.");

        return;
    }

    // Eye-in-hand calibration for original dataset
    // Input:  ^B T_E and ^C T_T
    // Output: ^E T_C

    cv::calibrateHandEye(
        R_ee_base_,
        t_ee_base_,
        R_target_cam_,
        t_target_cam_,
        R_base_cam_,
        t_base_cam_,
        cv::CALIB_HAND_EYE_PARK);

    printResults();
}

void EyeHandCalib::printResults()
{
    std::cout << "Camera rotation relative to end effector:\n"
              << R_base_cam_ << std::endl;

    std::cout << "Camera translation relative to end effector:\n"
              << t_base_cam_ << std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<EyeHandCalib>();

    node->calibrate();

    rclcpp::shutdown();

    return 0;
}