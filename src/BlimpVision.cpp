#include "BlimpVision.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

BlimpVision::BlimpVision() : Node("blimp_vision_node"), frame_count_(0) {
    RCLCPP_INFO(this->get_logger(), "Initializing Blimp Vision Node");

    // it_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this()); // https://answers.ros.org/question/353828/getting-a-nodesharedptr-from-this/
    // it_img_sub_ = std::make_shared<image_transport::Subscriber>(it_->subscribe("image_raw", 1, std::bind(&BlimpVision::image_callback, this, std::placeholders::_1)));
    // comp_img_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 1, std::bind(&BlimpVision::compressed_image_callback, this, _1));
    // it_.subscribe("image_raw", 1, std::bind(&BlimpVision::image_callback, this, std::placeholders::_1));
    // sub_camera_ = image_transport::create_camera_subscription(this, "image_raw", std::bind(&BlimpVision::image_callback, this, std::placeholders::_1, std::placeholders::_2), "raw");

    //Load camera configuration
    this->declare_parameter<std::string>("video_device", "/dev/video0");
    this->declare_parameter<int>("image_width", 2560);
    this->declare_parameter<int>("image_height", 960);

    std::string video_device = this->get_parameter("video_device").as_string();
    int image_width = this->get_parameter("image_width").as_int();
    int image_height = this->get_parameter("image_height").as_int();

    //Load camera calibration files
    this->declare_parameter<std::string>("camera_id", "camera1");
    std::string camera_id = this->get_parameter("camera_id").as_string();

    std::string cinfo_left_path = "package://blimp_vision/calibration/" + camera_id + "_elp_left.yaml";
    std::string cinfo_right_path = "package://blimp_vision/calibration/" + camera_id + "_elp_right.yaml";

    cinfomgr_left_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "elp_left");
    cinfomgr_right_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "elp_right");
    cinfomgr_left_->loadCameraInfo(cinfo_left_path);
    cinfomgr_right_->loadCameraInfo(cinfo_right_path);
    cinfo_left_ = cinfomgr_left_->getCameraInfo();
    cinfo_right_ = cinfomgr_right_->getCameraInfo();

    //Precompute camera calibration maps
    cv::Size size = cv::Size(cinfo_left_.width, cinfo_left_.height);

    cv::Matx33d K_left(cinfo_left_.k[0], cinfo_left_.k[1], cinfo_left_.k[2],
                cinfo_left_.k[3], cinfo_left_.k[4], cinfo_left_.k[5],
                cinfo_left_.k[6], cinfo_left_.k[7], cinfo_left_.k[8]);

    cv::Matx33d R_left(cinfo_left_.r[0], cinfo_left_.r[1], cinfo_left_.r[2],
                cinfo_left_.r[3], cinfo_left_.r[4], cinfo_left_.r[5],
                cinfo_left_.r[6], cinfo_left_.r[7], cinfo_left_.r[8]);

    cv::Matx34d P_left(cinfo_left_.p[0], cinfo_left_.p[1], cinfo_left_.p[2], cinfo_left_.p[3],
                cinfo_left_.p[4], cinfo_left_.p[5], cinfo_left_.p[6], cinfo_left_.p[7],
                cinfo_left_.p[8], cinfo_left_.p[9], cinfo_left_.p[10], cinfo_left_.p[11]);

    cv::Mat D_left(1, 5, CV_32F);
    D_left.at<float>(0, 0) = cinfo_left_.d[0];
    D_left.at<float>(0, 1) = cinfo_left_.d[1];
    D_left.at<float>(0, 2) = cinfo_left_.d[2];
    D_left.at<float>(0, 3) = cinfo_left_.d[3];
    D_left.at<float>(0, 4) = cinfo_left_.d[4];

    //We only need maps 1 & 2
    cv::initUndistortRectifyMap(K_left, D_left, R_left, P_left, size, CV_16SC2, map_1_left_, map_2_left_);

    cv::Matx33d K_right(cinfo_right_.k[0], cinfo_right_.k[1], cinfo_right_.k[2],
                cinfo_right_.k[3], cinfo_right_.k[4], cinfo_right_.k[5],
                cinfo_right_.k[6], cinfo_right_.k[7], cinfo_right_.k[8]);

    cv::Matx33d R_right(cinfo_right_.r[0], cinfo_right_.r[1], cinfo_right_.r[2],
                cinfo_right_.r[3], cinfo_right_.r[4], cinfo_right_.r[5],
                cinfo_right_.r[6], cinfo_right_.r[7], cinfo_right_.r[8]);

    cv::Matx34d P_right(cinfo_right_.p[0], cinfo_right_.p[1], cinfo_right_.p[2], cinfo_right_.p[3],
                cinfo_right_.p[4], cinfo_right_.p[5], cinfo_right_.p[6], cinfo_right_.p[7],
                cinfo_right_.p[8], cinfo_right_.p[9], cinfo_right_.p[10], cinfo_right_.p[11]);

    cv::Mat D_right(1, 5, CV_32F);
    D_right.at<float>(0, 0) = cinfo_right_.d[0];
    D_right.at<float>(0, 1) = cinfo_right_.d[1];
    D_right.at<float>(0, 2) = cinfo_right_.d[2];
    D_right.at<float>(0, 3) = cinfo_right_.d[3];
    D_right.at<float>(0, 4) = cinfo_right_.d[4];

    //We only need maps 1 & 2
    cv::initUndistortRectifyMap(K_right, D_right, R_right, P_right, size, CV_16SC2, map_1_right_, map_2_right_);

    //Todo: move this to parameter
    rect_interpolation_ = cv::INTER_LANCZOS4;

    //Initialize computer vision processing object with left and right camera infos
    computer_vision_.init(cinfo_left_, cinfo_right_);

    // model_.fromCameraInfo(cinfo_left_, cinfo_right_);

    // std::cout << model_.baseline() << std::endl;
    // cv::Matx44d Q = model_.reprojectionMatrix();

    //Compute stereo reprojection matrix Q
  
    // double cx = model_.left().cx();
    // double cy = model_.left().cy();
    // double fx = model_.left().fx();
    // double fy = model_.left().fy();
    // double cxr = model_.right().cx();

    // cv::Matx44d Q;
    // std::cout << Q << std::endl;

    //Open capture device
    cap_.open(video_device, cv::CAP_V4L);

    //Set the stereo cam to desired resolution
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);

    // image_subscriber_ = image_transport::create_subscription(this, "image_raw", std::bind(&BlimpVision::image_callback, this, std::placeholders::_1), "raw");
    // image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 1, std::bind(&BlimpVision::image_callback, this, std::placeholders::_1));

    // return;

    one_hz_timer_ = this->create_wall_timer(1000ms, std::bind(&BlimpVision::one_hz_timer_callback, this));
    camera_timer_ = this->create_wall_timer(1ns, std::bind(&BlimpVision::camera_timer_callback, this));
}

BlimpVision::~BlimpVision() {
    cap_.release();
}

void BlimpVision::one_hz_timer_callback() {
    RCLCPP_INFO(this->get_logger(), "%d frames/second", frame_count_);
    frame_count_ = 0;
}

void BlimpVision::camera_timer_callback() {
    if(!cap_.grab()) return;

    cv::Mat sync_frame;
    cap_.retrieve(sync_frame);

    //Split into left and right images
    cv::Rect left_roi(0, 0, sync_frame.cols/2, sync_frame.rows);
    cv::Rect right_roi(sync_frame.cols/2, 0, sync_frame.cols/2, sync_frame.rows);
    cv::Mat left_frame(sync_frame, left_roi);
    cv::Mat right_frame(sync_frame, right_roi);

    //Rectify left and right
    cv::Mat left_rect, right_rect;
    cv::remap(left_frame, left_rect, map_1_left_, map_2_left_, rect_interpolation_, cv::BORDER_CONSTANT, 0);
    cv::remap(right_frame, right_rect, map_1_right_, map_2_right_, rect_interpolation_, cv::BORDER_CONSTANT, 0);

    //Display left and right rectified frames
    cv::imshow("Left Rect", left_rect);
    cv::imshow("Right Rect", right_rect);


    autoState mode = searching;
    goalType goalColor = orange;
    computer_vision_.update(left_rect, right_rect, mode, goalColor);

    std::vector<std::vector<float>> target;
    target = computer_vision_.getTargetBalloon();

    // cv::imshow("Left", left_frame);

    cv::waitKey(1);

    frame_count_++;
}

