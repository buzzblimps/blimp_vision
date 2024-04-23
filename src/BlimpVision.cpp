#include "BlimpVision.hpp"
#include <iomanip>

using std::placeholders::_1;
using namespace std::chrono_literals;

// blimp namespace / targets
// Float64MultiArray [4] = [x (pixels), y (pixels), z (m)]
// If no target, still send message with z=1000

// goalColor,       Int64   /goal_color      orange=0, yellow=1
// autonomousState  Int64   /state_machine   same autoState enum

BlimpVision::BlimpVision() : Node("blimp_vision_node"), frame_count_(0) {
    RCLCPP_INFO(this->get_logger(), "Initializing Blimp Vision Node");

    this->declare_parameter<bool>("use_img_sub", false);
    use_img_sub_ = this->get_parameter("use_img_sub").as_bool();

    //Load camera configuration
    this->declare_parameter<std::string>("video_device", "/dev/video0");
    this->declare_parameter<int>("image_width", 2560);
    this->declare_parameter<int>("image_height", 960);

    std::string video_device = this->get_parameter("video_device").as_string();
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();

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

    // Get debug_imshow
    this->declare_parameter<bool>("imshow");
    debug_imshow_ = this->get_parameter("imshow").as_bool();

    // Get depth rate
    this->declare_parameter<double>("depth_rate");
    depth_rate_ = this->get_parameter("depth_rate").as_double();

    //Initialize computer vision processing object with left and right camera infos
    computer_vision_.init(cinfo_left_, cinfo_right_, debug_imshow_);

    //Open capture device
    cap_.open(video_device, cv::CAP_V4L);

    //Set the stereo cam to desired resolution
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    cap_.set(cv::CAP_PROP_FPS, 30);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    one_hz_timer_ = this->create_wall_timer(1000ms, std::bind(&BlimpVision::one_hz_timer_callback, this));

    if (use_img_sub_) {
        comp_img_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 1, std::bind(&BlimpVision::compressed_image_callback, this, _1));
    } else {
        camera_timer_ = this->create_wall_timer(1ns, std::bind(&BlimpVision::camera_timer_callback, this));
    }

    z_estimation_timer = this->create_wall_timer(1s * 1.0/depth_rate_, std::bind(&BlimpVision::z_estimation_timer_callback, this));

    goal_color_subscriber_ = this->create_subscription<std_msgs::msg::Int64>("goal_color", 1, std::bind(&BlimpVision::goal_color_subscription_callback, this, _1));
    state_machine_subscriber_ = this->create_subscription<std_msgs::msg::Int64>("state_machine", 1, std::bind(&BlimpVision::state_machine_subscription_callback, this, _1));

    targets_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("targets", 1);

    startTime_ = this->now();
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
    if(debug_imshow_) cv::imshow("Left Unrect", left_frame);
    if(debug_imshow_) cv::imshow("Right Unrect", right_frame);

    if(debug_imshow_) cv::imshow("Left Rect", left_rect);
    if(debug_imshow_) cv::imshow("Right Rect", right_rect);

    if(state_machine_ == searching || state_machine_ == approach || state_machine_ == catching){
        float ball_x, ball_y;
        bool success = computer_vision_.estimateBallLeftXY(left_rect, right_rect, ball_x, ball_y);
        if(success){
            cv::Mat left_rect_ball;
            left_rect.copyTo(left_rect_ball);
            cv::circle(left_rect_ball, cv::Point2f(ball_x, ball_y), 10, cv::Scalar(0, 0, 255), -1);
            if(debug_imshow_) cv::imshow("Left Rect Circle", left_rect_ball);
        }

        // PUBLISH ball_x, ball_y, ball_z_
        std_msgs::msg::Float64MultiArray targets_msg;
        targets_msg.data.clear();
        if(success){
            targets_msg.data.push_back(ball_x);
            targets_msg.data.push_back(ball_y);
            targets_msg.data.push_back(ball_z_);
        }else{
            targets_msg.data.push_back(0);
            targets_msg.data.push_back(0);
            targets_msg.data.push_back(1000);
        }
        targets_publisher_->publish(targets_msg);

    } else if(state_machine_ == goalSearch || state_machine_ == approachGoal || state_machine_ == scoringStart || state_machine_ == shooting){
        // float goal_x, goal_y, goal_z;
        // bool success = computer_vision_.estimateGoalLeftXY(left_rect, right_rect, goal_x, goal_y, goal_z, goal_color_);

        // // PUBLISH goal_x, goal_y, goal_z_
        // std_msgs::msg::Float64MultiArray targets_msg;
        // targets_msg.data.clear();
        // if(success){
        //     targets_msg.data.push_back(goal_x);
        //     targets_msg.data.push_back(goal_y);
        //     targets_msg.data.push_back(goal_z);
        // }else{
        //     targets_msg.data.push_back(0);
        //     targets_msg.data.push_back(0);
        //     targets_msg.data.push_back(1000);
        // }
        // targets_publisher_->publish(targets_msg);
    }

    cv::waitKey(1);
    frame_count_++;
}

void BlimpVision::z_estimation_timer_callback() {
    if(state_machine_ == searching || state_machine_ == approach || state_machine_ == catching){
        bool success = computer_vision_.estimateBallZ(ball_z_);
        std::cout << "ball_z_: " << ball_z_ << std::endl;
    } else if(state_machine_ == goalSearch || state_machine_ == approachGoal || state_machine_ == scoringStart || state_machine_ == shooting){
        // bool success = computer_vision_.estimateGoalZ(goal_z_);
        // std::cout << "goal_z_: " << goal_z_ << std::endl;
    }
}

void BlimpVision::goal_color_subscription_callback(const std_msgs::msg::Int64::SharedPtr goal_color_msg){
    try{
        goal_color_ = static_cast<goalType>(goal_color_msg->data);
    }catch(const std::exception){
        std::cout << "Recieved invalid goal_color value (" << goal_color_msg->data << ")." << std::endl;
    }
}

void BlimpVision::state_machine_subscription_callback(const std_msgs::msg::Int64::SharedPtr state_machine_msg){
    try{
        state_machine_ = static_cast<autoState>(state_machine_msg->data);
    }catch(const std::exception){
        std::cout << "Recieved invalid state_machine value (" << state_machine_msg->data << ")." << std::endl;
    }
}

void BlimpVision::compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr comp_img_msg) {
    frame_count_++;
}