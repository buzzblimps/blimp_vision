#ifndef BLIMP_VISION_HPP
#define BLIMP_VISION_HPP

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

//OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include <libv4l2.h>

#include "ComputerVision.hpp"
#include "VideoSaver.hpp"

class BlimpVision : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr comp_img_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr goal_color_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr state_machine_subscriber_;
    rclcpp::TimerBase::SharedPtr one_hz_timer_, camera_timer_, z_estimation_timer;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr targets_publisher_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfomgr_left_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfomgr_right_;

    sensor_msgs::msg::CameraInfo cinfo_left_;
    sensor_msgs::msg::CameraInfo cinfo_right_;

    cv::VideoCapture cap_;
    cv::Mat map_1_left_, map_2_left_;
    cv::Mat map_1_right_, map_2_right_;

    bool save_video_ = false;
    VideoSaver videoSaver_;

    int image_width_, image_height_;

    int rect_interpolation_;
    int frame_count_;

    bool use_img_sub_;
    bool debug_imshow_ = false;
    float depth_rate_ = 1.0;

    ComputerVision computer_vision_;
    // autoState state_machine_ = searching;
    autoState state_machine_ = searching;
    goalType goal_color_ = yellow;

    float ball_z_ = 1000;
    float goal_z_ = 1000;

    void one_hz_timer_callback();
    void camera_timer_callback();
    void z_estimation_timer_callback();
    rclcpp::Time startTime_;

    void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr comp_img_msg);
    void goal_color_subscription_callback(const std_msgs::msg::Int64::SharedPtr goal_color_msg);
    void state_machine_subscription_callback(const std_msgs::msg::Int64::SharedPtr state_machine_msg);

    // void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg);
    // void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);
public:
    BlimpVision();
    ~BlimpVision();
};

#endif
