/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include <rclcpp/rclcpp.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
// #include <sensor_msgs/srv/set_camera_info.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

// C includes
#include <cstring>

// C++ includes
#include <algorithm>
#include <array>
#include <atomic>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <condition_variable>
#include <mutex>

// Seek SDK includes
#include "seekcamera/seekcamera.h"
#include "seekcamera/seekcamera_manager.h"
#include "seekframe/seekframe.h"

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

namespace seek_wrapper {

class SeekWrapper
{
  public:
    SeekWrapper(const std::shared_ptr<rclcpp::Node> nh);
    
    ~SeekWrapper();

    // bool setCameraInfo(sensor_msgs::srv::SetCameraInfo::Request& req, sensor_msgs::srv::SetCameraInfo::Response& resp);

    // Used only in offline testing from Seek bag
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr &image);

  private:
    std::shared_ptr<rclcpp::Node> nh_;

    // Handles frame available events.
    void handle_camera_frame_available(seekcamera_t *camera, seekcamera_frame_t *camera_frame, void *user_data);

    // Handles camera connect events.
    void handle_camera_connect(seekcamera_t *camera, seekcamera_error_t event_status, void *user_data);

    // Handles camera disconnect events.
    void handle_camera_disconnect(seekcamera_t *camera, seekcamera_error_t event_status, void *user_data);

    // Handles camera error events.
    void handle_camera_error(seekcamera_t *camera, seekcamera_error_t event_status, void *user_data);

    // Handles camera ready to pair events
    void handle_camera_ready_to_pair(seekcamera_t *camera, seekcamera_error_t event_status, void *user_data);

    // Callback function for the camera manager; it fires whenever a camera event occurs.
    void camera_event_callback(seekcamera_t *camera, seekcamera_manager_event_t event, seekcamera_error_t event_status, void *user_data);

    seekcamera_manager_t *manager_ = nullptr;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // Used in offline testing from Seek bag
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermal_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    // rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_info_service_;

    bool calibration_mode_ = false;
    bool offline_ = false;
    
    sensor_msgs::msg::CameraInfo camera_info_;
};

}