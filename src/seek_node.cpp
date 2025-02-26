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
#include <opencv2/opencv.hpp>

// R88 service
#include "messages_88/srv/record_video.hpp"


std::shared_ptr<rclcpp::Node> node_;

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // Used in offline testing from Seek bag
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermal_pub_;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
// rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_info_service_;

rclcpp::Service<messages_88::srv::RecordVideo>::SharedPtr record_service_;

bool calibration_mode_ = false;
bool offline_ = false;
bool recording_ = false;

sensor_msgs::msg::CameraInfo camera_info_;
cv::VideoWriter video_writer_;
cv::VideoWriter video_writer_thermal_;

int frame_width_;
int frame_height_;
int thermal_width_;
int thermal_height_;

// bool setCameraInfo(sensor_msgs::srv::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& resp) {
//     camera_info_ = req.camera_info;
//     resp.success = true;
//     return true;
// }

// Used only in offline testing from Seek bag
void imageCallback(const sensor_msgs::msg::Image::ConstPtr &image) {
    camera_info_.header = image->header;
    info_pub_->publish(camera_info_);
}

// Handles frame available events.
void handle_camera_frame_available(seekcamera_t *camera, seekcamera_frame_t *camera_frame, void *user_data)
{
    seekframe_t* frame = nullptr;
    seekcamera_error_t status = seekcamera_frame_get_frame_by_format(camera_frame, SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888, &frame);
    if(status != SEEKCAMERA_SUCCESS)
    {
        std::cerr << "failed to get frame: " << seekcamera_error_get_str(status) << std::endl;
        return;
    }
    frame_width_ = (int)seekframe_get_width(frame);
    frame_height_ = (int)seekframe_get_height(frame);

    seekframe_t* thermal_frame = nullptr;
    status = seekcamera_frame_get_frame_by_format(camera_frame, SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT, &thermal_frame);
    if(status != SEEKCAMERA_SUCCESS)
    {
        std::cerr << "failed to get thermal frame: " << seekcamera_error_get_str(status) << std::endl;
        return;
    }
    thermal_width_ = (int)seekframe_get_width(thermal_frame);
    thermal_height_ = (int)seekframe_get_height(thermal_frame);

    cv::Mat frame_mat(frame_height_, frame_width_, CV_8UC4, seekframe_get_data(frame));
    cv::Mat thermal_mat(thermal_height_, thermal_width_, CV_32FC1, seekframe_get_data(thermal_frame));

    seekcamera_frame_header_t* header = (seekcamera_frame_header_t*)seekframe_get_header(frame);
    uint64_t time = header->timestamp_utc_ns;
    double sec = time * 1e-9;
    rclcpp::Time t = rclcpp::Time(sec);

    cv_bridge::CvImage image_msg;
    image_msg.header.frame_id = "seek";
    image_msg.header.stamp = t;
    image_msg.encoding = sensor_msgs::image_encodings::BGRA8;
    image_msg.image    = frame_mat;
    image_pub_->publish(*(image_msg.toImageMsg()).get());

    if (recording_)
        video_writer_.write(frame_mat);

    image_msg.header.frame_id = "seek";
    image_msg.header.stamp = t;
    image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    image_msg.image    = thermal_mat;
    thermal_pub_->publish(*(image_msg.toImageMsg()).get());

    if (recording_)
        video_writer_thermal_.write(thermal_mat);

    camera_info_.header = image_msg.header;
    info_pub_->publish(camera_info_);
}

// Handles camera connect events.
void handle_camera_connect(seekcamera_t *camera, seekcamera_error_t event_status, void *user_data)
{
    (void)event_status;
    (void)user_data;
    seekcamera_chipid_t cid{};
    seekcamera_get_chipid(camera, &cid);

    // Register a frame available callback function.
    seekcamera_error_t status = seekcamera_register_frame_available_callback(camera, handle_camera_frame_available, nullptr);
    if (status != SEEKCAMERA_SUCCESS)
    {
        std::cerr << "failed to register frame callback: " << seekcamera_error_get_str(status) << std::endl;
        return;
    }

    // Start the capture session.
    status = seekcamera_capture_session_start(camera, seekcamera_frame_format_t::SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888 | seekcamera_frame_format_t::SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT);
    if (status != SEEKCAMERA_SUCCESS)
    {
        std::cerr << "failed to start capture session: " << seekcamera_error_get_str(status) << std::endl;
        return;
    }

    status = seekcamera_set_pipeline_mode(camera, SEEKCAMERA_IMAGE_SEEKVISION);
    if (status != SEEKCAMERA_SUCCESS)
    {
        std::cerr << "failed to set image pipeline: " << seekcamera_error_get_str(status) << std::endl;
        return;
    }

    // TODO decide wheter to set manual shutter mode -- prevents the gaps in footage but causes weird thermal pixels that may get worse with time
    /* status = seekcamera_set_shutter_mode(camera, SEEKCAMERA_SHUTTER_MODE_MANUAL);
    if (status != SEEKCAMERA_SUCCESS)
    {
        std::cerr << "failed to set manual shutter mode: " << seekcamera_error_get_str(status) << std::endl;
        return;
    } */

    status = seekcamera_set_temperature_unit(camera, SEEKCAMERA_TEMPERATURE_UNIT_FAHRENHEIT);
    if (status != SEEKCAMERA_SUCCESS)
    {
        std::cerr << "failed to set fahrenheit: " << seekcamera_error_get_str(status) << std::endl;
        return;
    }

    // TODO determine later if linear mode is required
    // if (linear_mode_) {
    //     ROS_INFO("Setting Seek to linear mode.");
    //     status = seekcamera_set_agc_mode(camera, seekcamera_agc_mode_t::SEEKCAMERA_AGC_MODE_LINEAR);
    //     if (status != SEEKCAMERA_SUCCESS)
    //     {
    //         std::cerr << "failed to set linear agc: " << seekcamera_error_get_str(status) << std::endl;
    //         return;
    //     }
    //     status = seekcamera_set_linear_agc_lock_mode(camera, seekcamera_linear_agc_lock_mode_t::SEEKCAMERA_LINEAR_AGC_LOCK_MODE_MANUAL);
    //     if (status != SEEKCAMERA_SUCCESS)
    //     {
    //         std::cerr << "failed to set linear agc: " << seekcamera_error_get_str(status) << std::endl;
    //         return;
    //     }
    //     seekcamera_set_linear_agc_lock_min(camera, 0.0);
    //     seekcamera_set_linear_agc_lock_max(camera, 500.0);
    // }

	seekcamera_color_palette_t current_palette;
    if (calibration_mode_) {
        current_palette = SEEKCAMERA_COLOR_PALETTE_BLACK_HOT;
    }
    else {
        current_palette = SEEKCAMERA_COLOR_PALETTE_WHITE_HOT;
    }
     
    status = seekcamera_set_color_palette(camera, current_palette);
    if (status != SEEKCAMERA_SUCCESS)
    {
        std::cerr << "failed to set color palette: " << seekcamera_error_get_str(status) << std::endl;
        return;
    }
	std::cout << "color palette: " << seekcamera_color_palette_get_str(current_palette) << std::endl;
}

// Handles camera disconnect events.
void handle_camera_disconnect(seekcamera_t *camera, seekcamera_error_t event_status, void *user_data)
{
    (void)event_status;
    (void)user_data;
}

// Handles camera error events.
void handle_camera_error(seekcamera_t *camera, seekcamera_error_t event_status, void *user_data)
{
    (void)user_data;
    seekcamera_chipid_t cid{};
    seekcamera_get_chipid(camera, &cid);
    std::cerr << "unhandled camera error: (CID: " << cid << ")" << seekcamera_error_get_str(event_status) << std::endl;
    
    // Stop and reconnect
    seekcamera_error_t stop_status = seekcamera_capture_session_stop(camera);
    std::cerr << "stop status: (CID: " << cid << ")" << seekcamera_error_get_str(stop_status) << std::endl;
    seekcamera_error_t start_status = seekcamera_capture_session_start(camera, seekcamera_frame_format_t::SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888 | seekcamera_frame_format_t::SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT);
    std::cerr << "reconnect status: (CID: " << cid << ")" << seekcamera_error_get_str(start_status) << std::endl;
}

// Handles camera ready to pair events
void handle_camera_ready_to_pair(seekcamera_t *camera, seekcamera_error_t event_status, void *user_data)
{
    // Attempt to pair the camera automatically.
    // Pairing refers to the process by which the sensor is associated with the host and the embedded processor.
    const seekcamera_error_t status = seekcamera_store_calibration_data(camera, nullptr, nullptr, nullptr);
    if (status != SEEKCAMERA_SUCCESS)
    {
        std::cerr << "failed to pair device: " << seekcamera_error_get_str(status) << std::endl;
    }

    // Start imaging.
    handle_camera_connect(camera, event_status, user_data);
}

// Callback function for the camera manager; it fires whenever a camera event occurs.
void camera_event_callback(seekcamera_t *camera, seekcamera_manager_event_t event, seekcamera_error_t event_status, void *user_data)
{
    seekcamera_chipid_t cid{};
    seekcamera_get_chipid(camera, &cid);
    std::cout << seekcamera_manager_get_event_str(event) << " (CID: " << cid << ")" << std::endl;

    // Handle the event type.
    switch (event)
    {
    case SEEKCAMERA_MANAGER_EVENT_CONNECT:
        handle_camera_connect(camera, event_status, user_data);
        break;
    case SEEKCAMERA_MANAGER_EVENT_DISCONNECT:
        handle_camera_disconnect(camera, event_status, user_data);
        break;
    case SEEKCAMERA_MANAGER_EVENT_ERROR:
        handle_camera_error(camera, event_status, user_data);
        break;
    case SEEKCAMERA_MANAGER_EVENT_READY_TO_PAIR:
        handle_camera_ready_to_pair(camera, event_status, user_data);
        break;
    default:
        break;
    }
}

bool startRecording(const std::string &filename) {
    if (recording_)
    {
        RCLCPP_WARN(node_->get_logger(), "Already recording!");
        return false;
    }

    video_writer_.open(filename, 
                        cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 
                        27.0, // fps
                        cv::Size(frame_width_, frame_height_));

    if (!video_writer_.isOpened())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open video file for writing.");
        return false;
    }
    else {
        RCLCPP_INFO(node_->get_logger(), "Started recording to %s", filename.c_str());
    }

    std::string filename_thermal = filename.substr(0, filename.find_last_of(".")) + "_thermal.mp4";
    video_writer_thermal_.open(filename_thermal, 
                        cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 
                        27.0, // fps 
                        cv::Size(thermal_width_, thermal_height_));

    if (!video_writer_thermal_.isOpened())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open video file for writing.");
        return false;
    }
    else {
        RCLCPP_INFO(node_->get_logger(), "Started recording to %s", filename_thermal.c_str());
    }

    recording_ = true;
    return true;
}

bool stopRecording() {
    if (recording_)
    {
      video_writer_.release();
      recording_ = false;
      RCLCPP_INFO(node_->get_logger(), "Stopped recording.");
    }
    return true;
}

bool recordVideoCallback(const std::shared_ptr<messages_88::srv::RecordVideo::Request> req,
    std::shared_ptr<messages_88::srv::RecordVideo::Response> resp) {
    bool success;
    if (req->start)
      success = startRecording(req->filename);
    else
      success = stopRecording();

    resp->success = success;
    return success;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node_ = rclcpp::Node::make_shared("seek_wrapper");

    node_->declare_parameter("do_calibrate", calibration_mode_);
    node_->get_parameter("map_frame", calibration_mode_);
    node_->declare_parameter("offline", offline_);
    node_->get_parameter("offline", offline_);

    // Load camera info from yaml
    std::string camera_name = "seek_thermal";
    std::string yaml_path = ament_index_cpp::get_package_share_directory("seek_thermal_88") + "/config/calibration.yaml";
    camera_calibration_parsers::readCalibration( yaml_path, camera_name, camera_info_);

    // ROS setup
    image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("image", 10);
    thermal_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("image_thermal", 10);
    info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    // TODO bring back calibration and offline modes or delete
    // set_info_service_ = node_->create_service("set_camera_info", &setCameraInfo);
    // if (offline_) {
    //     image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&SeekWrapper::imageCallback, node_, _1));
    // }

    // Video recorder service
    record_service_ = node_->create_service<messages_88::srv::RecordVideo>("~/record", &recordVideoCallback);

    seekcamera_manager_t *manager = nullptr;
    if (!offline_) {
        // Create the camera manager.
        // node_ is the structure that owns all Seek camera devices.
        seekcamera_error_t status = seekcamera_manager_create(&manager, SEEKCAMERA_IO_TYPE_USB);
        if (status != SEEKCAMERA_SUCCESS)
        {
            std::cerr << "failed to create camera manager: " << seekcamera_error_get_str(status) << std::endl;
            return 1;
        }

        // Register an event handler for the camera manager to be called whenever a camera event occurs.
        status = seekcamera_manager_register_event_callback(manager, camera_event_callback, nullptr);
        if (status != SEEKCAMERA_SUCCESS)
        {
            std::cerr << "failed to register camera event callback: " << seekcamera_error_get_str(status) << std::endl;
            return 1;
        }
    }

    rclcpp::spin(node_);

    if (!offline_) {
        std::cout << "Destroying camera manager" << std::endl;

        // Teardown the camera manager.
        seekcamera_manager_destroy(&manager);
    }

    
  rclcpp::shutdown();

}