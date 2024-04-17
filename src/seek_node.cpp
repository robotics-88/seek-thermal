/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <camera_calibration_parsers/parse.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

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

ros::Publisher image_pub_;
ros::Publisher info_pub_;
ros::ServiceServer set_info_service_;

bool seek_ok_ = true;
sensor_msgs::CameraInfo camera_info_;

seekcamera_frame_format_t frame_format = seekcamera_frame_format_t::SEEKCAMERA_FRAME_FORMAT_COLOR_ARGB8888;

bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& resp) {
    camera_info_ = req.camera_info;
    resp.success = true;
    return true;
}

// Handles frame available events.
void handle_camera_frame_available(seekcamera_t *camera, seekcamera_frame_t *camera_frame, void *user_data)
{
    seekframe_t* frame = nullptr;
    seekcamera_error_t status = seekcamera_frame_get_frame_by_format(camera_frame, frame_format, &frame);
    if(status != SEEKCAMERA_SUCCESS)
    {
        std::cerr << "failed to get frame: " << seekcamera_error_get_str(status) << std::endl;
        return;
    }
    const int frame_width = (int)seekframe_get_width(frame);
    const int frame_height = (int)seekframe_get_height(frame);
    const int frame_stride = (int)seekframe_get_line_stride(frame);
    cv::Mat frame_mat(frame_height, frame_width, CV_8UC4, seekframe_get_data(frame));

    cv::cvtColor(frame_mat, frame_mat, CV_BGR2GRAY);
    frame_mat.convertTo(frame_mat, CV_32FC1);

    seekcamera_frame_header_t* header = (seekcamera_frame_header_t*)seekframe_get_header(frame);
    uint64_t time = header->timestamp_utc_ns;
    double sec = time * 1e-9;
    ros::Time t = ros::Time(sec);

    cv_bridge::CvImage image_msg;
    image_msg.header.frame_id = "seek";
    image_msg.header.stamp = t;
    image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    image_msg.image    = frame_mat;
    image_pub_.publish(image_msg.toImageMsg());

    camera_info_.header = image_msg.header;
    info_pub_.publish(camera_info_);
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
    status = seekcamera_capture_session_start(camera, frame_format);
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

	seekcamera_color_palette_t current_palette = SEEKCAMERA_COLOR_PALETTE_WHITE_HOT;
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
    seekcamera_error_t start_status = seekcamera_capture_session_start(camera, frame_format);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "seek_wrapper");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Create the camera manager.
    // This is the structure that owns all Seek camera devices.
    seekcamera_manager_t *manager = nullptr;
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

    ros::NodeHandle nh_;
    float fps = 30.0; 
    ros::Rate r(1 / (2 * fps)); // Check at twice the desired rate

    // ROS setup
    image_pub_ = nh_.advertise<sensor_msgs::Image>("image_raw", 10);
    info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 10);
    set_info_service_ = nh_.advertiseService("set_camera_info", &setCameraInfo);

    // Load camera info from yaml
    std::string camera_name = "seek_thermal";
    std::string yaml_path = ros::package::getPath("seek_thermal_88") + "/config/calibration.yaml";
    camera_calibration_parsers::readCalibration( yaml_path, camera_name, camera_info_);

    while (ros::ok() && seek_ok_)
    {
        ros::spinOnce();
        r.sleep();
    }

    std::cout << "Destroying camera manager" << std::endl;

    // Teardown the camera manager.
    seekcamera_manager_destroy(&manager);

    ros::shutdown();

    return 0;
}