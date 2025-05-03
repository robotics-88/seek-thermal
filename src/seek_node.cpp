/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include <rclcpp/rclcpp.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
#include <fstream>

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
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr meas_fps_pub_;

rclcpp::TimerBase::SharedPtr meas_fps_timer_;
rclcpp::TimerBase::SharedPtr record_timer_;

rclcpp::Service<messages_88::srv::RecordVideo>::SharedPtr record_service_;

bool calibration_mode_ = false;
bool offline_ = false;
bool recording_ = false;
bool rotate_ = false;

sensor_msgs::msg::CameraInfo camera_info_;
std::string camera_info_path_ = ament_index_cpp::get_package_share_directory("seek_thermal_88") + "/config/calibration.yaml";
cv::VideoWriter video_writer_, video_writer_thermal_;
cv::Mat last_frame_mat_, last_thermal_mat_;

std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
std::ofstream pose_file_;
std::string map_frame_ = "map";

std::mutex frames_mutex_;

int frame_width_;
int frame_height_;
int recording_width_;
int recording_height_;
int thermal_width_;
int thermal_height_;
double fps_ = 27.0;

int last_frame_count_ = 0;
std::atomic<int> frame_count_ = 0;
std::atomic<int> written_frame_count_ = 0;

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

void writeToPoseFile() {
    // Get the current camera pose from the TF tree
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_->lookupTransform(map_frame_, "seek_thermal", tf2::TimePointZero);
        rclcpp::Time transform_time(transform_stamped.header.stamp);
        if (node_->get_clock()->now() - transform_time > rclcpp::Duration::from_seconds(0.5)) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Camera TF is older than 0.5 seconds");
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Could not transform: %s", ex.what());
        return;
    }

    // TODO make this a service arg
    int camera_id = 1;
    std::string image_name = "image_" + std::to_string(written_frame_count_) + ".png";
    // Write the pose to the pose file
    if (pose_file_.is_open())
    {
        pose_file_  << written_frame_count_ << " ";
        pose_file_  << transform_stamped.transform.rotation.w << " " 
                    << transform_stamped.transform.rotation.x << " "
                    << transform_stamped.transform.rotation.y << " " 
                    << transform_stamped.transform.rotation.z << " ";
        pose_file_  << transform_stamped.transform.translation.x << " " 
                    << transform_stamped.transform.translation.y << " " 
                    << transform_stamped.transform.translation.z << " ";
        pose_file_  << std::to_string(camera_id) << " " << image_name << "\n";
        pose_file_  << "\n"; // Leave blank line between frames
    }
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
    recording_width_ = frame_width_;
    recording_height_ = frame_height_;

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
    if (rotate_) {
        cv::rotate(frame_mat, frame_mat, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::rotate(thermal_mat, thermal_mat, cv::ROTATE_90_COUNTERCLOCKWISE);
        recording_width_ = frame_height_;
        recording_height_ = frame_width_;
    }

    { // Lock context
    std::lock_guard<std::mutex> lock(frames_mutex_);
    if (last_frame_mat_.empty() || last_frame_mat_.size != frame_mat.size || last_frame_mat_.type() != frame_mat.type()) {
        last_frame_mat_ = frame_mat.clone();
    }
    else {
        frame_mat.copyTo(last_frame_mat_);
    }

    if (last_thermal_mat_.empty() || last_thermal_mat_.size != frame_mat.size || last_thermal_mat_.type() != frame_mat.type()) {
        last_thermal_mat_ = frame_mat.clone();
    }
    else {
        thermal_mat.copyTo(last_thermal_mat_);
    }
    }

    seekcamera_frame_header_t* header = (seekcamera_frame_header_t*)seekframe_get_header(frame);
    uint64_t time = header->timestamp_utc_ns;
    double sec = time * 1e-9;
    rclcpp::Time t = rclcpp::Time(sec);

    cv_bridge::CvImage image_msg;
    image_msg.header.frame_id = "seek_thermal";
    image_msg.header.stamp = t;
    image_msg.encoding = sensor_msgs::image_encodings::BGRA8;
    image_msg.image    = frame_mat;
    image_pub_->publish(*(image_msg.toImageMsg()).get());

    cv_bridge::CvImage image_msg_thermal;
    image_msg_thermal.header.frame_id = "seek_thermal";
    image_msg_thermal.header.stamp = t;
    image_msg_thermal.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    image_msg_thermal.image    = thermal_mat;
    thermal_pub_->publish(*(image_msg_thermal.toImageMsg()).get());

    camera_info_.header = image_msg.header;
    info_pub_->publish(camera_info_);
    
    frame_count_++;
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
                        fps_, // fps
                        cv::Size(recording_width_, recording_height_));

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
                                fps_, // fps 
                                cv::Size(thermal_width_, thermal_height_), false);

    if (!video_writer_thermal_.isOpened())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open video file for writing.");
        return false;
    }
    else {
        RCLCPP_INFO(node_->get_logger(), "Started recording to %s", filename_thermal.c_str());
    }

    recording_ = true;

    // Open the pose file
    std::string pose_filename = filename.substr(0, filename.find_last_of(".")) + "_pose.txt";
    pose_file_.open(pose_filename);

    return true;
}

bool stopRecording() {
    if (recording_)
    {
        if (video_writer_.isOpened())
            video_writer_.release();
        if (video_writer_thermal_.isOpened())
            video_writer_thermal_.release();
        if (pose_file_.is_open())
            pose_file_.close();
        recording_ = false;
        written_frame_count_ = 0;
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

void writeVideo() {
    if (recording_) {
        std::lock_guard<std::mutex> lock(frames_mutex_);
        if (video_writer_.isOpened())
        {
            cv::Mat frame_bgr;
            cv::cvtColor(last_frame_mat_, frame_bgr, cv::COLOR_BGRA2BGR);
            video_writer_.write(frame_bgr);
        }
        if (video_writer_thermal_.isOpened())
        {
            last_thermal_mat_.convertTo(last_thermal_mat_, CV_8UC1);
            video_writer_thermal_.write(last_thermal_mat_);
        }
        writeToPoseFile();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node_ = rclcpp::Node::make_shared("seek_thermal");

    node_->declare_parameter("do_calibrate", calibration_mode_);
    node_->get_parameter("do_calibrate", calibration_mode_);
    node_->declare_parameter("map_frame", map_frame_);
    node_->get_parameter("map_frame", map_frame_);
    node_->declare_parameter("offline", offline_);
    node_->get_parameter("offline", offline_);
    node_->declare_parameter("camera_info_path", camera_info_path_);
    node_->get_parameter("camera_info_path", camera_info_path_);
    node_->declare_parameter("rotate", rotate_);
    node_->get_parameter("rotate", rotate_);

    // Load camera info from yaml
    std::string camera_name = "seek_thermal";
    if (camera_calibration_parsers::readCalibration(camera_info_path_, camera_name, camera_info_)){
        info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 10);
        RCLCPP_INFO(node_->get_logger(), "Got camera info from %s", camera_info_path_.c_str());
    }
    else {
        info_pub_ = nullptr;
        RCLCPP_ERROR(node_->get_logger(), "Cannot get camera info, will not publish");
    }

    // ROS setup
    image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("~/image_raw", 10);
    thermal_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("~/image_thermal", 10);
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

    meas_fps_pub_ = node_->create_publisher<std_msgs::msg::Float32>("~/meas_fps", 10);

    // Publish how many frames received in last second
    meas_fps_timer_ = node_->create_wall_timer(std::chrono::seconds(1), []() {
      std_msgs::msg::Float32 msg;
      msg.data = frame_count_ - last_frame_count_;
      meas_fps_pub_->publish(msg);
      last_frame_count_ = frame_count_;
    });

    // Timer for recording
    record_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / fps_), writeVideo);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::spin(node_);

    if (!offline_) {
        std::cout << "Destroying camera manager" << std::endl;

        // Teardown the camera manager.
        seekcamera_manager_destroy(&manager);
    }

    
  rclcpp::shutdown();

}