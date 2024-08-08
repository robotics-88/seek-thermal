/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include <rclcpp/rclcpp.hpp>

#include "seek_wrapper.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr <rclcpp::Node> nh = rclcpp::Node::make_shared("seek_wrapper", node_options);
    seek_wrapper::SeekWrapper seek_wrapper(nh);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nh);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
