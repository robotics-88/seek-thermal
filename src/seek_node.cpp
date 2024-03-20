/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "seek_wrapper/seek_wrapper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seek_wrapper");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle node;
  seek_wrapper::SeekWrapper seek_wrapper(node);

  ros::spin();

  return 0;
}