/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "seek_wrapper/seek_wrapper.h"

namespace seek_wrapper
{
SeekWrapper::SeekWrapper(ros::NodeHandle& node)
  : nh_(node)
  , private_nh_("~")
{
}

SeekWrapper::~SeekWrapper() {
}


}