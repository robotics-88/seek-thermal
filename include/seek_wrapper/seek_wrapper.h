/* 
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef SEEK_WRAPPER_H_
#define SEEK_WRAPPER_H_

#include <string>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

namespace seek_wrapper {
/**
 * @class SpeciesMapper
 * @brief A class for converting images with bounding boxes on detected species into maps
 */
class SeekWrapper {

    public:
        SeekWrapper(ros::NodeHandle& node);

        ~SeekWrapper();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
};

}

#endif