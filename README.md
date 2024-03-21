# seek-thermal

ROS wrapper for the Seek Mosaic starter kit thermal camera. Built on the Seek SDK v4.4. Uses the Seek API to get frames, OpenCV to read data, and republishes as ROS image topic.

## Setup
Clone and build. Tested with ROS noetic.
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone git@github.com:robotics-88/seek-thermal.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

## Usage

With device plugged in by USB, run:
```
roslaunch seek_thermal_88 seek.launch
```

## TODO

* Sometimes the camera device does not connect properly at the start (maybe 25% of the time), so the ROS node is set to crash and respawn in that case. Not ideal but a temporary workaround until we can get it to programmatically reconnect the device.

* Publishing at a lower rate, about 18 fps. Why?