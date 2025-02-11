# seek-thermal

ROS wrapper for the Seek Mosaic starter kit thermal camera. Built on the Seek SDK v4.3. Uses the Seek API to get frames, OpenCV to read data, and republishes as ROS image topic.

## Setup
[Download](https://developer.thermal.com/support/solutions/articles/48001240854-seek-thermal-sdk-v4-3) and install SDK:
```
sudo apt-get install libsdl2-dev
unzip <SDK>
cd <SDK>
cd <your_architecture>
sudo cp lib/libseekcamera.so /usr/local/lib
sudo cp lib/libseekcamera.so.4.4 /usr/local/lib
sudo cp -r include/* /usr/local/include
sudo cp driver/udev/10-seekthermal.rules /etc/udev/rules.d
sudo udevadm control --reload
sudo chmod u+x bin/*
```

On some machines, I had to add this line to ~/.bashrc:

`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib`

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
If successful, the Rviz window that pops up should look like this:

![thermal image of a cat in Rviz](images/thermal-cat.png)

## Calibration

To use ROS camera calibration, the BLACK_HOT color map is preferred. Launch with:
```
roslaunch seek_thermal_88 seek.launch do_calibrate:=true
```

Then run ROS camera calibration with, e.g.,:
```
rosrun camera_calibration cameracalibrator.py --size 5x3 --square 0.85 image:=/seek_thermal/image_raw camera:=/seek_thermal
```

It seems to work better with large squares and a halogen lamp (as compared to using sunlight). Recommend a printed sheet of paper rather than a real chessboard to minimize reflections from the lamp.