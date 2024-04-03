1. install gstreamer
link:
https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=c

sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

pkg-config --cflags --libs gstreamer-1.0

2. install ros2 humble (if not already installed)

UBUNTU 22.04 Needed

https://docs.ros.org/en/humble/Installation.html

3. Test gstream on webcam

gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! ximagesink


4. Install gscam2 for gstreaming in ros2

Go to ros worksace (ros2_ws created in ros2 tutorial)

go to src/ clone ros2_shared and gscam2 repose
cd src
git clone https://github.com/ptrmu/ros2_shared.git
git clone https://github.com/clydemcqueen/gscam2.git
cd ..
colcon build

5. install build nodes
source /opt/ros/humble/setup.bash
source install/local_setup.bash

6. Mofify the params file: gscam2/cfg/params.yaml

gscam_config: 'v4l2src device=/dev/video0 ! videoconvert'

7. Run gstream in ros2
ros2 launch gscam2 node_param_launch.py

8. Run rviz and visualize image

ros2 run rviz2 rviz2
topic: /my_camera
