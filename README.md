# jaco_arm

Tested on Ubuntu 16.04.

1. [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
2. Install Kinova ROS

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Kinovarobotics/kinova-ros.git kinova-ros
rm -rf kinova-ros/kinova_moveit # Cannot be compiled with ROS Kinetic/Ubuntu 16.04
cd ~/catkin_ws
catkin_make
```

3. Install driver

```bash
sudo cp ~/catkin_ws/src/kinova-ros/kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/
```

4. Launch driver in background

```bash
roslaunch kinova_bringup kinova_robot.launch 
```

5. Launch grasping demo

```bash
python grasping.py
```
