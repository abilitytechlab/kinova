# Eye control

This repository contains a ROS package for controlling the Kinova Jaco using 
an eye tracker. 
It is meant to run on Ubuntu 20.04 with ROS noetic.
It depends on the kinova-ros stack: https://github.com/Kinovarobotics/kinova-ros

## Requires

- ROS-noetic
- Kinova-ROS package
- Python
- rosbridge_suit

## Installation

1. Install ROS: http://wiki.ros.org/noetic/Installation/Ubuntu

2. Set up your workspace: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

3. Install the kinova-ros package using their instructions, *IMPORTANT* use
catkin_make not the newer catkin build, while the kinova-ros package might work
using catkin build, this packageg has not been tested using catkin build.

4. Install rosbridge_suite: `sudo apt-get install ros-noetic-rosbridge-suite`

5. Install python dependencies: `pip install -r requirements.txt`

6. Run everything (see next section)

## Running everything

1. Source catkin_ws/devel/setup.bash
2. Launch the kinova ros stack: `roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300`
3. `source /opt/ros/noetic/setup.bash`
4. Launch rosbridge: `roslaunch rosbridge_server rosbridge_websocket.launch`
5. Run the node from the eye-control node: `rosrun eye-control gui_to_kinova`
6. Launch the interface in eye-control/gui/control_arm.py
7. Open `127.0.0.1:8000` in your browser

Optional:

7. Launch the pose_to_path node, which converts the pose output from the kinova
stack to a path, useful for visualuzing the movement of the arm in rviz:
`rosrun eye-control pose_to_path`

To connect from another computer I did the following:
1. Open a mobile hotspot from your phone
2. Connect both computers to the network
3. run `ifconfig` to see the ip on the computer running the system
4. Input the ip with port 8000 into the browser of the other computer connected
to the same network

## Troubleshooting

Regarding socketio debug mode, when this is enabled, the camera will not work
anymore, so keep that in mind. At the same time there is problem when debug 
mode is not enabled and you have a package like eventlet or gevent installed.
When this is the case, flask-socketio will start the server with eventlent or 
gevent, but in my experience this breaks the functionality of the buttons.
When debug mode is on, it will always launch a werkzeug server, in which the
buttons do work. If debug mode is off and the buttons do not seem to work, 
it might not be running a werkzeug server, the easiest solution is simply
uninstalling eventlet or gevent.

## Configuration

The camera which is used is set in the file `control_server.py` on line 7, cv2.VideoCapture(CAMERA_ID <<<, ...)

The speed at which the arm moves is set in the same file and is self.speed

The amount of time a move action takes is set in the move function, using the 
variable i. 100 is approximately 1 second (not super accurate, the execution
time for the function is not taken into account, but it works)

## To do

- [ ] Implement cartesian position control using the kinova pose action server
- [ ] Implement setpoint control (requires the above)
- [ ] Run on Raspberry Pi
