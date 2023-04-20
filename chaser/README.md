## Summary <br>

This assignment consists of one ROS2 package _chaser_, with three Python scripts: _collision.py_ _chaser.py_ _spin.py_. This package implements behaviours that will help the robot find coloured objects. <br>
Twist_mux was used to allow all these different nodes to publish to the /cmd_vel topic, making it possible to add priority to these behaviours.

The _laserScan_, _Image_ and _Twist_ messages with their respective topics were predominatly used for this implementation. _laserScan_ was used to stop collisions, _Image_ was used to detect colours and _Twist_ was used to move the robot.

## Installation <br>

Twist_mux was used in this package and can be installed with the following commands: <br>

_sudo apt update_
<br>
_sudo apt install ros-humble-twist-mux_

## Commands <br>

Once Twist_mux is installed the package can be used to find coloured objects. <br>
Below are commands to start the package with the Gazebo world. <br>

_ros2 launch uol_turtlebot_simulator object-search-1.launch.py_
<br>

# robotics workspace directory <br>

_ros2 launch chaser twist_mux_launch.py_
<br> <br>
To run all of the package nodes individually: <br>

_ros2 run chaser collision_ <br>
_ros2 run chaser spin_ <br>
_ros2 run chaser chaser_ <br>

<br> <br>
To use the launch file: <br>
_ros2 launch chaser nodes_launch.py_
