## Summary <br>

This assignment consists of one ROS2 package *chaser*, with three Python scripts: *collision.py* *chaser.py* *spin.py*. This package implements behaviours that will help the robot find coloured objects. <br>
Twist_mux was used to allow all these different nodes to publish to the /cmd_vel topic, making it possible to add priority to these behaviours. 

The *laserScan*, *Image* and *Twist* messages with their respective topics were predominatly used for this implementation. *laserScan* was used to stop collisions, *Image* was used to detect colours and *Twist* was used to move the robot.


## Installation <br>
Twist_mux was used in this package and can be installed with the following commands: <br>

*sudo apt update*
<br>
*sudo apt install ros-humble-twist-mux*




## Commands <br>
Once Twist_mux is installed the package can be used to find coloured objects. <br>
Below are commands to start the package with the Gazebo world. <br>

*ros2 launch uol_turtlebot_simulator object-search-1.launch.py*


# robotics workspace directory <br>
*ros2 launch chaser twist_mux_launch.py*
<br>
To run all of the package nodes individually: <br>

*ros2 run chaser collision*
*ros2 run chaser spin*
*ros2 run chaser chaser*
