Robotics assignment <br>
<br>
Chaser is the main package for this project and contains the source code as well as all the set up required. <br>
The ros-humble-twist-mux package has been used to add priorities to the /cmd_vel topic publishers. This allows for multiple nodes to publish to /cmd_vel with a specsified order to what publishers take priority. <br>


The colour.py file contains the code to make the robot chase the coloured beams. Multiple filters are applied to the ROS image in order to pinpoinht where the coloured objects are. <br>
