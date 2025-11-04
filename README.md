ROS1 Gazebo Simulation Competition
This repository contains a ROS1-based Gazebo simulation competition. After executing the following commands sequentially, the robot will perform autonomous navigation:

Launch the simulation:

bash
roslaunch robot_simulation simulation_robot.launch
Start RViz navigation:

bash
roslaunch robot_navigation navigation.launch
Launch YOLO community person detection:

bash
roslaunch yolo_ros yolo.launch
Start license plate recognition:

bash
rosrun robot_navigation detect_plate.py
Run the mission logic:

bash
rosrun robot_navigation mission.py
The robot will automatically start patrolling. When it reaches the fourth and fifth navigation points, it will invoke the YOLO model for image recognition. Upon arriving at the ninth navigation point, it will call the Baidu OCR license plate recognition API for license plate identification.

Image recognition is implemented through clients recognize_person and recognize_plate, which send parameters to their respective servers.
