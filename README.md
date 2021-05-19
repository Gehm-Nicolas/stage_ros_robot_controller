# stage_ros_robot_controller

ROS stage simulator code example to control a robot to reach a determined target avoiding obstacles using a laser sensor (LiDAR) of 270 degree range on a Hokuyo map.

<div align="center">
  <img src="./stage_map.png" alt="Hokuyo map" width=60%>
</div>

## Built with:
<ul>
  <li><a href="https://www.ros.org/">ROS 1</a></li>
  <li>Python 3</li> 
</ul>

## Project file tree
<pre>.
|-- catkin_ws
    |-- <strong>build</strong>
    |-- <strong>devel</strong>
    |-- <strong>src</strong>
        |-- CMakeLists.txt
        |-- <strong>stage_controller</strong>
            |-- <strong>[...]</strong>
            |-- <strong>scripts</strong>
                |-- stage_controller.py
</pre>

## Getting Started
### Install ROS
The code was developed on ROS Noetic Ninjemys distribution. Instruction can be found [here](http://wiki.ros.org/ROS/Installation).

### Run stage simulator environment
First, run this command to have access to the ROS commands and update the environment.
```
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
```
At the <code>catkin_ws</code> directory run the stage simulator using an hokuyo world.
```
rosrun stage_ros stageros src/stage_controller/world/create_hokuyo.world
```

### Run the robot control script
Now, run the python script which controls the robot with the command
```
rosrun stage_controller stage_controller.py
```

## Acknowledgments
This project was made as part of the Robot's Project course in [Robotics and AI postgraduate program](https://utec.edu.uy/posgrado-en-robotica-e-inteligencia-artificial-br/) at [UTEC](https://utec.edu.uy/)/[FURG](https://www.furg.br/).

The ideia was implement a solution without consult any existing method of localization and mapping.

## To-do list
- [ ] Improve the condition that makes the robot choose turn left/right.
- [ ] Implement a function to change robot's position at any time
- [ ] Implement a dynamic way of verify is there is some obstacle in the robot's path
- [ ] Implement a known mapping and location algorithm
