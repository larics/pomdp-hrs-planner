# Diplomski
This is a repository that contains code for solving three different problems using POMDP-s. Problems are solved using ROS Kinetic and Python and simulated using Gazebo. 
## Installation
To install ROS Kinetic please follow instructions from:
http://wiki.ros.org/kinetic/Installation

To download and install files necessary for running the UAV please follow the instructions from:
https://github.com/larics/mmuav_gazebo

To download the Gazebo world used in the simulations follow link:
https://github.com/larics/larics_gazebo_worlds

To download this repository use:

>$ cd ~/catkin_ws/src
>$ git clone https://github.com/NinaDruzetic/Diplomski/tree/master/pomdp
>$ git clone https://github.com/NinaDruzetic/Diplomski/tree/master/pomdp_paper_action
>$ git clone https://github.com/NinaDruzetic/Diplomski/tree/master/paper_extended
>$ cd ~/catkin_ws/src
>$ catkin build

## Simulation
### pomdp
This folder contains files used to simulate well-known tiger problem. To run the simulation use:

>$ roslaunch pomdp tigar.launch

### pomdp_paper
This folder is implementation of the problem found in:

https://smartech.gatech.edu/bitstream/handle/1853/21555/POMDP-FLAIRS.pdf

To run the simulation in Gazebo world with one UAV use:

>$ roslaunch pomdp_paper_action paper.launch

### pomdp_paper_extended
This folder contains solution of the paper mentioned before using UAV and mobile robot. The solution also considers energy consumption as well as UAVs battery charging. 

To run the simulation in Gazebo with UAV and mobile platform use:

>$ roslaunch paper_extended two_robots_simulation_final.launch
