# Diplomski
This is a repository that contains code for solving two different problems using POMDP-s. Problems are solved using ROS Kinetic and Python and simulated using Gazebo. 
## Installation
To install ROS Kinetic please follow instructions from:
http://wiki.ros.org/kinetic/Installation

To download and install files necessary for running the UAV please follow the instructions from:
https://github.com/larics/mmuav_gazebo

To download the Gazebo world used in the simulations follow link:
https://github.com/larics/larics_gazebo_worlds

## Simulation
### pomdp
This folder contains files used to simulate well-known tiger problem. To run the simulation use:

>$ roslaunch pomdp tigar.launch

### pomdp_paper
This folder is implementation of the problem found in:

https://smartech.gatech.edu/bitstream/handle/1853/21555/POMDP-FLAIRS.pdf

To run the simulation in Gazebo world with one UAV use:

>$ roslaunch pomdp_paper_action paper.launch
