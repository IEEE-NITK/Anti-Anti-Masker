# Anti-Anti-Masker Drone

### Abstract:
Anti-Anti-Masker is a drone that will roam in city crowded areas and search for people who are not wearing masks and breaking covid protocols. This project can have many other applications, such as security and surveillance. We will be using CAD tools to design the drone and then generate its URDF. We will be using ROS to design the robot’s environment and physics and simulate the robot. This drone will have a camera that will have a face recognition system to detect faces without a mask. This project aims to model and simulate a drone and design an AI face recognition algorithm that can be implemented in the UAV. The simulation can be done on ROS and for face recognition, OpenCV MTCNN can be used. This project will be an inter-sig project


### Project Members:
1. Shobuj Paul
2. Diptesh Banerjee

### Simulation Instructions:

- Clone the repository in the src folder in your workspace.
```bash
cd ~/catkin_ws/src/
git clone git@github.com:IEEE-NITK/Anti-Anti-Masker.git drone
```
- Build your workspace.
```bash
cd ~/catkin_ws
catkin build
```
- To view the drone model in gazebo environment run:
```bash
roslaunch drone_description gazebo.launch
```
- To open the model in RViz run:
```bash
roslaunch drone_description display.launch
```
