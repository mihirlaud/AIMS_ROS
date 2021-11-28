# Pixhawk ROS Flight Controller

## Introduction

This flight controller was made by me, Mihir Laud, an undergraduate student at Purdue University working for the Autonomous Multi-Agent Intelligent Systems Lab, or AIMS Lab. 

It was built for a custom quadrotor that uses the Pixhawk 4 Mini as its onboard flight computer. The controller is built on the ROS platform, specifically on ROS Melodic in Ubuntu 18.04. It can take, as input, a set of points in 3D space and have the quadrotor navigate to those points using velocity commands and PID control.

A demonstration of early point-to-point movement running on a simulated drone can be found [here](https://youtu.be/Y0Lm5s196Io). Another demo of the drone navigating to multiple waypoints can be found [here](https://youtu.be/-N6l_FVbApk).

## Installation

This installation guide only works for Ubuntu 18.04, as that is what is best supported by the PX4 Documentation. You may find different results using Ubuntu 20.04 or WSL.

Setup your ROS + Gazebo environment with the following commands:

```bash
wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
bash ubuntu_sim_ros_melodic.sh
```

If the above commands do not work on the first try, you may need to install additional Python packages. Use the error message to find which package is missing, install it, then run the bash command again to continue the setup.

The script will have created and initialized your catkin workspace. Clone the repo into the src folder to create a new node, then build the workspace.

```bash
cd ~/catkin_ws/src/
git clone https://github.com/mihirlaud/AIMS_ROS offb
catkin build
```

Clone the PX4 Autopilot firmware somewhere you can easily access it. This will be using to launch the Gazebo simulation.

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Finally, download and install the QGroundControl daily build using the instructions found [here](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html).

## Running the Simulation

In a terminal, find the PX4 firmware and make the SITL Gazebo simulation.

```bash
cd /path/to/PX4-Autopilot
make px4_sitl gazebo
```

During the installation, you will likely need to install additional Python packages, which will be prompted during the make process. 

You may also run into an error that regarding a GSTREAMER variable. In that case, installing the following dependency may help.

```bash
sudo apt-get install libgstreamer-plugins-base1.0-dev
```

Run QGroundControl to provide positional information to the quadrotor. Then, in a new terminal, launch the mavros node.

```bash
roslaunch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
```

Finally, in a third terminal, run the offb node to start the flight controller

```bash
rosrun offb offb_node x y z
```

Here, the x, y, and z arguments represent the point in 3D space that the drone will navigate to. If no point is specified, the drone will move to (0, 0, 1.5).