# ROS OptiTrack Client for Motive 2

## Installation

```bash
# Prerequisites
sudo apt install libeigen3-dev

# Install in ROS workspace. Assumes that 'wstool init' has been run in workspace
cd ~/catkin_ws/src

curl https://raw.githubusercontent.com/AgileDrones/OptiTrack-Motive-2-Client/master/.rosinstall >> .rosinstall
wstool update

cd ../
catkin_make

```

## Running Client Node

```bash
rosrun optitrack_motive_2_client optitrack_motive_2_client_node --server 192.168.2.10 --client 192.168.2.1
```

## Debugging Client Node

```bash
# From root of workspace
catkin_make -DCMAKE_BUILD_TYPE=Debug

gdb `catkin_find optitrack_motive_2_client optitrack_motive_2_client_node`
# Common gdb commands
> run
> bt
> quit
```