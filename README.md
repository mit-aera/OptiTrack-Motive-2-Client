# ROS OptiTrack Client for Motive 2

## Installation

```bash
# Prerequisites
sudo apt install libeigen3-dev

# Clone the code
cd ~/catkin_ws/src
git clone -b racecar https://github.com/AgileDrones/OptiTrack-Motive-2-Client.git

# Catkin make
cd ~/catkin_ws
catkin_make
```

## Running Client Node

```bash
rosrun optitrack_motive_2_client optitrack_motive_2_client_node --server 192.168.1.12 --local 192.168.1.103
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
