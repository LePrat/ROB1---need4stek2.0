# NEED4STEK

## Dependencies:
### Ros2:
A tutorial on how to install ros2 is available [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/).


## Setup the workspace:
First off, clone the git repository:
```sh
wget https://raw.githubusercontent.com/LePrat/ROB1---need4stek2.0/master/need4stek2.repos
vcs import < need4stek2.repos
```
Then, install the Turlebot3 and Need4Stek packages:
### Turlebot3
You need to build the turtlebot3 package:
```sh
# Go to the turtlebot3 directory you cloned using.
$ cd turtlebot3
# Build 
$ colcon build --symlink-install
```
Once you've installed the turtlebot3 packages, you can install ours.
### Need4stek
You need to build our launchiles before using them:
```sh
# Go to the directory you cloned using `vcs import` or `wstool`.
$ cd ROB1---need4stek2.0
# Build the launchfiles
$ colcon build
```

## Running the launchfiles
### Starting the world:
In one terminal, run:
```sh
# Source your shell. Either use the .sh or .zsh setup.
$ . install/local_setup.{z}sh
# Add the /resource/models folders to the Gazebo model path
$ export GAZEBO_MODEL_PATH=$PWD/resource/models:$GAZEBO_MODEL_PATH	
# Add to your environment which model you want to use. If none is exported, 'burger' will be selected
$ export TURTLEBOT3_MODEL=waffle
# Start the world using gazebo, and ads a robot in it using gz.
$ ros2 launch need4stek your_world_launch.py
```

### Starting the wall-following algorithm:
In another terminal, run:
```sh
$ . install/local_setup.{z}sh
$ export TURTLEBOT3_MODEL=waffle
$ export GAZEBO_MODEL_PATH=$PWD/resource/models:$GAZEBO_MODEL_PATH
# Launch the wall following node.
$ ros2 launch need4stek capability1
```

## Launchfiles:
### Your World:
File: `your_world.launch.py`
Documentation:
- Gets the model used from the environment.
- Generates the world using gazebo with the .world file in the resource/wolds directory.
- Adds the model to the map using gz with the selected model.

### Capability1:
File: `capability1.launch.py`
Documentation:
- Launches the `wall_following` node without a namespace in order to access the topics correctly.
- This node allows the robot to move in the world, going in a straight line untill it finds a wall, and then follows it.

## Authors:
### Kentin Pratelli
### Philippe De Sousa