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
$ ros2 launch need4stek slam.launch.py
```

### Starting the wall-following algorithm:
In another terminal, run:
```sh
$ . install/local_setup.{z}sh
$ export TURTLEBOT3_MODEL=waffle
$ export GAZEBO_MODEL_PATH=$PWD/resource/models:$GAZEBO_MODEL_PATH
# Launch the wall following node.
$ ros2 launch need4stek drive.launch.py
```

### Saving the map:
Run 
```sh
$ ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: need4stek_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

### Copying the map to resources
Run
```sh
$ mv need4stek_map.pgm resource/
$ mv need4stek_map.yaml resource/
```

### Lauching the world while loading a map
Run
```sh
$ ros2 launch need4stek loader.launch.py
```

## Launchfiles:
### Your World:
File: `your_world.launch.py`
Documentation:
- Gets the model used from the environment.
- Generates the world using gazebo with the .world file in the resource/wolds directory.
- Adds the model to the map using gz with the selected model.

### Drive:
File: `drive.launch.py`
Documentation:
- Maps the namespaced topics to default topics.
- Launches the `wall_following` node.
- This node allows the robot to move in the world, going in a straight line untill it finds a wall, and then follows it.

### Slam:
File: `slam.launch.py`
Documentation:
- Creates a world
- Adds the robot
- Launches SLAM to navigate and build the map

### Loader:
File: `loader.launch.py`
Documentation:
- Creates a world
- Adds the robot
- Loads a given map instead of using SLAM.

## Authors:
### Kentin Pratelli
### Philippe De Sousa
