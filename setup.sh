#!/bin/bash

export GAZEBO_MODEL_PATH=$PWD/resource/models:$GAZEBO_MODEL_PATH
gazebo -u --verbose resource/worlds/challenge_maze.world