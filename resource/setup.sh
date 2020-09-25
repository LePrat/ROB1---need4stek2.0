#!/bin/bash

export GAZEBO_MODEL_PATH=$PWD/models:$GAZEBO_MODEL_PATH
gazebo -u --verbose resource/worlds/challenge_maze.world
