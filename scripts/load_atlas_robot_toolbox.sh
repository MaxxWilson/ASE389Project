#!/bin/bash

ROBOT_TOOLBOX_PATH="$HOME/anaconda3/envs/pypnc/lib/python3.8/site-packages/rtbdata/xacro/"
ATLAS_PATH=$(dirname $(dirname $(realpath $0)))/atlas_ros/resource/atlas

# If folder already exists, prompt for user input
if [ -d "$ROBOT_TOOLBOX_PATH/atlas" ]
then
    echo "Atlas folder already exists."
    read -p "Should it be replaced(y/n)?" -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]
    then
        exit 0
    else
        rm -r "$ROBOT_TOOLBOX_PATH/atlas"
    fi
fi

# Copy atlas files to robot toolbox dir
if [ -d $ROBOT_TOOLBOX_PATH ]
then
    echo "ATLAS_PATH: " $ATLAS_PATH
    echo "ROBOT_TOOLBOX_PATH: " $ROBOT_TOOLBOX_PATH
    cp -r $ATLAS_PATH $ROBOT_TOOLBOX_PATH
    echo
    echo "Atlas files succesfully copied to robot toolbox"
fi