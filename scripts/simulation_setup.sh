#!/bin/bash

# get index of the robot by getting the index of the name of the container
export ROBOT_INDEX=${CONTAINER_NAME##*_}

# change robot name based on the index of the container
if [ $ROBOT_INDEX -gt 1 ]
then
    # format of robot name
    ROBOT_NAME="${ROBOT_NAME}_${ROBOT_INDEX}"
fi

export WEBOTS_CONTROLLER_URL=tcp://${WEBOTS_SIMULATION_IP}:${WEBOTS_SIMULATION_PORT}/${ROBOT_NAME}