#!/bin/bash

# Name of the Docker image
IMAGE_NAME="m4v2:ros-foxy"

# Name of the Docker container
CONTAINER_NAME="m4v2_ros_foxy"

# Path to the directory you want to mount inside the container
HOST_DIRECTORY_PATH="$(pwd)/m4_home"
CONTAINER_DIRECTORY_PATH="/home/m4version2"

# Check if the container is already running
if [ $(docker ps -q -f name=${CONTAINER_NAME}) ]; then
  echo "Container ${CONTAINER_NAME} is already running."
else
  # Run the container
  docker run -it --rm --privileged --name ${CONTAINER_NAME} \
	     --network host\
             -v ${HOST_DIRECTORY_PATH}:${CONTAINER_DIRECTORY_PATH} \
             -v /etc/timezone:/etc/timezone:ro -v /etc/localtime:/etc/localtime:ro \
	     -v /dev:/dev \
	     --device-cgroup-rule "c 81:* rmw" \
	     --device-cgroup-rule "c 189:* rmw" \
             ${IMAGE_NAME} /bin/bash
fi
