#!/bin/bash

IMAGE_NAME="g1_sim_controller"
CONTAINER_NAME="g1_sim_controller"
PROJECT_ROOT="$(cd "$(dirname "$0")" && cd .. && pwd)"

echo "Building Docker image ${IMAGE_NAME}..."
docker build -t ${IMAGE_NAME} "${PROJECT_ROOT}/g1_sim_controller" || { echo "Docker build failed, exiting."; exit 1; }

if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    echo "Stopping and removing existing container ${CONTAINER_NAME}..."
    docker stop ${CONTAINER_NAME} >/dev/null 2>&1
    docker rm ${CONTAINER_NAME} >/dev/null 2>&1
fi

echo "Running ${CONTAINER_NAME} as a daemon..."
docker run -dt \
    --name ${CONTAINER_NAME} \
    --network host \
    --ipc host \
    --restart unless-stopped \
    -v "${PROJECT_ROOT}/g1_sim_controller:/ros_ws" \
    -e ROS_DOMAIN_ID=0 \
    ${IMAGE_NAME}

echo "Container started. Use 'docker logs -f ${CONTAINER_NAME}' to follow logs."
