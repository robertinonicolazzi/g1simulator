#!/bin/bash

docker run --gpus all -it --rm --network host --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics,display \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /etc/vulkan/icd.d:/etc/vulkan/icd.d:ro \
  -v /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:ro \
  -v /home/rc/inorbit_g1_sim/assets:/home/code/unitree_sim_isaaclab/assets \
  -v /home/rc/inorbit_g1_sim/sim_main.py:/home/code/unitree_sim_isaaclab/sim_main.py \
  -v /home/rc/inorbit_g1_sim/dds:/home/code/unitree_sim_isaaclab/dds \
  -v /home/rc/inorbit_g1_sim/tasks:/home/code/unitree_sim_isaaclab/tasks \
  inorbit_g1_sim:latest /bin/bash
