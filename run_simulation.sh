#!/bin/bash

TELEIMAGER_LOG_LEVEL=WARNING python sim_main.py \
    --task Isaac-Move-Cylinder-G129-Dex1-Wholebody \
    --robot_type g129 \
    --enable_dex1_dds \
    --headless \
    --enable_cameras \
    --livestream 2 \
    --camera_include "front_camera,robot_camera" \
    --camera_exclude "left_wrist_camera,right_wrist_camera,world_camera" \
    --camera_jpeg_quality 70 \
    --solver_iterations 4 \
    --skip_cvtcolor \
    --rendering_mode performance \
    --/rtx/verifyDriverVersion/enabled=false
