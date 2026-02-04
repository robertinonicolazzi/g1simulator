# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

import numpy as np
import time
from dds.dds_base import DDSObject
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointField_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_

class LidarDDS(DDSObject):
    """Lidar DDS communication class"""
    
    def __init__(self, node_name: str = "lidar"):
        super().__init__()
        self.node_name = node_name
        self.cmd_pub = None

    def setup_publisher(self):
        self.cmd_pub = ChannelPublisher("rt/utlidar/cloud_livox_mid360", PointCloud2_)
        self.cmd_pub.Init()
        print(f"[{self.node_name}] Lidar publisher initialized on 'rt/utlidar/cloud_livox_mid360'")

    def setup_subscriber(self):
        pass

    def dds_publisher(self):
        # Publishing is handled manually in sim_main.py to synchronize with physics steps
        pass

    def dds_subscriber(self, msg, datatype=None):
        pass

    def publish(self, points, frame_id="livox_frame"):
        """
        Publish point cloud data.
        Args:
            points: numpy array of shape (N, 3) containing x, y, z coordinates
            frame_id: frame ID for the header
        """
        try:
            if points is None or len(points) == 0:
                return

            # Header
            header = Header_()
            header.frame_id = frame_id
            now = time.time()
            header.stamp = Time_(sec=int(now), nanosec=int((now - int(now)) * 1e9))

            # PointCloud2 fields
            height = 1
            width = len(points)
            
            # Define fields: x, y, z (float32)
            # PointField (name, offset, datatype, count)
            # FLOAT32 = 7
            fields = [
                PointField_(name="x", offset=0, datatype=7, count=1),
                PointField_(name="y", offset=4, datatype=7, count=1),
                PointField_(name="z", offset=8, datatype=7, count=1)
            ]
            
            is_bigendian = False
            point_step = 12 # 3 * 4 bytes
            row_step = point_step * width
            is_dense = True
            
            # Convert to bytes
            # Ensure points are float32
            points_f32 = points.astype(np.float32)
            data = list(points_f32.tobytes())

            msg = PointCloud2_(
                header,
                height,
                width,
                fields,
                is_bigendian,
                point_step,
                row_step,
                data,
                is_dense
            )

            self.cmd_pub.Write(msg)
            
        except Exception as e:
            print(f"[{self.node_name}] Error publishing lidar: {e}")
