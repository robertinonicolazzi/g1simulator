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

            msg = PointCloud2_()
            
            # Header
            msg.header = Header_()
            msg.header.frame_id = frame_id
            now = time.time()
            msg.header.stamp = Time_(sec=int(now), nanosec=int((now - int(now)) * 1e9))

            # PointCloud2 fields
            msg.height = 1
            msg.width = len(points)
            
            # Define fields: x, y, z (float32)
            # PointField (name, offset, datatype, count)
            # FLOAT32 = 7
            msg.fields = [
                PointField_(name="x", offset=0, datatype=7, count=1),
                PointField_(name="y", offset=4, datatype=7, count=1),
                PointField_(name="z", offset=8, datatype=7, count=1)
            ]
            
            msg.is_bigendian = False
            msg.point_step = 12 # 3 * 4 bytes
            msg.row_step = msg.point_step * msg.width
            msg.is_dense = True
            
            # Convert to bytes
            # Ensure points are float32
            points_f32 = points.astype(np.float32)
            msg.data = points_f32.tobytes() # returns list[int] or bytes? 
            # SDK msg.data is usually list[int] (sequence<uint8>)
            # In Python SDK dds_, it often expects list of ints or bytes depending on implementation.
            # Step 120 (sdk_repo.md) shows String_ using str.
            # PointCloud2_ data is 'sequence<uint8>'.
            # Let's try passing list(msg.data) if bytes fail, or check example.
            # example/b2/camera/camera_opencv.py uses np.frombuffer(data, dtype=np.uint8) on RECEIVE.
            # So send should be bytes or list of uint8.
            msg.data = list(points_f32.tobytes())

            self.cmd_pub.Write(msg)
            
        except Exception as e:
            print(f"[{self.node_name}] Error publishing lidar: {e}")
