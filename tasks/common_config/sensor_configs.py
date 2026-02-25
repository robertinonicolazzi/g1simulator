# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
public sensor configuration
"""

from isaaclab.sensors import RayCasterCfg, patterns
from isaaclab.utils import configclass

@configclass
class SensorPresets:
    """sensor preset configuration collection"""
    
    @classmethod
    def livox_mid360(cls) -> RayCasterCfg:
        """Livox Mid360 Lidar configuration
        
        Approximate specs:
        - 360 horizontal FOV
        - ~59 vertical FOV
        - 10Hz update rate
        """
        return RayCasterCfg(
            prim_path="/World/envs/env_.*/Robot/mid360_link",
            update_period=0.1, # 10Hz
            offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.1)), # Offset from head link
            attach_yaw_only=False,
            pattern_cfg=patterns.LidarPatternCfg(
                channels=32,
                vertical_fov_range=(-7.0, 52.0),
                horizontal_fov_range=(-180.0, 180.0),
                horizontal_res=0.2, # Approximate resolution
            ),
            debug_vis=False,
            mesh_prim_paths=["/World/envs/env_0/PackingTable_2/PackingTable_2/container_h20/container_h20_inst/Container_H20_01"], # Detect room walls
        )
