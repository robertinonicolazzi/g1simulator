# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
public sensor configuration
"""

from isaaclab.sensors.ray_caster import MultiMeshRayCasterCfg, patterns
from isaaclab.utils import configclass


@configclass
class SensorPresets:
    """sensor preset configuration collection"""
    
    @classmethod
    def livox_mid360(cls) -> MultiMeshRayCasterCfg:
        """Livox Mid360 Lidar configuration
        
        Approximate specs:
        - 360 horizontal FOV
        - ~59 vertical FOV
        - 10Hz update rate
        """
        return MultiMeshRayCasterCfg(
            prim_path="/World/envs/env_.*/Robot/mid360_link",
            update_period=0.1, # 10Hz
            offset=MultiMeshRayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.1)), # Offset from head link
            mesh_prim_paths=[
                "/World/Ground",
                MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_.*/Room/.*"),
                MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_.*/Room/Assets/.*"),
            ],
            ray_alignment="world",
            pattern_cfg=patterns.LidarPatternCfg(
                channels=32,
                vertical_fov_range=(-7.0, 52.0),
                horizontal_fov_range=(-180.0, 180.0),
                horizontal_res=0.2, # Approximate resolution
            ),
            debug_vis=False,
        )