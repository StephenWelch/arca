# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
from isaaclab.utils import configclass

from .rough_env_cfg import ArcaRoughEnvCfg


@configclass
class ArcaFlatEnvCfg(ArcaRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # actions
        self.actions.joint_pos.scale = 0.25
        # commands
        self.commands.base_velocity.ranges.lin_vel_x = (-0.5, 0.5)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.5, 0.5)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.5, 0.5)
        self.commands.base_velocity.rel_standing_envs = 0.1
        # rewards
        self.rewards.flat_orientation_l2.weight = -2.5
        self.rewards.feet_air_time.weight = 5.0
        self.rewards.track_lin_vel_xy_exp.params["std"] = math.sqrt(0.1)
        self.rewards.track_ang_vel_z_exp.params["std"] = math.sqrt(0.1)
        # self.rewards.joint_deviation_hip.params["asset_cfg"].joint_names = ["hip_rotation_.*"]
        # change terrain to flat
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None


class ArcaFlatEnvCfg_PLAY(ArcaFlatEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
