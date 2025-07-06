# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.managers import TerminationTermCfg as DoneTerm

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
import arca_rl.tasks.manager_based.arca_rl.mdp as arca_mdp
from .velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg

##
# Pre-defined configs
##
from arca_rl.assets import ARCA_CFG  # isort: skip

BASE_BODY_NAME = "trunk"
HIP_ROLL_JNT_NAME = ".*_hip_roll"
HIP_PITCH_JNT_NAME = ".*_hip_pitch"
FOOT_JNT_NAME = ".*_ankle_passive"
FOOT_BODY_NAME = "foot.*"



@configclass
class ArcaRewardsCfg(RewardsCfg):
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(
        func=arca_mdp.feet_air_time,
        weight=2.5,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODY_NAME),
            "command_name": "base_velocity",
            "threshold_min": 0.1,
            "threshold_max": 0.3,
        },
    )
    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[HIP_ROLL_JNT_NAME, HIP_PITCH_JNT_NAME])},
    )
    joint_deviation_toes = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=FOOT_JNT_NAME)},
    )
    foot_height = RewTerm(
        func=arca_mdp.foot_height_exp,
        weight=1.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODY_NAME),
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_BODY_NAME),
            "height": 0.05,
            "std": 0.25,
        },
    )
    # penalize toe joint limits
    # dof_pos_limits = RewTerm(
    #     func=mdp.joint_pos_limits,
    #     weight=-1.0,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=FOOT_JNT_NAME)},
    # )

@configclass
class ArcaTerminationsCfg():
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_low = DoneTerm(
        func=mdp.root_height_below_minimum,
        params={
            "minimum_height": 0.23,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

@configclass
class ArcaRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Arca rough environment configuration."""

    rewards: ArcaRewardsCfg = ArcaRewardsCfg()
    terminations: ArcaTerminationsCfg = ArcaTerminationsCfg()

    def __post_init__(self):
        super().__post_init__()
        # scene
        self.scene.robot = ARCA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/trunk"

        # actions
        self.actions.joint_pos.scale = 0.5

        # events
        self.events.push_robot = None
        self.events.add_base_mass = None
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.base_external_force_torque.params["asset_cfg"].body_names = ["trunk"]
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }
        self.events.base_com = None

        # terminations
        # self.terminations.base_contact.params["sensor_cfg"].body_names = ["trunk", "thigh_lower_mount.*"]

        # rewards
        self.rewards.undesired_contacts = None
        self.rewards.dof_torques_l2.weight = -5.0e-6
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 1.0
        self.rewards.action_rate_l2.weight *= 1.5
        self.rewards.dof_acc_l2.weight *= 1.5


@configclass
class ArcaRoughEnvCfg_PLAY(ArcaRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (0.7, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        # disable randomization for play
        self.observations.policy.enable_corruption = False
