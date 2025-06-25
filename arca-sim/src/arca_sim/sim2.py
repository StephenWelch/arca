import argparse
from typing import Literal
import mujoco
import mujoco.viewer
import zenoh
import numpy as np
from arca_sim.state import LowLevelState, LowLevelCommand
from arca_sim.robot_model import get_scene_path


class MujocoSim:

    def __init__(self, model_type: Literal["full", "reduced"], viewer: bool = False):
        self.model_path = get_scene_path(model_type)
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)

        if viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        else:
            self.viewer = None

        self.jnt_names = ["l_hip_inner", "l_hip_outer",
                          "l_knee", "r_hip_inner", "r_hip_outer", "r_knee"]
        self.jnt_qpos_idx = [self.model.joint(
            jnt_name).qposadr for jnt_name in self.jnt_names]
        self.jnt_qvel_idx = [i + 1 for i in self.jnt_qpos_idx]
        self.jnt_ctrl_idx = [self.model.actuator(n).id for n in self.jnt_names]


    def _compute_state(self) -> LowLevelState:
        return LowLevelState(
            gyro=self.data.qvel[3:6],
            quat=self.data.qpos[3:7],
            pos=self.data.qpos[self.jnt_qpos_idx].squeeze(),
            vel=self.data.qvel[self.jnt_qvel_idx].squeeze(),
            current=self.data.ctrl[self.jnt_ctrl_idx].squeeze()
        )

    def reset(self) -> LowLevelState:
        mujoco.mj_resetData(self.model, self.data)
        return self._compute_state()

    def step(self, ll_cmd: LowLevelCommand) -> LowLevelState:
        ll_state = self._compute_state()

        ll_state.current = ll_cmd.kp * (ll_cmd.pos - ll_state.pos) + ll_cmd.kd * (
            ll_cmd.vel - ll_state.vel) + ll_cmd.current
        self.data.ctrl[self.jnt_ctrl_idx] = ll_state.current

        mujoco.mj_step(self.model, self.data)

        return self._compute_state()

    def is_running(self) -> bool:
        if self.viewer:
            return self.viewer.is_running()
        else:
            return True
    
    def close(self):
        self.z.close()
        if self.viewer:
            self.viewer.close()
