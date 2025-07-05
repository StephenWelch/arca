import numpy as np
from dataclasses import dataclass, field
from . import math_utils

@dataclass
class State:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat: np.ndarray = field(default_factory=lambda: np.zeros(4))
    joint_pos: np.ndarray = field(default_factory=lambda: np.zeros(6))
    joint_vel: np.ndarray = field(default_factory=lambda: np.zeros(6))
    joint_force: np.ndarray = field(default_factory=lambda: np.zeros(6))
    _zenoh_subs: list = field(default_factory=list)
    _zenoh_pubs: dict = field(default_factory=dict)

    @property
    def rpy(self)->np.ndarray:
        return math_utils.rpy_from_quat(self.quat)

    @property
    def l_joint_pos(self)->np.ndarray:
        return self.joint_pos[:3]

    @l_joint_pos.setter
    def l_joint_pos(self, pos:np.ndarray):
        self.joint_pos[:3] = pos

    @property
    def r_joint_pos(self)->np.ndarray:
        return self.joint_pos[3:]

    @r_joint_pos.setter
    def r_joint_pos(self, pos:np.ndarray):
        self.joint_pos[3:] = pos

    @property
    def l_joint_vel(self)->np.ndarray:
        return self.joint_vel[:3]

    @l_joint_vel.setter
    def l_joint_vel(self, vel:np.ndarray):
        self.joint_vel[:3] = vel

    @property
    def r_joint_vel(self)->np.ndarray:
        return self.joint_vel[3:]

    @r_joint_vel.setter
    def r_joint_vel(self, vel:np.ndarray):
        self.joint_vel[3:] = vel

    @property
    def l_joint_force(self)->np.ndarray:
        return self.joint_force[:3]

    @l_joint_force.setter
    def l_joint_force(self, force: np.ndarray):
        self.joint_force[:3] = force

    @property
    def r_joint_force(self)->np.ndarray:
        return self.joint_force[3:]

    @r_joint_force.setter
    def r_joint_force(self, force:np.ndarray):
        self.joint_force[3:] = force

    @property
    def l_hip_roll(self)->float:
        return self.joint_pos[0]

    @l_hip_roll.setter
    def l_hip_roll(self, rad:float):
        self.joint_pos[0] = rad

    @property
    def l_hip_pitch(self):
        return self.joint_pos[1]

    @l_hip_pitch.setter
    def l_hip_pitch(self, rad:float):
        self.joint_pos[1] = rad

    @property
    def l_knee_pitch(self)->float:
        return self.joint_pos[2]

    @l_knee_pitch.setter
    def l_knee_pitch(self, rad:float):
        self.joint_pos[2] = rad

    @property
    def r_hip_roll(self)->float:
        return self.joint_pos[3]

    @r_hip_roll.setter
    def r_hip_roll(self, rad:float):
        self.joint_pos[3] = rad

    @property
    def r_hip_pitch(self)->float:
        return self.joint_pos[4]

    @r_hip_pitch.setter
    def r_hip_pitch(self, rad:float):
        self.joint_pos[4] = rad

    @property
    def r_knee_pitch(self)->float:
        return self.joint_pos[5]

    @r_knee_pitch.setter
    def r_knee_pitch(self, rad:float):
        self.joint_pos[5] = rad

    @property
    def l_hip_roll_vel(self)->float:
        return self.joint_vel[0]

    @l_hip_roll_vel.setter
    def l_hip_roll_vel(self, rad:float):
        self.joint_vel[0] = rad

    @property
    def l_hip_pitch_vel(self)->float:
        return self.joint_vel[1]

    @l_hip_pitch_vel.setter
    def l_hip_pitch_vel(self, rad:float):
        self.joint_vel[1] = rad

    @property
    def l_knee_pitch_vel(self)->float:
        return self.joint_vel[2]

    @l_knee_pitch_vel.setter
    def l_knee_pitch_vel(self, rad:float):
        self.joint_vel[2] = rad

    @property
    def r_hip_roll_vel(self)->float:
        return self.joint_vel[3]

    @r_hip_roll_vel.setter
    def r_hip_roll_vel(self, rad:float):
        self.joint_vel[3] = rad

    @property
    def r_hip_pitch_vel(self)->float:
        return self.joint_vel[4]

    @r_hip_pitch_vel.setter
    def r_hip_pitch_vel(self, rad:float):
        self.joint_vel[4] = rad

    @property
    def r_knee_pitch_vel(self)->float:
        return self.joint_vel[5]

    @r_knee_pitch_vel.setter
    def r_knee_pitch_vel(self, rad:float):
        self.joint_vel[5] = rad 