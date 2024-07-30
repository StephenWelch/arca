import util
import numpy as np
from enum import Enum
from dataclasses import dataclass, field

class Mode(Enum):
    JOINT = 0
    FORCE = 1
    POS = 2

@dataclass
class Setpoint:
    l_mode: Mode
    r_mode: Mode
    des_joint_pos: np.ndarray = field(default_factory=lambda: np.zeros(6))
    des_joint_vel: np.ndarray = field(default_factory=lambda: np.zeros(6))
    des_force: np.ndarray = field(default_factory=lambda: np.zeros(6))
    des_pos: np.ndarray = field(default_factory=lambda: np.zeros(6))

    @property
    def des_l_joint_pos(self)->np.ndarray:
        return self.des_joint_pos[:3]

    @des_l_joint_pos.setter
    def des_l_joint_pos(self, pos:np.ndarray):
        self.des_joint_pos[:3] = pos

    @property
    def des_r_joint_pos(self)->np.ndarray:
        return self.des_joint_pos[3:]

    @des_r_joint_pos.setter
    def des_r_joint_pos(self, pos:np.ndarray):
        self.des_joint_pos[3:] = pos

    @property
    def des_l_joint_vel(self)->np.ndarray:
        return self.des_joint_vel[:3]

    @des_l_joint_vel.setter
    def des_l_joint_vel(self, vel:np.ndarray):
        self.des_joint_vel[:3] = vel

    @property
    def des_r_joint_vel(self)->np.ndarray:
        return self.des_joint_vel[3:]

    @des_r_joint_vel.setter
    def des_r_joint_vel(self, vel:np.ndarray):
        self.des_joint_vel[3:] = vel

    @property
    def des_l_force(self)->np.ndarray:
        return self.des_force[:3]

    @des_l_force.setter
    def des_l_force(self, force:np.ndarray):
        self.des_force[:3] = force

    @property
    def des_r_force(self)->np.ndarray:
        return self.des_force[3:]

    @des_r_force.setter
    def des_r_force(self, force:np.ndarray):
        self.des_force[3:] = force

    @property
    def des_l_pos(self)->np.ndarray:
        return self.des_pos[:3]
    
    @des_l_pos.setter
    def des_l_pos(self, pos:np.ndarray):
        self.des_pos[:3] = pos

    @property
    def des_r_pos(self)->np.ndarray:
        return self.des_pos[3:]
    
    @des_r_pos.setter
    def des_r_pos(self, pos:np.ndarray):
        self.des_pos[3:] = pos

    @property
    def des_l_hip_roll(self)->float:
        return self.des_joint_pos[0]

    @des_l_hip_roll.setter
    def des_l_hip_roll(self, rad:float)->float:
        return self.des_joint_pos[0]

    @property
    def des_l_hip_pitch(self)->float:
        return self.des_joint_pos[1]

    @des_l_hip_pitch.setter
    def des_l_hip_pitch(self, rad:float):
        self.des_joint_pos[1] = rad

    @property
    def des_l_knee_pitch(self)->float:
        return self.des_joint_pos[2]

    @des_l_knee_pitch.setter
    def des_l_knee_pitch(self, rad:float):
        self.des_joint_pos[2] = rad

    @property
    def des_r_hip_roll(self)->float:
        return self.des_joint_pos[3]

    @des_r_hip_roll.setter
    def des_r_hip_roll(self, rad:float):
        self.des_joint_pos[3] = rad

    @property
    def des_r_hip_pitch(self)->float:
        return self.des_joint_pos[4]

    @des_r_hip_pitch.setter
    def des_r_hip_pitch(self, rad:float):
        self.des_joint_pos[4] = rad

    @property
    def des_r_knee_pitch(self)->float:
        return self.des_joint_pos[5]

    @des_r_knee_pitch.setter
    def des_r_knee_pitch(self, rad:float):
        self.des_joint_pos[5] = rad

    @property
    def des_l_hip_roll_vel(self)->float:
        return self.des_joint_vel[0]

    @des_l_hip_roll_vel.setter
    def des_l_hip_roll_vel(self, rad:float):
        self.des_joint_vel[0] = rad

    @property
    def des_l_hip_pitch_vel(self)->float:
        return self.des_joint_vel[1]

    @des_l_hip_pitch_vel.setter
    def des_l_hip_pitch_vel(self, rad:float):
        self.des_joint_vel[1] = rad

    @property
    def des_l_knee_pitch_vel(self)->float:
        return self.des_joint_vel[2]

    @des_l_knee_pitch_vel.setter
    def des_l_knee_pitch_vel(self, rad:float):
        self.des_joint_vel[2] = rad

    @property
    def des_r_hip_roll_vel(self)->float:
        return self.des_joint_vel[3]

    @des_r_hip_roll_vel.setter
    def des_r_hip_roll_vel(self, rad:float):
        self.des_joint_vel[3] = rad

    @property
    def des_r_hip_pitch_vel(self)->float:
        return self.des_joint_vel[4]

    @des_r_hip_pitch_vel.setter
    def des_r_hip_pitch_vel(self, rad:float):
        self.des_joint_vel[4] = rad

    @property
    def des_r_knee_pitch_vel(self)->float:
        return self.des_joint_vel[5]

    @des_r_knee_pitch_vel.setter
    def des_r_knee_pitch_vel(self, rad:float):
        self.des_joint_vel[5] = rad

@dataclass
class State:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat: np.ndarray = field(default_factory=lambda: np.zeros(4))
    joint_pos: np.ndarray = field(default_factory=lambda: np.zeros(6))
    joint_vel: np.ndarray = field(default_factory=lambda: np.zeros(6))
    joint_force: np.ndarray = field(default_factory=lambda: np.zeros(6))

    @property
    def rpy(self)->np.ndarray:
        return util.rpy_from_quat(self.quat)

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
