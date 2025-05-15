import copy
import dataclasses
import zenoh
from zenoh import Encoding
import time
from arca_sim import robot_model
import numpy as np
import mujoco
from mujoco import _mujoco as mj
from dataclasses import dataclass, field
from enum import Enum
from arca_sim.robot import Setpoint, Mode
from arca_sim.state import State

np.set_printoptions(
    precision=4,
    linewidth=300
)

l_leg_idx_qpos = [7, 8, 9, 10]
r_leg_idx_qpos = [11, 12, 13, 14]
l_leg_idx_ctrl = [0, 1, 2]
r_leg_idx_ctrl = [3, 4, 5]
l_motor_idx_pos = [7, 8, 9]
r_motor_idx_pos = [11, 12, 13]
l_motor_idx_vel = [6, 7, 8]
r_motor_idx_vel = [10, 11, 12]

def foot_jac_from_mujoco(model:mj.MjModel, data:mj.MjData, state:State)->tuple[np.ndarray,np.ndarray]:
    # l_foot_body = [model.body(i) for i in range(model.nbody) if 'onshape/foot_2' == model.body(i).name][0]
    # r_foot_body = [model.body(i) for i in range(model.nbody) if 'onshape/foot' == model.body(i).name][0]

    l_foot_body = [model.geom(i) for i in range(model.ngeom) if 'onshape//unnamed_geom_7' == model.geom(i).name][0]
    r_foot_body = [model.geom(i) for i in range(model.ngeom) if 'onshape//unnamed_geom_3' == model.geom(i).name][0]

    l_full_jac = np.zeros((6, model.nv))
    r_full_jac = np.zeros((6, model.nv))
    # mj.mj_jacBody(model, data, l_full_jac[:3], l_full_jac[3:], l_foot_body.id)
    # mj.mj_jacBody(model, data, r_full_jac[:3], r_full_jac[3:], r_foot_body.id)
    mj.mj_jacGeom(model, data, l_full_jac[:3], l_full_jac[3:], l_foot_body.id)
    mj.mj_jacGeom(model, data, r_full_jac[:3], r_full_jac[3:], r_foot_body.id)
    return l_full_jac, r_full_jac

def state_from_mujoco(model:mj.MjModel, data:mj.MjData, state:State):
    state.pos = data.qpos[0:3]
    state.quat = data.qpos[3:7]
    state.joint_pos = data.qpos[l_motor_idx_pos + r_motor_idx_pos]
    state.joint_vel = data.qvel[l_motor_idx_vel + r_motor_idx_vel]

    l_foot_jac, r_foot_jac = foot_jac_from_mujoco(model, data, state)

    # actuators are in reverse order :(
    # state.l_joint_force = l_foot_jac[:3, 10:13] @ data.qfrc_actuator[10:13]
    # state.r_joint_force = r_foot_jac[:3, 6:9] @ data.qfrc_actuator[6:9]
    state.l_joint_force = l_foot_jac[:3, 10:13] @ data.ctrl[3:6]
    state.r_joint_force = r_foot_jac[:3, 6:9] @ data.ctrl[0:3]


def subcribe_to_zenoh(z: zenoh.Session, path: str, state: State):
    state._zenoh_subs = []
    for f in dataclasses.fields(state):
        def callback(n):
            def cb(sample):
                setattr(state, n, np.frombuffer(sample.payload))
            return cb
        state._zenoh_subs.append(z.declare_subscriber(f"{path}/{f.name}", callback(f.name)))


def publish_to_zenoh(z: zenoh.Session, path: str, state: State):
    if not hasattr(state, "_zenoh_pubs"):
        state._zenoh_pubs = {}
        for f in dataclasses.fields(state):
            if not f.name.startswith('_'):
                state._zenoh_pubs[f.name] = z.declare_publisher(f"{path}/{f.name}")

    for f, pub in state._zenoh_pubs.items():
        pub.put(getattr(state, f).tobytes(), encoding=Encoding.APP_OCTET_STREAM)


def pd(q, qd, q_des, qd_des, kp, kd, ff):
    return kp * (q_des - q) + kd * (qd_des - qd) + ff


class SimLowLevelController:

    def update(self, model:mj.MjModel, data:mj.MjData, state:State, setpoint:Setpoint):
        torque = np.zeros(6)
        if setpoint.l_mode == Mode.JOINT:
            torque = pd(state.joint_pos, state.joint_vel, setpoint.des_joint_pos, setpoint.des_joint_vel, 10, 0.01, 0)
        elif setpoint.l_mode == Mode.FORCE:
            l_foot_jac, r_foot_jac = foot_jac_from_mujoco(model, data, state)
            # data.ctrl[3:6] = l_foot_jac[:3, 10:13].T @ setpoint.des_l_force
            # data.ctrl[0:3] = r_foot_jac[:3, 6:9].T @ setpoint.des_r_force
            
            # print(f"L Jac: {l_foot_jac[:3, 10:13]}")
            # print(f"R Jac: {r_foot_jac[:3, 6:9]}")
            # print(f"Des L: {setpoint.des_l_force}")
            # print(f"Des R: {setpoint.des_r_force}")

            torque[3:6] = l_foot_jac[:6, 10:13].T @ np.concatenate([setpoint.des_l_force, np.zeros(3)])
            torque[0:3] = r_foot_jac[:6, 6:9].T @ np.concatenate([setpoint.des_r_force, np.zeros(3)])

            # apply forces through each leg
        elif setpoint.l_mode == Mode.POS:
            l_foot_jac, r_foot_jac = foot_jac_from_mujoco(model, data, state)
            
            l_pos = (data.body("onshape/foot_2").xpos[:3] - data.body("onshape/").xpos[:3])
            r_pos = (data.body("onshape/foot").xpos[:3] - data.body("onshape/").xpos[:3])
            l_vel = (data.body("onshape/foot_2").cvel[:3] - data.body("onshape/").cvel[:3])
            r_vel = (data.body("onshape/foot").cvel[:3] - data.body("onshape/").cvel[:3])
            l_err = setpoint.des_l_pos - l_pos
            r_err = setpoint.des_r_pos - r_pos
            
            
            # setpoint.des_l_force = 1 * l_err + 0.1 * (l_foot_jac[:3, 10:13].T @ data.qvel[10:13])
            # setpoint.des_r_force = 1 * r_err + 0.1 * (r_foot_jac[:3, 6:9].T @ data.qvel[6:9]) 
            setpoint.des_l_force = 0.5 * l_err
            setpoint.des_r_force = 0.5 * r_err
            
            torque[3:6] = l_foot_jac[:3, 10:13].T @ setpoint.des_l_force
            torque[0:3] = r_foot_jac[:3, 6:9].T @ setpoint.des_r_force

        return torque


def main_passive():

    # model = MjModel.from_xml_path("models/urdf/robot.xml")
    model = robot_model.get_mujoco_model("models/full", fixed=True)
    data = mj.MjData(model)

    state_out = State()
    state_in = State()
    controller = SimLowLevelController()
    # setpt = Setpoint(
    #     l_mode=Mode.JOINT,
    #     r_mode=Mode.JOINT,
    #     # des_joint_pos=np.array([0, 0, 0, 0, 0, 0])
    #     des_joint_pos=np.array([0, np.deg2rad(46.81234), np.deg2rad(-90), 0, np.deg2rad(46.81234), np.deg2rad(-90)])
    # )
    setpt = Setpoint(
        l_mode=Mode.POS,
        r_mode=Mode.POS,
        # des_force=np.array([0, 0, -5, 0, -0, -5])
        des_pos=np.array([-0.038, 0.062, -0.25, -0.038, -0.062, -0.25])
        # des_pos=np.array([0, 0, 0.265, 0, 0, 0.265])
    )

    z = zenoh.open(zenoh.Config())
    publish_to_zenoh(z, "arca/state", state_out)
    subcribe_to_zenoh(z, "arca/state", state_in)

    with mj.viewer.launch_passive(model, data) as viewer:
        # data.qpos = [
        #     0, 0, 0.265, 1, 0, 0, 0,
        #     0, np.deg2rad(46.81234), np.deg2rad(-90), np.deg2rad(-45),
        #     0, np.deg2rad(46.81234), np.deg2rad(-90), np.deg2rad(-45)
        # ]
        # data.qpos = [
        #     0, 0, 0.265, 1, 0, 0, 0,
        #     0, np.deg2rad(30), np.deg2rad(-90), np.deg2rad(-45),
        #     0, np.deg2rad(30), np.deg2rad(-90), np.deg2rad(-45)
        # ]

        # for i in range(50):
        #     mj.mj_step(model, data)
        mj.mj_step(model, data)
        for i in range(data.ncon):
            geom1 = model.geom(data.contact[i].geom[0])
            geom2 = model.geom(data.contact[i].geom[1])
            print(f"Contact: {geom1.name} {geom2.name}")
        # mj.mj_forward(model, data)

        # interact with viewer
        with viewer.lock():
            viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = True
            viewer.opt.flags[mj.mjtVisFlag.mjVIS_PERTFORCE] = True
            
        while viewer.is_running():
            step_start = time.time()

            state_from_mujoco(model, data, state_out)
            publish_to_zenoh(z, "arca/state", state_out)

            # data.qpos[:7] = [0, 0, 0.261, 1, 0, 0, 0]
            data.qpos[:7] = [0, 0, 0.5, 1, 0, 0, 0]
            # data.ctrl = controller.update(state_in, setpt)
            # data.ctrl = controller.update(model, data, state_out, setpt)
            # data.ctrl = np.full(6, 0)
            # print(f"{data.ctrl}")
            # print(f"{state_out.l_joint_force}\t{state_out.r_joint_force}")
            # print(state_in.pos)

            mj.mj_step(model, data)
            
            viewer.sync()
            # input()           
            while time.time() - step_start < model.opt.timestep:
                pass

            
        viewer.close()


if __name__ == '__main__':
    # main()
    main_passive()
