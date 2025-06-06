import argparse
import mujoco
import mujoco.viewer
import rerun as rr

import numpy as np
from arca_sim import math_utils, sim_utils

np.set_printoptions(precision=3, suppress=True)

class Linkage:
    def __init__(
        self, 
        model: mujoco.MjModel,
        ir_jnt_names: list[str],
        or_jnt_names: list[str],
        il_site_names: list[str],
        ol_site_names: list[str],
    ):
        self.ir_jnt_names = ir_jnt_names
        self.or_jnt_names = or_jnt_names
        self.il_start_site_names = il_site_names
        self.ol_site_names = ol_site_names

        self.or_jnt_dofadrs = np.array([model.joint(n).dofadr[0] for n in or_jnt_names], dtype=np.int32)


    def ol_to_ir(self, model, data, or_eff_des: np.ndarray) -> None:

        ol_J_or_list = [] 
        for site_name in self.ol_site_names:
            J_site_full = sim_utils.get_site_jacobian(model, data, site_name)
            ol_J_or_list.append(J_site_full[:3, self.or_jnt_dofadrs])
        
        il_J_or_T = np.vstack(ol_J_or_list).T

        linkage_dirs_list = []
        for driving_site_name, driven_site_name in zip(self.il_start_site_names, self.ol_site_names):
            driving_site_pos_w = data.site(driving_site_name).xpos
            driven_site_pos_w = data.site(driven_site_name).xpos
            linkage_vec = driven_site_pos_w - driving_site_pos_w

            norm_linkage_vec = np.linalg.norm(linkage_vec)
            if norm_linkage_vec > 1e-9:
                linkage_dirs_list.append(linkage_vec / norm_linkage_vec)
            else:
                linkage_dirs_list.append(np.zeros(3))

        linkage_dir_diag_matrix = np.diag(np.concatenate(linkage_dirs_list))
        
        M = il_J_or_T @ linkage_dir_diag_matrix
        des_frcs_flat = np.linalg.pinv(M) @ or_eff_des
        
        des_frcs_list_scaled = np.split(linkage_dir_diag_matrix @ des_frcs_flat, len(self.il_start_site_names))

        current_ir_jnt_dofadrs = np.array([model.joint(n).dofadr[0] for n in self.ir_jnt_names], dtype=np.int32)
        current_ir_actuator_ids = np.array([model.actuator(n).id for n in self.ir_jnt_names], dtype=np.int32)

        for i in range(len(self.ir_jnt_names)):
            des_frc_scaled = des_frcs_list_scaled[i]
            driving_site_name = self.il_start_site_names[i]
            
            J_driving_site_full = sim_utils.get_site_jacobian(model, data, driving_site_name)
            ol_J_ir_vec = J_driving_site_full[:3, current_ir_jnt_dofadrs[i]]
            
            input_joint_effort_des = ol_J_ir_vec.T @ des_frc_scaled

            data.ctrl[current_ir_actuator_ids[i]] = input_joint_effort_des


def main(args):
    rr.init("arca_sim", spawn=True)
    
    # Load the MuJoCo model from the specified XML file
    model = mujoco.MjModel.from_xml_path(f"models/{args.model}/mjcf/scene.xml")
    data = mujoco.MjData(model)

    def key_cb(keycode):
        if chr(keycode) == 'R':
            gantry_eq_id = model.equality("gantry").id
            if data.eq_active[gantry_eq_id] == 1:
                data.eq_active[gantry_eq_id] = 0
            else:
                data.eq_active[gantry_eq_id] = 1
                mujoco.mj_resetDataKeyframe(model, data, model.keyframe("home").id)
            print("Releasing gantry")

    print(f"Bodies:")
    for i in range(model.nbody):
        print(f"{model.body(i).name}")

    print(f"Equalities:")
    for i in range(model.neq):
        print(f"{model.equality(i).name}")

    print(f"Joints:")
    for i in range(model.njnt):
        print(f"{model.joint(i).name}")

    print(f"Sites:")
    for i in range(model.nsite):
        # print(f"{model.site(i).name}")
        print(f"{data.site(i)}")

    l_hip_linkage = Linkage(
        model,
        ir_jnt_names=["l_hip_inner", "l_hip_outer"],
        or_jnt_names=["l_hip_roll_passive", "l_hip_pitch_passive"],
        il_site_names=["l_hip_inner_arm", "l_hip_outer_arm"],
        ol_site_names=["closing_l_hip_inner_2", "closing_l_hip_outer_2"],
    )
    l_knee_linkage = Linkage(
        model,
        ir_jnt_names=["l_knee"],
        or_jnt_names=["l_knee_passive"],
        il_site_names=["l_knee_arm_end"],
        ol_site_names=["closing_l_knee_linkage_2"],
    )
    r_hip_linkage = Linkage(
        model,
        ir_jnt_names=["r_hip_inner", "r_hip_outer"],
        or_jnt_names=["r_hip_roll_passive", "r_hip_pitch_passive"],
        il_site_names=["r_hip_inner_arm", "r_hip_outer_arm"],
        ol_site_names=["closing_r_hip_inner_2", "closing_r_hip_outer_2"],
    )
    r_knee_linkage = Linkage(
        model,
        ir_jnt_names=["r_knee"],
        or_jnt_names=["r_knee_passive"],
        il_site_names=["r_knee_arm_end"],
        ol_site_names=["closing_r_knee_linkage_1"],
    )

    def ctrl_loop(model, data):
        rr.set_time("mujoco", duration=data.time)
        
        kp = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
        kd = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        or_jnt_names = ["l_hip_roll_passive", "l_hip_pitch_passive", "r_hip_roll_passive", "r_hip_pitch_passive", "l_knee_passive", "r_knee_passive"]
        
        # Get qpos and dof addresses for output rotary (or) joints
        # Using dofadr for velocity-based calculations (Jacobians)
        # model.joint().qposadr and .dofadr return 1-element arrays, so [0] extracts the scalar.
        # Using int32 for numpy indexing.
        or_jnt_qposadrs = np.array([model.joint(n).qposadr[0] for n in or_jnt_names], dtype=np.int32)
        or_jnt_dofadrs = np.array([model.joint(n).dofadr[0] for n in or_jnt_names], dtype=np.int32)

        des_or_pos_rad = np.zeros(6)
        des_or_vel_rad = np.zeros(6)
        
        m = model.body("trunk").subtreemass

        # des_l_hip_pitch, des_l_hip_pitch_vel = math_utils.sin_traj(data.time, 1.0, np.deg2rad(-45.0), np.deg2rad(20.0))
        # des_or_pos_rad[1] = des_l_hip_pitch
        # des_or_vel_rad[1] = des_l_hip_pitch_vel
        
        current_or_pos_rad = data.qpos[or_jnt_qposadrs]
        or_eff_des = kp * (des_or_pos_rad - current_or_pos_rad) + kd * (des_or_vel_rad - data.qvel[or_jnt_dofadrs])
        
        l_hip_linkage.ol_to_ir(model, data, or_eff_des[0:2])
        r_hip_linkage.ol_to_ir(model, data, or_eff_des[2:4])
        l_knee_linkage.ol_to_ir(model, data, or_eff_des[4:5])
        r_knee_linkage.ol_to_ir(model, data, or_eff_des[5:6])


    # Launch the viewer in passive mode
    with mujoco.viewer.launch_passive(model, data, key_callback=key_cb) as viewer:
        mujoco.mj_resetDataKeyframe(model, data, model.keyframe("home").id)

        ctrl_rate = 1000
        ctrl_decimation = (1 / model.opt.timestep) // ctrl_rate
        ctr = 0
        while viewer.is_running():
            mujoco.mj_step(model, data)
            if ctr % ctrl_decimation == 0:
                ctrl_loop(model, data)
            ctr += 1
            viewer.sync()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="View a MuJoCo model.")
    parser.add_argument(
        "--model",
        type=str,
        # default="reduced",
        default="full",
        help="Name of a model directory under models/",
    )
    args = parser.parse_args()

    main(args)