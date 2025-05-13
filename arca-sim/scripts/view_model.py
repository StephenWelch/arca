import argparse
import mujoco
import mujoco.viewer

import numpy as np
from arca_sim import util

np.set_printoptions(precision=3, suppress=True)

def get_body_jacobian(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> np.ndarray:
    body_id = model.body(body_name).id
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
    return np.vstack((jacp, jacr))

def get_site_jacobian(model: mujoco.MjModel, data: mujoco.MjData, site_name: str) -> np.ndarray:
    site_id = model.site(site_name).id
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    return np.vstack((jacp, jacr))

def main(args):
    # Load the MuJoCo model from the specified XML file
    model = mujoco.MjModel.from_xml_path(f"models/{args.model}/mjcf/scene.xml")
    data = mujoco.MjData(model)

    def key_cb(keycode):
        if chr(keycode) == 'R':
            gantry_eq_id = model.equality("gantry").id
            data.eq_active[gantry_eq_id] ^= 1
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


    def ctrl_loop(model, data):
        
        # driving_joints = ["l_hip_inner", "l_hip_outer"]
        driven_joint_name = "l_hip_roll_passive"
        driving_joint_names = ["l_hip_inner", "l_hip_outer"]
        driving_site_names = ["closing_l_hip_inner_1", "closing_l_hip_outer_1"]
        driven_site_names = ["closing_l_hip_inner_2", "closing_l_hip_outer_2"]
        # qvel_adrs = [model.joint(n).qposadr-1 for n in driving_joint_names]

        des_or_pos = np.deg2rad(0.0)
        or_pos = data.joint(driven_joint_name).qpos
        des_or_eff = 100.0*(des_or_pos - or_pos)
        print(f"{np.rad2deg(des_or_pos)=} {np.rad2deg(or_pos)=}")
        
        il_J_or = [] # Map from driven joint (output rotary) to driving site (input linear)
        for driven_site in driven_site_names:
            nvadr = model.joint(driven_joint_name).qposadr-1
            il_J_or.append(get_site_jacobian(model, data, driven_site)[:3, nvadr])

        linkage_dirs = []
        for driving_site_name, driven_site_name in zip(driving_site_names, driven_site_names):
            driving_site_pos_w = data.site(driving_site_name).xpos
            driven_site_pos_b = data.site(driven_site_name).xpos
            linkage_dir = driven_site_pos_b - driving_site_pos_w
            linkage_dirs.append(linkage_dir / np.linalg.norm(linkage_dir))
        # print(f"{np.vstack(il_J_or).T.shape=}")
        # print(f"{np.diag(np.concat(linkage_dirs)).shape=}")

        linkage_dir_diag = np.diag(np.concat(linkage_dirs))
        des_frcs_mat = np.linalg.pinv(np.vstack(il_J_or).T @ linkage_dir_diag) @ des_or_eff
        # des_frcs = linkage_dir_diag @ des_frcs_mat
        des_frcs = np.split(linkage_dir_diag @ des_frcs_mat, len(driving_site_names)) # Split into n groups of 3

        for des_frc, driving_site_name, driving_joint_name in zip(des_frcs, driving_site_names, driving_joint_names):
            nvadr = model.joint(driving_joint_name).qposadr-1
            ol_J_ir = get_site_jacobian(model, data, driving_site_name)[:3, nvadr]
            des_or_eff = ol_J_ir.T @ des_frc
            data.ctrl[model.actuator(driving_joint_name).id] = des_or_eff

        # print(des_frc)

        # J = compute_effective_jacobian(model, data, "thigh_lower_mount")
        # print(J)
        
        

    # Launch the viewer in passive mode
    with mujoco.viewer.launch_passive(model, data, key_callback=key_cb) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            ctrl_loop(model, data)
            viewer.sync()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="View a MuJoCo model.")
    parser.add_argument(
        "--model",
        type=str,
        default="reduced",
        help="Name of a model directory under models/",
    )
    args = parser.parse_args()

    main(args)