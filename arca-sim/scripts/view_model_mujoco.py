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
        or_jnt_names = ["l_hip_roll_passive", "l_hip_pitch_passive"]
        ir_jnt_names = ["l_hip_inner", "l_hip_outer"]
        il_site_names = ["closing_l_hip_inner_1", "closing_l_hip_outer_1"]
        ol_site_names = ["closing_l_hip_inner_2", "closing_l_hip_outer_2"]
        
        # Get qpos and dof addresses for output rotary (or) joints
        # Using dofadr for velocity-based calculations (Jacobians)
        # model.joint().qposadr and .dofadr return 1-element arrays, so [0] extracts the scalar.
        # Using int32 for numpy indexing.
        or_jnt_qposadrs = np.array([model.joint(n).qposadr[0] for n in or_jnt_names], dtype=np.int32)
        or_jnt_dofadrs = np.array([model.joint(n).dofadr[0] for n in or_jnt_names], dtype=np.int32)

        kp = np.array([10.0, 10.0])
        kd = np.array([1.0, 1.0])
        # Desired positions for output rotary joints, converted to radians immediately
        des_or_pos_rad = np.deg2rad(np.array([0.0, 0.0]))
        
        current_or_pos_rad = data.qpos[or_jnt_qposadrs]
        # Calculate desired effort for output rotary joints using a PD controller (P-term only here)
        or_eff_des = kp * (des_or_pos_rad - current_or_pos_rad) + kd * (0.0 - data.qvel[or_jnt_dofadrs])
        
        # Debug printing (values in degrees for readability)
        # Using new variable names for clarity in the print output
        print(f"des_or_pos_rad (deg)={np.rad2deg(des_or_pos_rad)}\ncurrent_or_pos_rad (deg)={np.rad2deg(current_or_pos_rad)}")
        
        # Calculate Jacobians: mapping output rotary joint velocities to linear velocities of "output linkage" sites (ol_site_names)
        # Each Jacobian J_site_vs_or_dofs will have shape (3, num_or_joints)
        il_J_or_list = [] 
        for site_name in ol_site_names:
            J_site_full = get_site_jacobian(model, data, site_name) # Shape (6, model.nv)
            # Select translational part (rows 0,1,2) and columns corresponding to or_jnt_dofadrs
            il_J_or_list.append(J_site_full[:3, or_jnt_dofadrs])
        
        # Stack Jacobians into a single matrix, shape (3 * num_ol_sites, num_or_joints)
        stacked_il_J_or = np.vstack(il_J_or_list)

        # Calculate linkage direction unit vectors
        linkage_dirs_list = []
        for driving_site_name, driven_site_name in zip(il_site_names, ol_site_names):
            driving_site_pos_w = data.site(driving_site_name).xpos
            driven_site_pos_w = data.site(driven_site_name).xpos
            linkage_vec = driven_site_pos_w - driving_site_pos_w # Vector from driving to driven site
            
            norm_linkage_vec = np.linalg.norm(linkage_vec)
            if norm_linkage_vec > 1e-9: # Add epsilon for numerical stability
                linkage_dirs_list.append(linkage_vec / norm_linkage_vec)
            else:
                linkage_dirs_list.append(np.zeros(3)) # Avoid division by zero if sites coincide

        # Create a diagonal matrix from concatenated linkage direction vectors
        # linkage_dir_flat has shape (3 * num_ol_sites,)
        linkage_dir_flat = np.concatenate(linkage_dirs_list)
        # linkage_dir_diag_matrix has shape (3*num_ol_sites, 3*num_ol_sites)
        linkage_dir_diag_matrix = np.diag(linkage_dir_flat)
        
        # Define the matrix M that maps intermediate forces to output rotary joint efforts:
        # or_eff_des = M @ des_frcs_flat
        # M = stacked_il_J_or.T @ linkage_dir_diag_matrix
        # M has shape (num_or_joints, 3 * num_ol_sites)
        M = stacked_il_J_or.T @ linkage_dir_diag_matrix
        
        # Solve for des_frcs_flat (intermediate forces).
        # M is typically wide (e.g., 2x6), so the system is underdetermined.
        # We seek the minimum norm solution for des_frcs_flat.
        # This can be found via des_frcs_flat = M.T @ inv(M @ M.T) @ or_eff_des
        MMT = M @ M.T # Shape (num_or_joints, num_or_joints), e.g., (2,2)
        
        try:
            # Use np.linalg.solve for MMT @ w = or_eff_des
            w = np.linalg.solve(MMT, or_eff_des)
        except np.linalg.LinAlgError:
            # Fallback to pseudo-inverse if MMT is singular or ill-conditioned
            # print("Warning: M@M.T may be singular or ill-conditioned. Using pseudo-inverse.")
            w = np.linalg.pinv(MMT) @ or_eff_des
        des_frcs_flat = M.T @ w # Shape (3 * num_ol_sites,)

        # print(f"M.shape={M.shape}")
        # des_frcs_flat = np.linalg.pinv(M) @ or_eff_des

        # The original code then calculates forces to be applied:
        # des_frcs = np.split(linkage_dir_diag @ des_frcs_mat, len(il_site_names))
        # This means des_frcs_flat (original des_frcs_mat) are further transformed by linkage_dir_diag_matrix.
        des_frcs_list_scaled = np.split(linkage_dir_diag_matrix @ des_frcs_flat, len(il_site_names))

        # Precompute dofadrs and actuator IDs for input rotary (ir) joints.
        # For strict adherence to "rewrite selection only", these are computed here.
        # For better performance, these model-dependent values should be computed once outside ctrl_loop.
        current_ir_jnt_dofadrs = np.array([model.joint(n).dofadr[0] for n in ir_jnt_names], dtype=np.int32)
        current_ir_actuator_ids = np.array([model.actuator(n).id for n in ir_jnt_names], dtype=np.int32)

        for i in range(len(ir_jnt_names)):
            # des_frc_scaled is the i-th Cartesian force vector from des_frcs_list_scaled
            des_frc_scaled = des_frcs_list_scaled[i] # Shape (3,)
            driving_site_name = il_site_names[i]
            
            # Jacobian for the i-th driving site (il_site_names) w.r.t. the i-th input rotary joint's DOF
            J_driving_site_full = get_site_jacobian(model, data, driving_site_name) # Shape (6, model.nv)
            # Translational part of Jacobian for this site, for the specific input DOF. Shape (3,).
            # This is d(driving_site_pos) / d(q_input_joint_i)
            ol_J_ir_vec = J_driving_site_full[:3, current_ir_jnt_dofadrs[i]]
            
            # Map Cartesian force (des_frc_scaled) to input joint effort: effort = J^T @ Force
            input_joint_effort_des = ol_J_ir_vec.T @ des_frc_scaled # Dot product, scalar result
            
            data.ctrl[current_ir_actuator_ids[i]] = input_joint_effort_des # Apply effort to actuator

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
        # default="reduced",
        default="full",
        help="Name of a model directory under models/",
    )
    args = parser.parse_args()

    main(args)