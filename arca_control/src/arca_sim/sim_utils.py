import mujoco
import numpy as np

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