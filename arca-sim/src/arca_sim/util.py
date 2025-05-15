import numpy as np

def rpy_to_quat(rpy):
    r, p, y = rpy
    cr, cp, cy = np.cos(r/2), np.cos(p/2), np.cos(y/2)
    sr, sp, sy = np.sin(r/2), np.sin(p/2), np.sin(y/2)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    ])

def rpy_from_quat(q):
    # Extract roll, pitch, yaw from quaternion
    # q = [w, x, y, z]
    w, x, y, z = q
    
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)
    
    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.array([roll, pitch, yaw])
