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

def quat_to_rpy(quat):
    return np.array([
        np.arctan2(2*(quat[3]*quat[0] + quat[1]*quat[2]), 1 - 2*(quat[0]**2 + quat[1]**2)),
        np.arcsin(2*(quat[3]*quat[1] - quat[2]*quat[0])),
        np.arctan2(2*(quat[3]*quat[2] + quat[0]*quat[1]), 1 - 2*(quat[1]**2 + quat[2]**2))
    ])

def sin_traj(t, f, min, max):
    sin = np.interp(np.sin(2*np.pi*f*t), [-1, 1], [min, max])
    dsin = np.interp(2*np.pi*f*np.cos(2*np.pi*f*t), [-1, 1], [-2*np.pi*f*min, 2*np.pi*f*max])
    return sin, dsin


