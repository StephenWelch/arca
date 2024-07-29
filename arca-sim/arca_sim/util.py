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
