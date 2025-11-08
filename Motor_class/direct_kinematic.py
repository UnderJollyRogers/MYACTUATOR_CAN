import numpy as np
from scipy.optimize import minimize

def DirectKinematic(theta, P0=None):
    """
    DirectKinematic computes the Cartesian position of the end-effector
    of a Delta robot given the actuator angles (theta1, theta2, theta3)
    using a numerical optimization approach.

    Parameters
    ----------
    theta : array_like (3,)
        Actuator joint angles [rad]
    P0 : array_like (3,), optional
        Initial guess for [x, y, z] (mm)

    Returns
    -------
    P : ndarray (3,)
        End-effector Cartesian position [x, y, z] (mm)
    """

    L = np.array([0.2294, 0.1082, 0.287780011578225, 0.431670017367337]) * 1000
    alpha = np.deg2rad([0, 120, 240])
    Rb, Rp, L1, L2 = L
    alpha = np.array(alpha).flatten()
    theta = np.array(np.deg2rad(theta)).flatten()

    # --- Initial guess for position
    if P0 is None:
        P0 = np.array([0.0, 0.0, -400.0])
    else:
        P0 = np.array(P0).flatten()

    # --- Objective function: sum of squared leg errors
    def objective(p):
        total = 0.0
        for i in range(3):
            xi = (p[0] + Rp*np.cos(alpha[i])
                  - L1*np.cos(theta[i])*np.cos(alpha[i])
                  - Rb*np.cos(alpha[i]))
            yi = (p[1] + Rp*np.sin(alpha[i])
                  - L1*np.cos(theta[i])*np.sin(alpha[i])
                  - Rb*np.sin(alpha[i]))
            zi = (p[2] + L1*np.sin(theta[i]))
            residual = (xi**2 + yi**2 + zi**2 - L2**2)**2
            total += residual
        return total

    # --- Optimization settings (silent)
    res = minimize(
        objective,
        P0,
        method='BFGS',  # quasi-Newton, same as MATLAB fminunc
        options={
            'disp': False,
            'gtol': 1e-9,
            'maxiter': 100000
        }
    )

    return res.x
