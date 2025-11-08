import numpy as np
from scipy.optimize import minimize

def InverseKinematic_Opt(P, theta0=None):
    """
    InverseKinematic_Opt computes actuator angles of a Delta robot
    by solving the inverse kinematics as an optimization problem.

    Parameters
    ----------
    P : array_like (3,)
        End-effector position [x, y, z] (mm)
    theta0 : array_like (3,), optional
        Initial guess for actuator angles [rad]

    Returns
    -------
    theta : ndarray (3,)
        Optimized actuator angles [rad]
    """

    # --- Extract geometry
    L = np.array([0.2294, 0.1082, 0.287780011578225, 0.431670017367337]) * 1000
    alpha = np.deg2rad([0, 120, 240])
    Rb, Rp, L1, L2 = L
    alpha = np.array(alpha).flatten()
    P = np.array(P).flatten()

    # --- Initial guess
    if theta0 is None:
        theta0 = np.deg2rad([20, 20, 20])
    else:
        theta0 = np.deg2rad(np.array(theta0)).flatten()

    # --- Objective function (sum of squared leg residuals)
    def objective(th):
        total = 0.0
        for i in range(3):
            xi = (P[0] + Rp*np.cos(alpha[i]) - L1*np.cos(th[i])*np.cos(alpha[i]) - Rb*np.cos(alpha[i]))
            yi = (P[1] + Rp*np.sin(alpha[i]) - L1*np.cos(th[i])*np.sin(alpha[i]) - Rb*np.sin(alpha[i]))
            zi = (P[2] + L1*np.sin(th[i]))
            residual = (xi**2 + yi**2 + zi**2 - L2**2)**2
            total += residual
        return total

    # --- Optimization (Quasi-Newton BFGS)
    res = minimize(
        objective,
        theta0,
        method='BFGS',
        options={'disp': False, 'maxiter': 100000, 'gtol': 1e-9}
    )

    return np.rad2deg(res.x)
