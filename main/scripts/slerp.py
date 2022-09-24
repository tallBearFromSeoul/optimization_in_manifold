from exp_and_log_map import *
from metrics import *

# spherical linear interpolation (SLERP)
# R1 and R2 are 3x3 rotation matrices,
# where the function computes the geodesic line
# from R1 to R2 where lamda : [0, 1]
# R(0) = R1 and R(1) = R2
# the function returns the interpolated rotation matrices
# from lamda = 0 to lamda = 1
# based on the number of steps, default is 25
# the resulting motion is in constant angular velocity
# along the surface of the hypersphere
def slerp_R(R1: np.ndarray, R2: np.ndarray, n_steps: int = 25):
    if not isinstance(R1, np.ndarray) or not isinstance(R2, np.ndarray):
        raise NotNumpyArray
    if R1.shape[0] != R1.shape[1] or R2.shape[0] != R2.shape[1]:
        raise NotSquareMatrix
    if R1.shape[0] != 3 or R2.shape[0] != 3:
        raise Not3by3Matrix
    if not isinstance(n_steps, int):
        raise NotInt
    if n_steps < 0:
        raise NotPositive
    Rs_interp = []
    k_log = log_map_from_SO3_to_so3(np.linalg.inv(R1) @ R2)
    for i, lamda in enumerate(np.linspace(0.0, 1.0, n_steps)):
        if i==0:
            continue
        R = exp_map_from_so3_to_SO3(lamda*k_log)
        Rs_interp.append(R1 @ R)
    return Rs_interp

def slerp_q_direct(q1: np.ndarray, q2: np.ndarray, n_steps: int = 25):
    if not isinstance(q1, np.ndarray) or not isinstance(q2, np.ndarray):
        raise NotNumpyArray
    if q1.ndim != 1 or q2.ndim != 1:
        raise NotOneDimArray
    if q1.shape[0] != 4 or q2.shape[0] != 4:
        raise Not4Vector
    if not isinstance(n_steps, int):
        raise NotInt
    if n_steps < 0:
        raise NotPositive
    qs_interp = []
    q1_inv = quaternion_inverse(q1)
    q1_inv_q2 = quaternion_mult(q1_inv, q2)
    for i, lamda in enumerate(np.linspace(0.0, 1.0, n_steps)):
        if i==0:
            continue
        q1_inv_q2_pow = quaternion_power(q1_inv_q2, lamda)
        q = quaternion_mult(q1, q1_inv_q2_pow)
        qs_interp.append(q)
    return qs_interp
 
def slerp_q_exp_and_log(q1: np.ndarray, q2: np.ndarray, n_steps: int = 25):
    if not isinstance(q1, np.ndarray) or not isinstance(q2, np.ndarray):
        raise NotNumpyArray
    if q1.ndim != 1 or q2.ndim != 1:
        raise NotOneDimArray
    if q1.shape[0] != 4 or q2.shape[0] != 4:
        raise Not4Vector
    if not isinstance(n_steps, int):
        raise NotInt
    if n_steps < 0:
        raise NotPositive
    qs_interp = []
    q1_inv = quaternion_inverse(q1)
    q1_inv_q2 = quaternion_mult(q1_inv, q2)
    k_log = log_map_from_SU2_to_so3(q1_inv_q2)
    for i, lamda in enumerate(np.linspace(0.0, 1.0, n_steps)):
        if i==0:
            continue
        q_exp = exp_map_from_so3_to_SU2(lamda*k_log)
        q = quaternion_mult(q1, q_exp)
        qs_interp.append(q)
    return qs_interp
               
