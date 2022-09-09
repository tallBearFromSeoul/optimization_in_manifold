import numpy as np
from utility import *
from conversion import *

# jacobian : 9 x 3 
def jacob_exp_map_from_so3_to_SO3():
    e1 = np.array([1,0,0])
    e2 = np.array([0,1,0])
    e3 = np.array([0,0,1])
    return -np.vstack((hat_operator(e1), hat_operator(e2), hat_operator(e3)))
   
# jacobian : 4 x 3
def jacob_exp_map_from_so3_to_SU2(k: np.ndarray):
    if not isinstance(k, np.ndarray): 
        raise NotNumpyArray
    if k.ndim != 1:
        raise NotOneDimArray
    if k.shape[0] != 3:
        raise Not3Vector
    theta = np.linalg.norm(k)
    theta_2 = theta / 2
    s_2 = np.sin(theta_2)
    c_2 = np.cos(theta_2)
    s_2_theta = s_2 / theta
    const_0 = ((c_2/(2*theta)) - s_2/(theta**2))
    d_exp_kq__d_wtheta = np.array([
        [0, 0, 0, -s_2/2],
        [s_2_theta, 0, 0, k[0]*const_0],
        [0, s_2_theta, 0, k[1]*const_0],
        [0, 0, s_2_theta, k[2]*const_0]])
    k_norm = k / theta
    d_wtheta__d_w = np.vstack((np.eye(3), k_norm))
    print(d_wtheta__d_w)
    return d_exp_kq__d_wtheta @ d_wtheta__d_w

# jacobian : 3 x 9
def jacob_log_map_from_SO3_to_so3(R: np.ndarray):
    if not isinstance(R, np.ndarray):
        raise NotNumpyArray
    if R.ndim != 2:
        raise NotTwoDimArray
    if R.shape[0] != R.shape[1]:
        raise NotSquareMatrix
    if R.shape[0] != 3:
        raise Not3by3Matrix
    tr = trace(R)
    # angle of rotation theta (rad)
    cos_theta = (tr - 1) / 2
    if cos_theta > 0.999999:
        return np.array([
            [0,0,0, 0,0,0.5, 0,-0.5,0],
            [0,0,-0.5, 0,0,0, 0.5,0,0],
            [0,0.5,0, -0.5,0,0, 0,0,0]])

    theta = np.arccos(cos_theta)
    sin_theta = np.sqrt(1-cos_theta**2)
    # (R-R.T)vee
    R_minus_RT_vee = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
    a = R_minus_RT_vee * (theta*cos_theta - sin_theta) / (4*(sin_theta**3))
    a1 = a[0]
    a2 = a[1]
    a3 = a[2]
    b = theta / (2*sin_theta)
    print(f'a :\n{a}')
    print(f'a1 : {a1}')
    print(f'a2 : {a2}')
    print(f'a3 : {a3}')
    print(f'b : {b}')
    return np.array([
        [a1,0,0, 0,a1,b, 0,-b,a1],
        [a2,0,-b, 0,a2,0, b,0,a2],
        [a3,b,0, -b,a3,0, 0,0,a3]])

def main():
    k = np.array([1,1,1])
    print(k)
    euler_angles_deg = np.array([45,30,20])
    euler_angles_rad = deg2rad(euler_angles_deg)
    R = R_from_euler_angles_rad(euler_angles_rad)

    print(jacob_exp_map_from_so3_to_SO3())
    print(jacob_exp_map_from_so3_to_SU2(k))
    print(jacob_log_map_from_SO3_to_so3(R))

if __name__=='__main__':
    main()
