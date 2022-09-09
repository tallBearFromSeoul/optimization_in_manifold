import numpy as np
from error import *
from utility import *

### Rodrigues formula :
# R = I_3 + sin(theta)*K + (1-cos(theta))*K^2
# where the skew_symmetric and symmetric decomposition of R is :
# R = (R - R.T)/2 + (R + R.T)/2
# any square matrix can be written uniquely as a skew-symmetric matrix
# sin(theta)*K  : (R-R.T)/2 -> skew_symmetric
# I_3 + (1-cos(theta))*K^2 : (R+R.T)/2 -> symmetric

# axis-angle representation k
# to rotation matrix R
def exp_map_from_so3_to_SO3(k: np.ndarray):
    if not isinstance(k, np.ndarray):
        raise NotNumpyArray
    if k.ndim != 1:
        raise NotOneDimArray
    if k.shape[0] != 3:
        raise Not3Vector
    K = skew_symmetric(k)
    theta = np.linalg.norm(k)
    if theta == 0:
        raise ZeroDivisionError
    R = np.eye(3) + (np.sin(theta)/theta) * K + (1 - np.cos(theta))/(theta**2) * K@K
    return R

# rotation matrix R
# to axis-angle representation k
def log_map_from_SO3_to_so3(R: np.ndarray):
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
    theta = np.arccos((tr - 1) / 2)
    #print('theta :\n',theta)
    R_log = (theta / (2*np.sin(theta))) * (R - R.T)
    # axis of rotation vector k
    k = vee_operator(R_log)
    return k

# axis-angle representation k
# to quaternion q : [qr, qv].T
def exp_map_from_so3_to_SU2(k: np.ndarray):
    if not isinstance(k, np.ndarray): 
        raise NotNumpyArray
    if k.ndim != 1:
        raise NotOneDimArray
    if k.shape[0] != 3:
        raise Not3Vector
    theta = np.linalg.norm(k)
    theta_2 = theta/2
    s_2 = np.sin(theta_2)
    if theta == 0:
        raise ZeroDivisionError
    if (np.allclose(k.sum(),0)):
        return np.array([1,0,0,0])
    else:
        return np.array([np.cos(theta_2), s_2*k[0]/theta, s_2*k[1]/theta, s_2*k[2]/theta])

# quaternion q : [qr, qv].T
# to axis-angle representation k
def log_map_from_SU2_to_so3(q: np.ndarray):
    if not isinstance(q, np.ndarray):
        raise NotNumpyArray
    if q.ndim != 1:
        raise NotOneDimArray
    if q.shape[0] != 4:
        raise Not4Vector
    qr = q[0]
    qv = q[1:]
    if (np.allclose(qv.sum(),0)):
        raise ZeroDivisionError
    k = 2*np.arccos(qr)*qv/np.linalg.norm(qv)
    return k

