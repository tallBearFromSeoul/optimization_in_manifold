import numpy as np
from utility import *

# euler_angles in rad : [roll, pitch, yaw].T
# to rotation matrix R 
def R_from_euler_angles_rad(euler_angles_rad: np.ndarray):
    if not isinstance(euler_angles_rad, np.ndarray):
        raise NotNumpyArray
    if euler_angles_rad.ndim != 1:
        raise NotOneDimArray
    if euler_angles_rad.shape[0] != 3:
        raise Not3Vector
    roll = euler_angles_rad[0]
    pitch = euler_angles_rad[1]
    yaw = euler_angles_rad[2]
    R_x = np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
    R_y = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    return R_z @ R_y @ R_x

# rotation matrix R
# to euler_angles in rad : [roll, pitch, yaw].T
def euler_angles_rad_from_R(R: np.ndarray):
    if not isinstance(R, np.ndarray):
        raise NotNumpyArray
    if R.shape[0] != R.shape[1]:
        raise NotSquareMatrix
    if R.shape[0] != 3:
        raise Not3by3Matrix
    roll = np.arctan2(R[2,1], R[2,2])
    pitch = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
    yaw = np.arctan2(R[1,0], R[0,0])
    return np.array([roll, pitch, yaw])

# euler_angles in rad : [roll, pitch, yaw].T
# to quaternion q : [qr, qv].T
def q_from_euler_angles_rad(euler_angles_rad: np.ndarray):
    if not isinstance(euler_angles_rad, np.ndarray):
        raise NotNumpyArray
    if euler_angles_rad.ndim != 1:
        raise NotOneDimArray
    if euler_angles_rad.shape[0] != 3:
        raise Not3Vector
    roll_2 = euler_angles_rad[0]/2
    pitch_2 = euler_angles_rad[1]/2
    yaw_2 = euler_angles_rad[2]/2
    c_x = np.cos(roll_2)
    c_y = np.cos(pitch_2)
    c_z = np.cos(yaw_2)
    s_x = np.sin(roll_2)
    s_y = np.sin(pitch_2)
    s_z = np.sin(yaw_2)
    q = np.array(
        [c_x*c_y*c_z + s_x*s_y*s_z,
        s_x*c_y*c_z - c_x*s_y*s_z,
        c_x*s_y*c_z + s_x*c_y*s_z,
        c_x*c_y*s_z - s_x*s_y*c_z])
    return q

# quaternion q : [qr, qv].T
# to euler_angles in rad : [roll, pitch, yaw].T
def euler_angles_rad_from_q(q: np.ndarray):
    if not isinstance(q, np.ndarray):
        raise notnumpyarray
    if q.ndim != 1: 
        raise notonedimarray
    if q.shape[0] != 4:
        raise not4vector
    roll = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1-2*(q[1]**2 + q[2]**2))
    pitch = np.arcsin(2*(q[0]*q[2] - q[3]*q[1]))
    yaw = np.arctan2(2*(q[0]*q[3] + q[1]*q[2]), 1-2*(q[2]**2 + q[3]**2))
    return np.array([roll, pitch, yaw])    

#http://www.songho.ca/opengl/gl_quaternion.html
def R_from_q(q: np.ndarray):
    if not isinstance(q, np.ndarray):
        raise notnumpyarray
    if q.ndim != 1: 
        raise notonedimarray
    if q.shape[0] != 4:
        raise not4vector
    return np.array([
        [1-2*(q[2]**2)-2*(q[3]**2), 2*q[1]*q[2]-2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2]],
        [2*q[1]*q[2]+2*q[0]*q[3], 1-2*(q[1]**2)-2*(q[3]**2), 2*q[2]*q[3]-2*q[0]*q[1]],
        [2*q[1]*q[3]-2*q[0]*q[2], 2*q[2]*q[3]+2*q[0]*q[1], 1-2*(q[1]**2)-2*(q[2]**2)]])

def main():
    e_deg = np.array([125,100,40])
    #e_deg = np.array([80,20,10])
    e_rad = deg2rad(e_deg)
    q = q_from_euler_angles_rad(e_rad)
    e_test = euler_angles_rad_from_q(q)
    print('e_rad',e_rad)
    print('q',q)
    print('e_test',e_test)

if __name__=='__main__':
    main()

