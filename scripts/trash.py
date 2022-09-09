
def visualize_qs(qs_interp: list, draw_3d: bool):
    f, ax = plt.subplots(1,4, figsize=(20,6))
    f.suptitle(f'Spherical Linear Interpolation from q1 to q2 into {len(qs_interp)} points in Axis-Angle Representation')
    idxs = [*range(len(qs_interp))]
    #ks_norm1, ks_norm2, ks_x1, ks_y1, ks_z1, ks_x2,ks_y2, ks_z2 = [], [], [], [], [], [], [], [] 
    thetas, ks_x, ks_y, ks_z = [], [], [], []
    ks_norm, ks_norm_x, ks_norm_y, ks_norm_z = [], [], [], []
    #thetas_1, thetas_2 = [], []
    #qs_r1, qs_x1, qs_y1, qs_z1 = [], [], [], []
    #qs_r2, qs_x2, qs_y2, qs_z2 = [], [], [], []
    #for q1, q2 in zip(qs_interp, qs_interp_alter):
    for q_interp in qs_interp:
        k = log_map_from_SU2_to_so3(q_interp)
        k_norm = np.linalg.norm(k)
        thetas.append(rad2deg(k_norm))
        ks_x.append(k[0])
        ks_y.append(k[1])
        ks_z.append(k[2])
        ks_norm.append(k_norm)
        ks_norm_x.append(k[0]/k_norm)
        ks_norm_y.append(k[1]/k_norm)
        ks_norm_z.append(k[2]/k_norm)
        '''
        k1 = log_map_from_SU2_to_so3(q1)
        k2 = log_map_from_SU2_to_so3(q2)
        k1_norm = np.linalg.norm(k1)
        k2_norm = np.linalg.norm(k2)
        ks_norm1.append(k1_norm)
        ks_norm2.append(k2_norm)
        ks_x1.append(k1[0])
        ks_y1.append(k1[1])
        ks_z1.append(k1[2])
        ks_x2.append(k2[0])
        ks_y2.append(k2[1])
        ks_z2.append(k2[2])
        thetas_1.append(rad2deg(np.linalg.norm(k1)))
        thetas_2.append(rad2deg(np.linalg.norm(k2)))
        qs_r1.append(q1[0])
        qs_x1.append(q1[1])
        qs_y1.append(q1[2])
        qs_z1.append(q1[3])
        qs_r2.append(q2[0])
        qs_x2.append(q2[1])
        qs_y2.append(q2[2])
        qs_z2.append(q2[3])
        '''
    ax[0].scatter(idxs, thetas, alpha=0.5, s=8, marker='o', c='r')
    #ax[0].scatter(idxs, thetas_2, alpha=0.5, s=8, marker='+', c='r')
    #ax[0].legend(('q1', 'q0'))
    ax[0].set_title('axis angle in degrees of quaternion')
   
    ax[1].scatter(idxs, ks_norm, alpha=0.5, s=8, marker='o', c='orange')
    ax[1].scatter(idxs, ks_x, alpha=0.5, s=8, marker='o', c='r')
    ax[1].scatter(idxs, ks_y, alpha=0.5, s=8, marker='o', c='g')
    ax[1].scatter(idxs, ks_z, alpha=0.5, s=8, marker='o', c='b')
    '''
    ax[1].scatter(idxs, ks_norm2, alpha=0.5, s=8, marker='+', c='orange')
    ax[1].scatter(idxs, ks_x2, alpha=0.5, s=8, marker='+', c='r')
    ax[1].scatter(idxs, ks_y2, alpha=0.5, s=8, marker='+', c='g')
    ax[1].scatter(idxs, ks_z2, alpha=0.5, s=8, marker='+', c='b')
    '''
    ax[1].legend(('Frobenius norm of axis vector of quaternion\n equals to angle in radians', 'component in x axis', 'component in y axis', 'component in z axis'), fontsize=10)
    ax[1].set_title('axis vector of quaternion') 
    '''
    ax[2].scatter(idxs, qs_r1, alpha=0.5, s=8, marker='o', c='r')
    ax[2].scatter(idxs, qs_r2, alpha=0.5, s=8, marker='+', c='b')
    ax[2].scatter(idxs, qs_x1, alpha=0.5, s=8, marker='o', c='r')
    ax[2].scatter(idxs, qs_y1, alpha=0.5, s=8, marker='o', c='g')
    ax[2].scatter(idxs, qs_z1, alpha=0.5, s=8, marker='o', c='b')
    ax[2].scatter(idxs, qs_x2, alpha=0.5, s=8, marker='+', c='r')
    ax[2].scatter(idxs, qs_y2, alpha=0.5, s=8, marker='+', c='g')
    ax[2].scatter(idxs, qs_z2, alpha=0.5, s=8, marker='+', c='b')
    ax[2].legend(('q1_r','q2_r','q1_x','q1_y','q1_z','q2_x','q2_y','q2_z'))
    ax[2].set_title('quaternion components')
    '''

def rodrigues_k_vec(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    if not isinstance(a, np.ndarray) or not isinstance(b, np.ndarray):
        raise NotNumpyArray
    if a.ndim != 1 or b.ndim != 1:
        raise NotOneDimArray
    if a.shape[0] != 3 or b.shape[0] != 3:
        raise Not3Vector
    cross = cross_product(a, b)
    l1_norm = cross.sum()
    return cross / l1_norm

def rodrigues_rotation_mat(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    k_vec = rodrigues_k_vec(a, b)
    K = skew_symmetric(k_vec)
    return np.eye(3) + np.sin(theta)*K + (1-cos(theta))*K@K

def q_from_so3_to_SU2(w: np.ndarray, w_des: np.ndarray) -> np.ndarray:
    if not isinstance(w, np.ndarray) and not isinstance(w_des, np.ndarray):
        raise NotNumpyArray
    if w.ndim != 1 and not w_des.ndim != 1:
        raise NotOneDimArray
    cp = cross_product(w, w_des)
    theta = np.arccos(w.dot(w_des) / np.linalg.norm(w) * np.linalg.norm(w_des))
    if (np.allclose(w.sum(),0)):
        return np.array([1,0,0,0])
    else:
        return np.array([np.cos(theta/2), np.sin(theta/2)*w[0]/theta, np.sin(theta/2)*w[1]/theta, np.sin(theta/2)*w[2]/theta])

def skew_symmetric_inplace(v: np.ndarray, out: np.ndarray):
    if not isinstance(v, np.ndarray):
        raise NotNumpyArray
    if v.ndim != 1:
        raise NotOneDimArray
    if not isinstance(out, np.ndarray):
        raise NotNumpyArray
    if out.shape != (3,3):
        raise Not3by3Matrix
    out[0,1] = -v[2]
    out[0,2] = v[1]
    out[1,0] = v[2]
    out[1,2] = -v[0]
    out[2,0] = -v[1]
    out[2,1] = v[0]


# there are four vectors present : 
# w : the vector to rotate by angle theta about the axis of rotation k
# w_des : the desired final vector after the rotation
# k : axis of rotation
# theta : angle of rotation
def R_from_so3_to_SO3(w: np.ndarray, w_des: np.ndarray) -> np.ndarray:
    if not isinstance(w, np.ndarray) and not isinstance(w_des, np.ndarray):
        raise NotNumpyArray
    if w.ndim != 1 and not w_des.ndim != 1:
        raise NotOneDimArray
    cp = cross_product(w, w_des)
    # axis of rotation vector k
    k = cp / np.linalg.norm(cp)
    # angle of rotation theta (rad)
    theta = np.arccos(w.dot(w_des) / (np.linalg.norm(w) * np.linalg.norm(w_des)))
    K = skew_symmetric(k) 
    # R = I_3 + sin(theta)*K + (1-cos(theta))*K^2
    # where the skew_symmetric and symmetric decomposition of R is :
    # R = (R - R.T)/2 + (R + R.T)/2
    # any square matrix can be written uniquely as a skew-symmetric matrix
    # sin(theta)*K  : (R-R.T)/2 -> skew_symmetric
    # I_3 + (1-cos(theta))*K^2 : (R+R.T)/2 -> symmetric
    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K@K
    return k, theta, R


### exp_and_log_map.py
# main() to test and validate the exponential map and logarithm map of rotations
def main():
    w_s = [np.ones(3)*np.sqrt(1/3), np.array([np.sqrt(1/2), 0, np.sqrt(1/2)]), np.array([0, np.sqrt(1/2), np.sqrt(1/2)]), np.array([0,0,1])]
    w_des_s = [np.array([1,0,0]), np.array([1,0,0]), np.array([1,0,0]), np.array([1,0,0])]

    all_test_pass = True
    print_res = False

    for i, (w, w_des) in enumerate(zip(w_s, w_des_s)):
        test_pass = True
        print(f'\nTest {i} :\n')
        print('original vector w :\n',w)
        print('desired vector w_des :\n',w_des)
        
        k, theta, R = R_from_so3_to_SO3(w, w_des)
        if not np.allclose(R@w, w_des):
            test_pass = False
            all_test_pass = False
        if print_res:
            print('rotation matrix R obtained from Rodrigues formula,\n or exponential map of the original vector w to SO3 is :\n', R)
            if test_pass:
                print('---PASS---')
                print('w_new = R @ w == w_des is within tolerance of rtol=1e-05, atol=1e-08,\nwhere\n')
            else:
                print('---WRONG---')
                print('w_new = R @ w == w_des is NOT within tolerance of rtol=1e-05, atol=1e-08,\nwhere\n')
            print('w_new = R @ w is : ',R@w)
            print('w_des is : ',w_des)

        k_log, theta_log, R_log = log_map_from_SO3_to_so3(R)
        test_pass = True
        if not np.allclose(k, k_log):
            test_pass = False
            all_test_pass = False
        if print_res:
            print('logarithm map of the rotation matrix R to so3 :\n',R_log)
            if test_pass:
                print('---PASS---')
                print('the axis of rotation from the exponential map and logarithm map are within tolerance of rtol=1e-05, atol=1e-08,\nwhere\n')
            else:
                print('---WRONG---')
                print('the axis of rotation from the exponential map and logarithm map are NOT within tolerance of rtol=1e-05, atol=1e-08,\nwhere\n')
            print('axis of rotation from exponential map is :\n',k)
            print('axis of rotation from logarithm map is :\n',k_log)

        test_pass = True
        if not np.allclose(theta, theta_log):
            test_pass = False
            all_test_pass = False
        if print_res:
            if test_pass:
                print('---PASS---')
                print('the angle of rotation from the exponential map and logarithm map are within tolerance of rol=1e-05, atol=1e-08,\nwhere\n')
            else:
                print('---WRONG---')
                print('the angle of rotation from the exponential map and logarithm map are NOT within tolerance of rol=1e-05, atol=1e-08,\nwhere\n')
            print('angle of rotation is :\n',theta)
            print('angle of rotation is :\n',theta_log)


    if all_test_pass:
        print('---ALL_PASS---\n')
    '''  
    q = q_from_so3_to_SU2(w, w_des)
    print('quaternion q obtained from Rodrigues formula,\n or exponential map of the original vector w to SU2 is :\n',q)
    '''

    '''
    try:
        skew_symmetric(v, v_skew)
        print(id(v_skew))
    except NotNumpyArray:
        print('use numpy array')
    except NotOneDimArray:
        print('use one dim vector')
    print(v_skew)
    '''


