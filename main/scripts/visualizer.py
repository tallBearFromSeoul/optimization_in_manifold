from copy import deepcopy
from slerp import *
from conversion import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def draw_sphere(n_pts: int = 50):
    xs, ys, zs = [], [], []
    for i in range(n_pts):
        for j in range(n_pts):
            xs.append(np.sin(np.pi*i/n_pts)*np.cos(2*np.pi*j/n_pts))
            ys.append(np.sin(np.pi*i/n_pts)*np.sin(2*np.pi*j/n_pts))
            zs.append(np.cos(np.pi*i/n_pts))
    return xs, ys, zs


def deriv_list(l: list):
    res = []
    x_prev = None
    for x in l:
        if x_prev != None:
            res.append(x-x_prev)
        x_prev = x
    return res

def visualize(init_conditions: tuple, Rs_interp: list, qs_interp: list, draw_3d: bool, fig_idx: int):
    ### MPL Formatting
    mpl.rcParams['text.usetex'] = True
    mpl.rcParams['text.latex.preamble'] = r'\usepackage{{amsmath}}'
    mpl.rcParams['legend.fontsize'] = 10
    mpl.rcParams['axes.titlepad'] = -10
    #mpl.rcParams['lines.linewidth'] = 1
    #mpl.rcParams['lines.linestyle'] = 'dashdot'
    
    euler_angles_deg1 = init_conditions[0]
    euler_angles_deg2 = init_conditions[1]
    R1 = init_conditions[2]
    R2 = init_conditions[3]
    q1 = init_conditions[4]
    q2 = init_conditions[5]
    v_init = init_conditions[6]
    path_title = r'path of vector $v_{init} =  [%.3f, %.3f, %.3f]^T$'%(v_init[0], v_init[1], v_init[2])+'\nfrom series of rotations on unit sphere\nusing rotation matrices and quaternions'
    axis_ang_title = 'axis angle vector of the rotations\nfrom slerp using rotations matrices and quaternions'

    f, axs = plt.subplots(3,4,figsize=(20,9))
    plt.subplots_adjust(left=0.03,bottom=0.15,right=0.97,top=0.7,wspace=0.5,hspace=0.4)
    suptitle = f'Comparison of Spherical Linear Interpolation from A to B between using rotation matrices and quaternions to {len(Rs_interp)} points in euler angles and axis-angle representation, '+'where\n' 
    suptitle += r'$A : euler\ angles_{deg} = [%.1f, %.1f, %.1f]^T$'%(euler_angles_deg1[0], euler_angles_deg1[1], euler_angles_deg1[2])+'\n'
    suptitle += r'$B : euler\ angles_{deg} = [%.1f, %.1f, %.1f]^T$'%(euler_angles_deg2[0], euler_angles_deg2[1], euler_angles_deg2[2])+'\n'
    suptitle += r'$R_A = \begin{bmatrix} %.3f & %.3f & %.3f \\ %.3f & %.3f & %.3f \\ %.3f & %.3f & %.3f \end{bmatrix}$'%(R1[0,0],R1[0,1],R1[0,2],R1[1,0],R1[1,1],R1[1,2],R1[2,0],R1[2,1],R1[2,2])
    suptitle += r'$R_B = \begin{bmatrix} %.3f & %.3f & %.3f \\ %.3f & %.3f & %.3f \\ %.3f & %.3f & %.3f \end{bmatrix}$'%(R1[0,0],R1[0,1],R1[0,2],R1[1,0],R1[1,1],R1[1,2],R1[2,0],R1[2,1],R1[2,2])
    suptitle += r'$q_A = [%.3f, %.3f, %.3f, %.3f]^T$'%(q1[0], q1[1], q1[2], q1[3])
    suptitle += r'$q_B = [%.3f, %.3f, %.3f, %.3f]^T$'%(q2[0], q2[1], q2[2], q2[3])+'\n'
    f.suptitle(suptitle)

    ### Data Extraction
    idxs, jdxs = [*range(len(Rs_interp))], [*range(len(Rs_interp)-1)]
    idxs_paths = [*range(len(Rs_interp))]
    R_path, q_path = deepcopy(v_init), deepcopy(v_init)#np.array([1,0,0])
    R_paths_x, R_paths_y, R_paths_z = [],[],[]#R_path[0]], [R_path[1]], [R_path[2]]
    q_paths_x, q_paths_y, q_paths_z = [],[],[]#[q_path[0]], [q_path[1]], [q_path[2]]
    qs_w, qs_x, qs_y, qs_z = [], [], [], []
    R_rolls, R_pitchs, R_yaws = [], [], []
    q_rolls, q_pitchs, q_yaws = [], [], []
    R_thetas, R_ks_x, R_ks_y, R_ks_z = [], [], [], []
    q_thetas, q_ks_x, q_ks_y, q_ks_z = [], [], [], []
    R_ks_norm, R_ks_norm_x, R_ks_norm_y, R_ks_norm_z = [], [], [], []
    q_ks_norm, q_ks_norm_x, q_ks_norm_y, q_ks_norm_z = [], [], [], []

    #geodesic_metrics, hyperbolic_metrics, frobenius_metrics = compute_metrics(Rs_interp[0], Rs_interp)
    for R_interp in Rs_interp:
        R_paths_x.append(R_path[0])
        R_paths_y.append(R_path[1])
        R_paths_z.append(R_path[2])
        R_path = R_interp @ R_path

        euler_angles_rad = euler_angles_rad_from_R(R_interp)
        R_rolls.append(rad2deg(euler_angles_rad[0]))
        R_pitchs.append(rad2deg(euler_angles_rad[1]))
        R_yaws.append(rad2deg(euler_angles_rad[2]))

        k = log_map_from_SO3_to_so3(R_interp)
        theta = np.linalg.norm(k)
        R_thetas.append(rad2deg(theta))
        R_ks_x.append(k[0])
        R_ks_y.append(k[1])
        R_ks_z.append(k[2])
        k_norm = np.linalg.norm(k)
        R_ks_norm.append(k_norm)
        R_ks_norm_x.append(k[0]/k_norm)
        R_ks_norm_y.append(k[1]/k_norm)
        R_ks_norm_z.append(k[2]/k_norm)
    for q_interp in qs_interp:
        qs_w.append(q_interp[0])
        qs_x.append(q_interp[1])
        qs_y.append(q_interp[2])
        qs_z.append(q_interp[3])
        q_interp_R = R_from_q(q_interp)
        q_paths_x.append(q_path[0])
        q_paths_y.append(q_path[1])
        q_paths_z.append(q_path[2])
        q_path = q_interp_R @ q_path

        euler_angles_rad = euler_angles_rad_from_q(q_interp)
        q_rolls.append(rad2deg(euler_angles_rad[0]))
        q_pitchs.append(rad2deg(euler_angles_rad[1]))
        q_yaws.append(rad2deg(euler_angles_rad[2]))

        k = log_map_from_SU2_to_so3(q_interp)
        theta = np.linalg.norm(k)
        q_thetas.append(rad2deg(theta))
        q_ks_x.append(k[0])
        q_ks_y.append(k[1])
        q_ks_z.append(k[2])
        q_ks_norm.append(k_norm)
        q_ks_norm_x.append(k[0]/k_norm)
        q_ks_norm_y.append(k[1]/k_norm)
        q_ks_norm_z.append(k[2]/k_norm)

    axs[0,0].scatter(idxs, R_rolls, c='r', marker='o', s=30, alpha=0.5)
    axs[0,0].scatter(idxs, R_pitchs, c='g', marker='o', s=30, alpha=0.5)
    axs[0,0].scatter(idxs, R_yaws, c='b', marker='o', s=30, alpha=0.5)
    axs[0,0].scatter(idxs, q_rolls, c='m', marker='^', s=20, alpha=0.5)
    axs[0,0].scatter(idxs, q_pitchs, c='lime', marker='^', s=20, alpha=0.5)
    axs[0,0].scatter(idxs, q_yaws, c='c', marker='^', s=20, alpha=0.5)
    axs[0,0].legend(('R_roll','R_pitch','R_yaw','q_roll','q_pitch','q_yaw'))
    axs[0,0].set_title('euler angles in degrees')

    axs[0,1].scatter(jdxs, deriv_list(R_rolls), c='r', marker='o', s=30, alpha=0.5)
    axs[0,1].scatter(jdxs, deriv_list(R_pitchs), c='g', marker='o', s=30, alpha=0.5)
    axs[0,1].scatter(jdxs, deriv_list(R_yaws), c='b', marker='o', s=30, alpha=0.5)
    axs[0,1].scatter(jdxs, deriv_list(q_rolls), c='m', marker='^', s=20, alpha=0.5)
    axs[0,1].scatter(jdxs, deriv_list(q_pitchs), c='lime', marker='^', s=20, alpha=0.5)
    axs[0,1].scatter(jdxs, deriv_list(q_yaws), c='c', marker='^', s=20, alpha=0.5)
    axs[0,1].legend(('d[0,R_roll]','d[0,R_pitch]','d[0,R_yaw]','d[0,q_roll]','d[0,q_pitch]','d[0,q_yaw]'))
    axs[0,1].set_title('euler angles in degrees')


    axs[0,2].scatter(idxs, R_thetas, c='r', marker='o', s=30, alpha=0.5)
    axs[0,2].scatter(idxs, q_thetas, c='m', marker='^', s=20, alpha=0.5)
    axs[0,2].legend(('R_theta', 'q_theta'))
    axs[0,2].set_title('axis angle in degrees')
    
    axs[0,3].scatter(idxs, R_ks_norm, c='orange', marker='o', s=30, alpha=0.5)
    axs[0,3].scatter(idxs, R_ks_x, c='r', marker='o', s=30, alpha=0.5)
    axs[0,3].scatter(idxs, R_ks_y, c='g', marker='o', s=30, alpha=0.5)
    axs[0,3].scatter(idxs, R_ks_z, c='b', marker='o', s=30, alpha=0.5)
    axs[0,3].legend(('norm of axis vector','R_k_x','R_k_y','R_k_z'))
    axs[0,3].set_title('axis vector')
    
    for ax in axs[1, :]:
        ax.remove()
    gs = axs[1, 2].get_gridspec()
    ax_b = f.add_subplot(gs[1, :4])
    ax_b.scatter(idxs, qs_w, c='black', marker='o', s=30, alpha=0.5)
    ax_b.scatter(idxs, qs_x, c='r', marker='o', s=30, alpha=0.5)
    ax_b.scatter(idxs, qs_y, c='g', marker='o', s=30, alpha=0.5)
    ax_b.scatter(idxs, qs_z, c='b', marker='o', s=30, alpha=0.5)
    ax_b.legend(('q_w','q_x','q_y','q_z'))
    ax_b.set_title(r'quaternion $q = [q_w, q_x, q_y, q_z]^T$')
    
    for ax in axs[2, :]:
        ax.remove()
    gs = axs[1, 2].get_gridspec()
    ax_b_0 = f.add_subplot(gs[2, :2])
    ax_b_0.scatter(idxs_paths, R_paths_x, c='r', marker='o', s=30, alpha=0.5)
    ax_b_0.scatter(idxs_paths, R_paths_y, c='b', marker='o', s=30, alpha=0.5)
    ax_b_0.scatter(idxs_paths, R_paths_z, c='g', marker='o', s=30, alpha=0.5)
    ax_b_0.plot(idxs_paths, R_paths_x, c='r', alpha=0.5)
    ax_b_0.plot(idxs_paths, R_paths_y, c='b', alpha=0.5)
    ax_b_0.plot(idxs_paths, R_paths_z, c='g', alpha=0.5)
    ax_b_0.legend(('R_path_x','R_path_y','R_path_z'))
    ax_b_0.set_title(path_title)
    
    ax_b_1 = f.add_subplot(gs[2, 2:])
    ax_b_1.scatter(idxs_paths, q_paths_x, c='m', marker='^', s=20, alpha=0.5)
    ax_b_1.scatter(idxs_paths, q_paths_y, c='lime', marker='^', s=20, alpha=0.5)
    ax_b_1.scatter(idxs_paths, q_paths_z, c='c', marker='^', s=20, alpha=0.5)
    ax_b_1.plot(idxs_paths, q_paths_x, c='r', alpha=0.5)
    ax_b_1.plot(idxs_paths, q_paths_y, c='b', alpha=0.5)
    ax_b_1.plot(idxs_paths, q_paths_z, c='g', alpha=0.5)

    ax_b_1.legend(('q_path_x','q_path_y','q_path_z'))
    ax_b_1.set_title(path_title)

    plt.savefig(f'R_q_slerp_comparison_{fig_idx}.png')

    '''
    ax[2].plot(idxs_metrics, geodesic_metrics, c='r')
    ax[2].plot(idxs_metrics, hyperbolic_metrics, c='g')
    ax[2].plot(idxs_metrics, frobenius_metrics, c='b')
    ax[2].scatter(idxs_metrics, geodesic_metrics, s=8,  c='r')
    ax[2].scatter(idxs_metrics, hyperbolic_metrics, s=8,  c='g')
    ax[2].scatter(idxs_metrics, frobenius_metrics, s=8,  c='b')
    ax[2].legend(('geodesic metric', 'hyperbolic metric', 'frobenius metric'))
    ax[2].set_title('various metrics used\nbetween R1 and Ri')
    ax[3].plot(idxs_metrics, geodesic_metrics, c='r')
    ax[3].scatter(idxs_metrics, geodesic_metrics, s=8,  c='r')
    ax[3].set_yscale('log')
    ax[3].set_title('geodesic metric plotted \nin log scale for accuracy')
    plt.savefig('slerp_0.png')
    '''
    if draw_3d:
        sphere_x, sphere_y, sphere_z = draw_sphere(100)
        #f3d, ax3ds = plt.subplots(1,2,figsize=(20,12))
        f3d = plt.figure(figsize=(20,9))
        f3d.suptitle(suptitle)
        ax3d_0 = f3d.add_subplot(1, 2, 1, projection='3d')
        ax3d_1 = f3d.add_subplot(1, 2, 2, projection='3d')
        plt.subplots_adjust(left=0.03,bottom=0.02,right=0.97,top=0.76,wspace=0.1,hspace=0.4)
       
        ax3d_0.scatter3D(R_paths_x, R_paths_y, R_paths_z, marker='o', s=30, c='r', alpha=0.8)
        ax3d_0.scatter3D(q_paths_x, q_paths_y, q_paths_z, marker='^', s=20, c='g', alpha=0.8)
        ax3d_0.scatter3D(sphere_x, sphere_y, sphere_z, marker='.', s=3, c='c', alpha=0.2)
        ax3d_0.plot3D(R_paths_x, R_paths_y, R_paths_z, linestyle='-.', linewidth=1, c='r', alpha=0.5)
        ax3d_0.plot3D(q_paths_x, q_paths_y, q_paths_z, linestyle=':', linewidth=1, c='g', alpha=0.5)
        ax3d_0.legend(('path from A to B using R', 'path from A to B using q', 'unit sphere'))
        ax3d_0.set_title(path_title)

        ax3d_1.scatter3D(R_ks_x, R_ks_y, R_ks_z, marker='^', s=20, c='r', alpha=1.0)
        #ax3d_1.scatter3D(R_ks_norm_x,R_ks_norm_y, R_ks_norm_z, marker='+', s=20, c='g', alpha=1.0)
        ax3d_1.scatter3D(q_ks_x, q_ks_y, q_ks_z, marker='^', s=20, c='m', alpha=1.0)
        #ax3d_1.scatter3D(q_ks_norm_x,q_ks_norm_y, q_ks_norm_z, marker='+', s=20, c='lime', alpha=1.0)
        ax3d_1.scatter3D(sphere_x, sphere_y, sphere_z, marker='.', s=3, c='c', alpha=0.3)
        #ax3d_1.legend(('R_ks', 'normalized R_ks', 'q_ks', 'normalized q_ks', 'unit sphere'))
        ax3d_1.legend(('R_ks', 'q_ks', 'unit sphere'))
        ax3d_1.set_title(axis_ang_title)
        for (k_x, k_y, k_z, k_norm) in zip(R_ks_x, R_ks_y, R_ks_z, R_ks_norm):
            ax3d_1.plot3D([0,k_x],[0,k_y],[0,k_z], c='r', linewidth=1, alpha=0.5)
            #ax3d_1.plot3D([0,k_x/k_norm],[0,k_y/k_norm],[0,k_z/k_norm], c='g', linewidth=1, alpha=0.5)
        for (k_x, k_y, k_z, k_norm) in zip(q_ks_x, q_ks_y, q_ks_z, q_ks_norm):
            ax3d_1.plot3D([0,k_x],[0,k_y],[0,k_z], c='m', linewidth=1, alpha=0.5)
            #ax3d_1.plot3D([0,k_x/k_norm],[0,k_y/k_norm],[0,k_z/k_norm], c='lime', linewidth=1, alpha=0.5)
        plt.savefig(f'R_q_slerp_comparison_3d_{fig_idx}.png')

def compute_and_visualize(euler_angles_deg1: np.ndarray, euler_angles_deg2: np.ndarray, n_interp: int, v_init: np.ndarray, draw_3d: bool, fig_idx: int):
    if not isinstance(euler_angles_deg1, np.ndarray) or not isinstance(euler_angles_deg2, np.ndarray) or not isinstance(v_init, np.ndarray):
        raise NotNumpyArray
    if euler_angles_deg1.ndim != 1 or euler_angles_deg2.ndim != 1 or v_init.ndim != 1:
        raise NotOneDimArray
    if euler_angles_deg1.shape[0] != 3 or euler_angles_deg2.shape[0] != 3 or v_init.shape[0] != 3:
        raise Not3Vector
    if not isinstance(n_interp, int) or not isinstance(fig_idx, int):
        raise NotInt

    euler_angles_rad1 = deg2rad(euler_angles_deg1)
    euler_angles_rad2 = deg2rad(euler_angles_deg2)

    R1 = R_from_euler_angles_rad(euler_angles_rad1)
    R2 = R_from_euler_angles_rad(euler_angles_rad2)
    Rs_interp = slerp_R(R1, R2, n_interp)
    
    q1 = q_from_euler_angles_rad(euler_angles_rad1)
    q2 = q_from_euler_angles_rad(euler_angles_rad2)
    qs_interp_direct = slerp_q_direct(q1, q2, n_interp)
    qs_interp_exp_and_log = slerp_q_exp_and_log(q1, q2, n_interp) 

    init_conditions = (euler_angles_deg1, euler_angles_deg2, R1, R2, q1, q2, v_init)
    visualize(init_conditions, Rs_interp, qs_interp_direct, draw_3d, fig_idx)

def main():
    draw_3d = True
    euler_angles_deg1_s = [np.array([45,0,0]), np.array([0,45,0]), np.array([125,100,40]), np.array([125,100,40])]
    euler_angles_deg2_s = [np.array([-45,0,0]), np.array([0,-45,0]), np.array([75,150,150]), np.array([75,150,250])]
    n_interps = [12, 30, 50, 50]
    v_init = np.array([1,1,1])
    v_init = v_init / np.linalg.norm(v_init)
    for (i, (euler_angles_deg1, euler_angles_deg2, n_interp)) in enumerate(zip(euler_angles_deg1_s, euler_angles_deg2_s, n_interps)):
        print(f'\n\nCase #{i}')
        print(f'euler_angles_deg1 : {euler_angles_deg1}\neuler_angles_deg2 : {euler_angles_deg2}\nn_interp : {n_interp}\nv_init :\n{v_init}')
        compute_and_visualize(euler_angles_deg1, euler_angles_deg2, n_interp, v_init, draw_3d, i)
    plt.show()

if __name__=='__main__':
    main()

