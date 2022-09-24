import numpy as np
import matplotlib.pyplot as plt
from exp_and_log_map import *
import csv

def draw_xyz(ax,x_max,y_max,z_max):
    ax.plot3D((0,x_max),(0,0),(0,0),c='r',alpha=1.0)
    ax.plot3D((0,0),(0,y_max),(0,0),c='r',alpha=1.0)
    ax.plot3D((0,0),(0,0),(0,z_max),c='r',alpha=1.0)

def read_chain(path):
    d = dict()
    with open(path, newline='') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        next(csv_reader)
        for row in csv_reader:
            d[int(row[0])] = (np.array([*map(float,row[1:4])]), np.array([*map(float, row[4:7])]), np.array([*map(float,row[9:])]).reshape(int(row[7]),int(row[8])))
    return d

def draw_joint(ax, k, direc, se3):
    tr = se3[:3,3]
    ax.scatter3D(tr[0],tr[1],tr[2],c='c',marker='o',alpha=1.0)
    ax.scatter3D(tr[0]+k[0],tr[1]+k[1],tr[2]+k[2],c='r',marker='^',alpha=1.0)
    ax.plot3D((tr[0],tr[0]+k[0]),(tr[1],tr[1]+k[1]),(tr[2],tr[2]+k[2]),c='b',linestyle='-',linewidth=1,alpha=1.0)
    
    ax.scatter3D(tr[0]+direc[0],tr[1]+direc[1],tr[2]+direc[2],c='g',marker='^',s=50,alpha=1.0)
    ax.plot3D((tr[0],tr[0]+direc[0]),(tr[1],tr[1]+direc[1]),(tr[2],tr[2]+direc[2]),c='g',linestyle='-',linewidth=6,alpha=0.5)

def draw_bone(ax, tr0, tr1):
    ax.plot3D((tr0[0],tr1[0]),(tr0[1],tr1[1]),(tr0[2],tr1[2]),c='b',linestyle='-',linewidth=2,alpha=0.5)

def draw_se3_chain(d_se3: dict):
    f = plt.figure(figsize=(20,9))
    ax = f.add_subplot(1,1,1,projection='3d')
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    tr0, tr1 = None, None
    direc_coeff = 0.05
    draw_xyz(ax, 0.05, 0.05, 0.05)
    for i in range(len(d_se3)):
        '''
        ax = f.add_subplot(1,len(d_se3),i+1,projection='3d')
        ax.set_xlabel('x axis')
        ax.set_ylabel('y axis')
        ax.set_zlabel('z axis')
        '''
        print(i)
        k, direc, se3 = d_se3[i]
        tr0 = tr1
        tr1 = se3[:3, 3]
        draw_joint(ax, k, direc_coeff*direc, se3)
        if tr0 is None:
            continue
        draw_bone(ax, tr0, tr1)

def main():
    path = '../../csv_data/chain0.csv'
    d_chain = read_chain(path)
    draw_se3_chain(d_chain)
    plt.show()

if __name__=='__main__':
    main()
