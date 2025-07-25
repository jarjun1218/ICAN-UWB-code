import matplotlib.pyplot as plt 
import numpy as np
from mpl_toolkits.mplot3d import axes3d

ls = np.loadtxt("position_result\\lsdata.txt")
wls = np.loadtxt("position_result\\wlsdata.txt")
wls_r = np.loadtxt("position_result\\wlsrdata.txt")

ls_coordinate = [ls[:, 0], ls[:, 1], ls[:, 2]]
wls_coordinate = [wls[:, 0], wls[:, 1], wls[:, 2]]
wls_r_coordinate = [wls_r[:, 0], wls_r[:, 1], wls_r[:, 2]]

#ground truth 路徑可自行調整
x = [1, 4, 4, 1, 1]
y = [1, 1, 4, 4, 1]
z = [1, 1, 1, 1, 1]

def plot_3d():
    
    plt.figure(figsize=(10,10))
    ax = plt.subplot(111, projection='3d')

    ax.set_xlim((0, 4))
    ax.set_ylim((0, 4))
    ax.set_zlim((0, 2))

    ax.set_title("trajectory")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    ax.plot(x, y, z, c='b', label='path')

    #選擇不同近似解的定位結果
    #LS
    #ax.plot(ls_coordinate[0], ls_coordinate[1], ls_coordinate[2], c='b', label='ls')
    #WLS
    #ax.plot(wls_coordinate[0], wls_coordinate[1], wls_coordinate[2], c='r', label='wls')
    #WLS-R
    ax.plot(wls_r_coordinate[0], wls_r_coordinate[1], wls_r_coordinate[2], c='g', label='wls-r', linestyle='-')
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_2d():
    plt.figure(figsize=(7,7))
    plt.xlim((0, 5))
    plt.ylim((0, 5))
    plt.plot(x, y, label='real path', c='red')
    #選擇不同近似解的定位結果
    #LS
    #plt.plot(ls_coordinate[0], ls_coordinate[1], label='ls', c='b', marker='x')
    #WLS
    #plt.plot(wls_coordinate[0], wls_coordinate[1], c='r', linestyle='-', label='wls', marker='x')
    #WLS-R
    plt.plot(wls_r_coordinate[0], wls_r_coordinate[1], c='b', linestyle='-',label='UWB only(WLS-R)')
   
    plt.xlabel('x [m]', fontsize=14)
    plt.ylabel('y [m]', fontsize=14)
    plt.tight_layout()
    plt.legend(fontsize=14)
    plt.show()
    


#生成定位結果圖選項

#plot_3d()  #3D定位結果圖
plot_2d()   #x-y平面 定位節果圖

