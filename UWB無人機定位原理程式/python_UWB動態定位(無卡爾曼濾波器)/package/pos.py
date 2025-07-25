import numpy as np
from numpy.linalg import inv 
import package.pos_anchor_select as pos_anchor_select


def positioning_ls(distance_list, anchorUseList):

    distance_list, a, num_anchor = pos_anchor_select.get_distance_need_use(distance_list, anchorUseList)
    A = np.zeros((num_anchor-1, 3))
    b = np.zeros((num_anchor-1, 1))
    position = np.zeros((3, 1))

    for i in range(num_anchor-1):
        A[i][0] = 2 * (a[i][0] - a[num_anchor-1][0])
        A[i][1] = 2 * (a[i][1] - a[num_anchor-1][1])
        A[i][2] = 2 * (a[i][2] - a[num_anchor-1][2])

    for i in range(num_anchor-1):
        b[i] = a[i][0]**2 + a[i][1]**2 + a[i][2]**2 - (a[num_anchor-1][0]**2 + a[num_anchor-1][1]**2  + a[num_anchor-1][2]**2) - (distance_list[i]**2 - distance_list[num_anchor-1]**2)

    position = inv(A.T @ A) @ (A.T @ b)
    return position


def positioning_wls(distance_list, anchorUseList):
    max_num_anchor = 8
    distance_list, a, num_anchor = pos_anchor_select.get_distance_need_use(distance_list, anchorUseList)   
    Q_variance_list = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]    
    Q_variance = []
    for i in range(max_num_anchor):
        if anchorUseList[i] == 1:
            Q_variance.append(Q_variance_list[i])
    
    h = np.zeros((num_anchor, 1))
    for i in range(num_anchor):
        h[i] = distance_list[i]**2 - (a[i][0]**2 + a[i][1]**2 + a[i][2]**2) 

    Ga = np.zeros((num_anchor, 4))
    for i in range(num_anchor):
        Ga[i][0] = -2 * a[i][0]
        Ga[i][1] = -2 * a[i][1]
        Ga[i][2] = -2 * a[i][2]
        Ga[i][3] = 1

    Q = np.zeros((num_anchor, num_anchor))
    B = np.zeros((num_anchor, num_anchor))
    for i in range(num_anchor):
        Q[i][i] = Q_variance[i]
        B[i][i] = distance_list[i]
    
    Psi = 4 * (B @ Q @ B)
    Psi_inv = np.linalg.inv(Psi)
    Za = inv(Ga.T @ Psi_inv @ Ga) @ (Ga.T @ Psi_inv @ h)
    position = Za[0:3]
    return position


def positioning_wls_with_R(distance_list, anchorUseList):
    max_num_anchor = 8
    distance_list, a, num_anchor = pos_anchor_select.get_distance_need_use(distance_list, anchorUseList)
    anchor_variance_list = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  
    anchor_variance = []
    for i in range(max_num_anchor):
        if anchorUseList[i] == 1:
            anchor_variance.append(anchor_variance_list[i])

    h = np.zeros((num_anchor, 1))
    Ga = np.zeros((num_anchor, 4))
    Za = np.zeros((num_anchor, 1))
    Q = np.zeros((num_anchor, num_anchor))
    B = np.zeros((num_anchor, num_anchor))
    Psi = np.zeros((num_anchor, num_anchor))

    hp = np.zeros((4, 1))
    Gap = np.array([(1, 0, 0), 
                    (0, 1, 0), 
                    (0, 0, 1),
                    (1, 1, 1)])
    Zap = np.zeros((3, 1))
    Psip = np.zeros((4, 4))
    Bp = np.zeros((4, 4))
    cov_Za = np.zeros((4, 4))

    for anchor in range(num_anchor):
        h[anchor] = distance_list[anchor]**2 - (a[anchor][0]**2 + a[anchor][1]**2 + a[anchor][2]**2)
        Ga[anchor, 0] = -2*a[anchor][0]
        Ga[anchor, 1] = -2*a[anchor][1]
        Ga[anchor, 2] = -2*a[anchor][2]
        Ga[anchor, 3] = 1
        Q[anchor, anchor] = anchor_variance[anchor]
        B[anchor, anchor] = distance_list[anchor]
    

    Psi = 4*(B@Q@B)
    Psi_inv = inv(Psi)
    Za = inv(Ga.T@Psi_inv@Ga)@(Ga.T@Psi_inv@h)
    
    for index in range(3):
        hp[index] = Za[index]**2
    hp[3] = Za[3]
    
    cov_Za = inv(Ga.T@Psi_inv@Ga)
    for index in range(3):
        Bp[index, index] = Za[index]
    Bp[3, 3] = 0.5

    Psip = 4*(Bp@cov_Za@Bp)
    Psip_inv = inv(Psip)
    Zap = np.matmul(inv(Gap.T@Psip_inv@Gap), (Gap.T@Psip_inv@hp))
    Zp = np.sqrt(abs(Zap))
    position = Zp
    return position

