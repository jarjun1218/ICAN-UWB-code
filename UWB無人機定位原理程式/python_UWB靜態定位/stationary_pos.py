import re 
import numpy as np
import package.pos as pos
import math
import matplotlib.pyplot as plt 

anchor = []
num_anchor = 0
tag = []
num_point = 0
anchorUseList = [1, 1, 1, 1, 1, 1, 1, 1]

anchor_file = open('anchor_tag\\anchor_toa.txt', 'r')
tag_file = open('anchor_tag\\tag_toa.txt', 'r')



#讀取anchor資料
anchor_list = anchor_file.readlines()
for a in anchor_list:
    a_list = a.split(' ')
    anchor_set = []
    for num in a_list:
        anchor_set.append(float(num.replace("\n", '')))
    anchor.append(anchor_set)
    num_anchor += 1


#讀取測試點資料   
tag_list = tag_file.readlines()
for t in tag_list:
    t_list = t.split(' ')
    tag_set = []
    for num in t_list:
        tag_set.append(float(num.replace("\n", '')))
    tag.append(tag_set)
    num_point += 1

anchor_file.close()
tag_file.close()




#計算測試點與anchor間真實距離
real_distance = []

for i in range(num_point):
    distance_r = []
    for j in range(num_anchor):
        dis = math.sqrt(((tag[i][0]-anchor[j][0])**2 + (tag[i][1]-anchor[j][1])**2 + (tag[i][2]-anchor[j][2])**2))
        distance_r.append(dis)
    real_distance.append(distance_r) 

print("real distance")
print(np.array(real_distance))




variance_point = []
std_point = []
error_mean_point = []
rmse_point = []
mse = [0, 0, 0]


x_ls_point = []
y_ls_point = []
z_ls_point = []

x_wls_point = []
y_wls_point = []
z_wls_point = []

x_wls_r_point = []
y_wls_r_point = []
z_wls_r_point = []

#將距離資料中的距離取出以及篩選誤差過大的測距資料
total_calulation = 0
for point in range(num_point):
    
    filename = 'uwb_data'+str(point+1)+'.txt'
    f = open('collect_data\\'+filename, 'r')
    lines = f.readlines()
    lines2 = lines[::2]
    anchor_char = []
    for line in lines:
        if(line[0] == 'A' and len(line) == 9):
            anchor_char.append(line)

    dis_a = []
    distance = []
    for a in range(num_anchor):
        dis_a.append([])
        distance.append(-1)
   
    index_count = 0
    receive_correct = True
    for char in anchor_char:
        ant_index = int(char[1]) - 1
        pattern = r":([0-9]+\.[0-9]+)" 
        match = re.search(pattern, char)
        if match:
            float_distance = float(match.group(1))
            distance[ant_index] = float_distance
            index_count += 1
        if index_count == 8:
            index_count = 0
            for i in range(num_anchor):
                if distance[i] == -1 or distance[i] > 10 or distance[i] <= 0:  #將測距值為負數或是大於10m的視為過大誤差並刪除(定義方式可更改)
                    receive_correct = False
                    break
            if receive_correct:
                for a in range(num_anchor):
                    dis_a[a].append(distance[a])
                    distance[a] = -1
            else:
                receive_correct = True
        
    
    f.close()


#計算距離量測誤差相關統計資料
    dis_error = []
    for i in range(num_anchor):
        dis_error.append(np.array(dis_a[i]) - np.full((len(dis_a[i])), real_distance[point][i]))
    
    variance = []
    std = []
    error_mean = []

    for i in range(num_anchor):
        variance.append(dis_error[i].var())
        std.append(dis_error[i].std())
        error_mean.append(dis_error[i].mean())

    variance_point.append(variance)
    error_mean_point.append(error_mean)
    std_point.append(std)

    x_ls = []
    y_ls = []
    z_ls = []

    x_wls = []
    y_wls = []
    z_wls = []

    x_wls_r = []
    y_wls_r = []
    z_wls_r = []

    
    #positioning part
#包含當下的與前四組共五組的距離資料做平均

    filter = 5 #與前filter-1組的距離資料平均  
    for index in range(filter-1, len(dis_a[0])):
        distance_filter = []
        for i in range(num_anchor):
            distance_filter.append(0)
        total_calulation += 1
        for i in range(filter):
            for j in range(num_anchor):
                distance_filter[j] += (dis_a[j][index-i] / filter)    

       #計算以LS、WLS、WLS-R求得的定位結果近似解         
        position1 = pos.positioning_ls(distance_filter, anchorUseList)
        position2 = pos.positioning_wls(distance_filter, anchorUseList)
        position3 = pos.positioning_wls_with_R(distance_filter, anchorUseList)
        
        x_ls.append(position1[0, 0])
        y_ls.append(position1[1, 0])    
        z_ls.append(position1[2, 0])

        x_wls.append(position2[0, 0])
        y_wls.append(position2[1, 0])
        z_wls.append(position2[2, 0])

        x_wls_r.append(position3[0, 0])
        y_wls_r.append(position3[1, 0])
        z_wls_r.append(position3[2, 0])

    x_ls_point.append(x_ls)
    y_ls_point.append(y_ls)
    z_ls_point.append(z_ls)

    x_wls_point.append(x_wls)
    y_wls_point.append(y_wls)
    z_wls_point.append(z_wls)

    x_wls_r_point.append(x_wls_r)
    y_wls_r_point.append(y_wls_r)
    z_wls_r_point.append(z_wls_r)




    #計算定位RMSE
    rmse_3_way = [0, 0, 0]

    for i in range(len(x_ls)):
        rmse_3_way[0] += (x_ls[i] - tag[point][0])**2 + (y_ls[i] - tag[point][1])**2 + (z_ls[i] - tag[point][2])**2
        rmse_3_way[1] += (x_wls[i] - tag[point][0])**2 + (y_wls[i] - tag[point][1])**2 + (z_wls[i] - tag[point][2])**2
        rmse_3_way[2] += (x_wls_r[i] - tag[point][0])**2 + (y_wls_r[i] - tag[point][1])**2 + (z_wls_r[i] - tag[point][2])**2
        mse[0] += (x_ls[i] - tag[point][0])**2 + (y_ls[i] - tag[point][1])**2 + (z_ls[i] - tag[point][2])**2
        mse[1] += (x_wls[i] - tag[point][0])**2 + (y_wls[i] - tag[point][1])**2 + (z_wls[i] - tag[point][2])**2
        mse[2] += (x_wls_r[i] - tag[point][0])**2 + (y_wls_r[i] - tag[point][1])**2 + (z_wls_r[i] - tag[point][2])**2


    for i in range(3):
        rmse_3_way[i] = math.sqrt(rmse_3_way[i] / (len(x_ls)))
        
    rmse_point.append(rmse_3_way)

    point_choice = 2   #選擇要圖形顯示定位結果的測試點編號

    if point == point_choice-1:
        point_choice_len = len(x_ls)
        error_index_x = [[], [], []]
        error_index_y = [[], [], []]
        error_index_z = [[], [], []]
        std_index_x = [[], [], []]
        std_index_y = [[], [], []]
        std_index_z = [[], [], []]

        x = [x_ls, x_wls, x_wls_r]
        y = [y_ls, y_wls, y_wls_r]
        z = [z_ls, z_wls, z_wls_r]

        for way in range(3):
            #real data
            x_error = []
            y_error = []
            z_error = []
            x_error_mean = []
            y_error_mean = []
            z_error_mean = []
            x_std = []
            y_std = []
            z_std = []


            for i in range(len(x_ls)):
                
                x_error.append(abs(x[way][i]-tag[point][0]))
                y_error.append(abs(y[way][i]-tag[point][1]))
                z_error.append(abs(z[way][i]-tag[point][2]))
                x_error_mean.append(sum(x_error)/len(x_error))
                y_error_mean.append(sum(y_error)/len(y_error))
                z_error_mean.append(sum(z_error)/len(z_error))

                x_std.append(np.std(x[way][:i+1]))
                y_std.append(np.std(y[way][:i+1]))
                z_std.append(np.std(z[way][:i+1]))

            error_index_x[way] = x_error_mean
            error_index_y[way] = y_error_mean
            error_index_z[way] = z_error_mean

            std_index_x[way] = x_std
            std_index_y[way] = y_std
            std_index_z[way] = z_std


variance_point = np.array(variance_point)   #每個點在不同anchor的測距誤差variance
std_point = np.array(std_point)             #每個點在不同anchor的測距誤差標準差
rmse_point = np.array(rmse_point)           #每個點定位RMSE
error_mean_point = np.array(error_mean_point)     #每個點在不同anchor的平均測距誤差

rmse_total = np.sqrt(np.array(mse)/total_calulation)  #整體定位RMSE

print("error mean for each point")
print(error_mean_point)
print("error mean mean")
print(np.mean(error_mean_point, axis=0))
print("variance for each point")
print(variance_point)
print("variance mean")
print(np.mean(variance_point, axis=0))
print("std for each point")
print(std_point)
print("std mean")
print(np.mean(std_point, axis=0))

print("rmse for each point")
print("[ls wls wls-r")
print(np.round(rmse_point, 3))

print("rmse total")
print(rmse_total)



def plot_scatter_3d(point):
    fig = plt.figure(figsize=(7,7))
    ax = plt.subplot(111, projection='3d')
    ax.set_xlim(0, 2)
    ax.set_ylim(0, 2)
    x_tag = [i[0] for i in tag]
    y_tag = [i[1] for i in tag]
    z_tag = [i[2] for i in tag]

    #plot each point
    index = 100
    ax.scatter([x_tag[point-1]], [y_tag[point-1]], [z_tag[point-1]], label='tag', color='black')
    ax.scatter(x_ls_point[point-1], 
               y_ls_point[point-1], 
               z_ls_point[point-1],  label='ls', color='blue')
    ax.scatter(x_wls_point[point-1], 
               y_wls_point[point-1], 
               z_wls_point[point-1],  label='wls', color='red')
    ax.scatter(x_wls_r_point[point-1], 
               y_wls_r_point[point-1], 
               z_wls_r_point[point-1],  label='wls-r', color='green')

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    plt.title('point'+str(point)+'('+str(tag[point-1][0])+', '+str(tag[point-1][1])+', '+str(tag[point-1][2])+')')
    plt.legend()
    plt.show()

def plot_scatter_2d(point):
    fig = plt.figure(figsize=(7,7))
    x_tag = [i[0] for i in tag]
    y_tag = [i[1] for i in tag]
    z_tag = [i[2] for i in tag]

    plt.scatter([x_tag[point-1]], [y_tag[point-1]], label='tag', color='black')
    plt.scatter(x_ls_point[point-1], y_ls_point[point-1],  label='ls', color='blue')
    plt.scatter(x_wls_point[point-1], y_wls_point[point-1],  label='wls', color='red')
    plt.scatter(x_wls_r_point[point-1], y_wls_r_point[point-1], label='wls_r', color='green')
    plt.title('point'+str(point)+'('+str(tag[point-1][0])+', '+str(tag[point-1][1])+', '+str(tag[point-1][2])+')')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.show()

def plot_point():
    fig = plt.figure(figsize=(7,7))
    ax = plt.subplot(111, projection='3d')
    x_tag = [i[0] for i in tag]
    y_tag = [i[1] for i in tag]
    z_tag = [i[2] for i in tag]

    x_anchor = [i[0] for i in anchor]
    y_anchor = [i[1] for i in anchor]
    z_anchor = [i[2] for i in anchor]


    ax.scatter(x_tag, y_tag, z_tag, color='blue', label='test point')
    ax.scatter(x_anchor, y_anchor, z_anchor, color='red', label='anchor')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    for i in range(len(x_tag)):
        ax.text(x_tag[i], y_tag[i], z_tag[i], f'Point{i+1}')

    plt.title("Point")
    plt.legend()
    plt.show()


#畫出定位結果相關圖片
   
#plot_scatter_3d(point_choice) #3D定位結果分布圖     
#plot_scatter_2d(point_choice) #x-y 平面定位結果分布圖
#plot_point()                  #定位測試點式意圖