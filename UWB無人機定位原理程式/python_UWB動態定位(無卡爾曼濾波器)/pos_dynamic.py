import package.pos as pos
import re
import numpy as np

file_num = 1
filename = 'uwb_data'+str(file_num)+'.txt'
f_distance = open("path_data/"+filename, 'r')

#讀取距離資料
lines = f_distance.readlines()
lines2 = lines[::2]
anchor_char = []
for line in lines:
    if(line[0] == 'A'):
        anchor_char.append(line)


num_anchor = 8
distance_each_anchor = []
distance = []
anchor_receive_total = []
for a in range(num_anchor):
    distance_each_anchor.append([])
    distance.append(-1)
    anchor_receive_total.append(0)


index_count = 0
receive_correct = True

#從距離資料中取出距離資訊
for char in anchor_char:
    ant_index = int(char[1]) - 1
    anchor_receive_total[ant_index] += 1
    pattern = r":([0-9]+\.[0-9]+)" 
    match = re.search(pattern, char)
    if match:
        float_distance = float(match.group(1))
        if float_distance > 20 or float_distance < 0:
            float_distance = -1
        distance[ant_index] = float_distance
        index_count += 1
    if index_count == 8:
        index_count = 0
        num_anchor_rx_error = 0
        for i in range(num_anchor):
            if distance[i] == -1:
                distance[i] = 0
                num_anchor_rx_error += 1
        
        # 若過大誤差的距離大於3組，這一輪不定位  
        if num_anchor_rx_error > 3: 
            receive_correct = False
        
        if receive_correct:
            for a in range(num_anchor):
                distance_each_anchor[a].append(distance[a])
                distance[a] = -1
        else:
            receive_correct = True
            for a in range(num_anchor):
                distance[a] = -1 



f_distance.close()

print(anchor_receive_total)
print(len(distance_each_anchor[1]))

filter = 5

f_ls = open("position_result\\lsdata.txt", 'w') 
f_wls = open("position_result\\wlsdata.txt", 'w') 
f_wls_r = open("position_result\\wlsrdata.txt", 'w')

first_pos = False

X_sum = np.zeros((5, 3))



#計算定位結果以及將定位結果儲存至positin_result中，分別有三個檔案表示不同方法的定位結果
for index in range(filter-1, len(distance_each_anchor[0])):
    
    anchorUseList =    [1, 1, 1, 1, 1, 1, 1, 1]
    distance_filtered =  [0, 0, 0, 0, 0, 0, 0, 0]     #distance_filter = [0 0 0 0 0 0 0 0]
    anchor_use_total = [0, 0, 0, 0, 0, 0, 0, 0]

    for i in range(filter):
        for anchor in range(num_anchor):
            if distance_each_anchor[anchor][index-i] != 0:
                anchor_use_total[anchor] += 1
                anchorUseList[anchor] = 1
            else:
                anchorUseList[anchor] = 0
            distance_filtered[anchor] += distance_each_anchor[anchor][index-i] 

    for anchor in range(8):
        if anchor_use_total[anchor] != 0:
            distance_filtered[anchor] = distance_filtered[anchor] / anchor_use_total[anchor]

    position1 = pos.positioning_ls(distance_filtered, anchorUseList)
    position2 = pos.positioning_wls(distance_filtered, anchorUseList)
    position3 = pos.positioning_wls_with_R(distance_filtered, anchorUseList)
    
    f_ls.write(f"{position1[0, 0]} {position1[1, 0]} {position1[2, 0]}\n")
    f_wls.write(f"{position2[0, 0]} {position2[1, 0]} {position2[2, 0]}\n")
    f_wls_r.write(f"{position3[0, 0]} {position3[1, 0]} {position3[2, 0]}\n")

f_ls.close()
f_wls.close()
f_wls_r.close()

