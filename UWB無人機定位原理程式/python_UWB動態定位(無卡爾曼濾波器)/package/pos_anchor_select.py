import numpy as np


max_num_anchor = 0
anchor = []
anchor_file = open('anchor_tag\\anchor_toa.txt', 'r')

anchor_list = anchor_file.readlines()
for a in anchor_list:
    a_list = a.split(' ')
    anchor_set = []
    for num in a_list:
        anchor_set.append(float(num.replace("\n", '')))
        
    anchor.append(anchor_set)
    max_num_anchor += 1
   
anchor_file.close()

anchor_array = np.array(anchor)

def get_distance_need_use(total_distancelist, anchorUseList):
    distance_use_list = []
    anchor_use_array = []
    num_anchor = 0
    for index in range(max_num_anchor):
        if anchorUseList[index] == 1:
            distance_use_list.append(total_distancelist[index])
            anchor_use_array.append(anchor_array[index])
            num_anchor += 1
    return distance_use_list, anchor_use_array, num_anchor

