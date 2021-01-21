# -*- coding: UTF-8 -*-

# Filename	: output.py
# Author	: Xuyan Wang
# Date		: 2021-01-20
# Email		: chaixiaojing22@163.com
# Command			: python3 output.py

import math
import copy
import numpy as np
from matplotlib import pyplot as plt

class parse :
    def __init__ (self, file_name):
        self.file_name = file_name

        self.communication_time_tick = {}
        self.cal_start_tick = {}
        self.wait_start_tick = {}
        self.communication_all = 0
        self.cal_all = 0
        self.finish_tick = 0
        self.finish_task_line_num = 0

        self.parse_file()
        self.compute_wait_tick()
    
    def parse_file(self):
        with open(self.file_name, "r") as f:
            lines = f.readlines()
            for line_num, line in enumerate(lines):
                line = line.strip().split()
                if line[0] == "communication":
                    self.communication_time_tick[line_num] = line[3]
                    self.communication_all += int(line[3])
                    self.cal_start_tick[line_num] = line[7]
                elif line[0] == "finish":
                    self.finish_tick = int(line[4])
                    self.finish_task_line_num = line[8]
        self.cal_all = self.finish_tick - self.communication_all
    
    def compute_wait_tick(self):
        for comm_num, comm in self.communication_time_tick.items():
            wait_tick = int(self.cal_start_tick[comm_num]) - int(self.communication_time_tick[comm_num])
            self.wait_start_tick[comm_num] = wait_tick


node = {}
communication_time_all = {}
comm_time_all = 0
cal_time_all = {}
c_time_all = 0

num_row = 12
num_col = 15
num_col_gem5 = 16
cc_pe_num = (num_row-1)*(num_col-1)
max_finish = 0
max_id = 0

for i in range(0 , num_row):
    for j in range(1, num_col):
        nodeID = i*num_col_gem5 + j 
        file_name = str(nodeID) + ".txt"
        node[nodeID] = parse(file_name)

        if node[nodeID].finish_tick > max_finish :
            max_finish = node[nodeID].finish_tick
            max_id = nodeID
        communication_time_all[nodeID] = node[nodeID].communication_all
        comm_time_all += node[nodeID].communication_all
        cal_time_all[nodeID] = node[nodeID].cal_all
        c_time_all += node[nodeID].cal_all
        #print(str(i)+'     '+str(communication_time_all[i]) + "   " + str(cal_time_all[i]))
print("max finish tick : "+ str(max_finish)+"  node id : "+str(max_id))
average_comm = int(comm_time_all / cc_pe_num)
average_cal = int(c_time_all / cc_pe_num)

x = []
y1 = []
y2 = []

for a , b in communication_time_all.items():
    if a < 256:
        x.append(a)
        y1.append(b)
        y2.append(cal_time_all[a])
plt.figure(1)
plt.title("conv1")
x_label = ['communication','calculation']
y_label = ['average cycle']
width = 0.5
plt.bar(x_label[0], average_comm , width, label = x_label[0])
plt.bar(x_label[1], average_cal , width, label = x_label[1])

plt.ylabel(y_label)
plt.legend()
plt.show()

plt.figure(2)
plt.title("conv1")
plt.xlabel('cc node id')
plt.ylabel('cycle')
width = 0.5
p1 = plt.bar(x, y2,width)
p2 = plt.bar(x, y1,width, bottom=y2)
plt.legend((p1[0], p2[0]), ('cal', 'communication'))
plt.show()


