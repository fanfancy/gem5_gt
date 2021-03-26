import math
import os
import sys

node_num = 55
node_link_o = {}
max_len = {}
for i in range(0, node_num):
	node_link_o[i] = {}
	max_len[i] = 0
	f = open("./task_lenet/" + str(i) + ".txt")
	lines = f.readlines()
	for line in lines:
		if line.startswith("send"):
			line_list = line.split(" ")
			if len(line_list) - 1 >= max_len[i]:
				max_len[i] = len(line_list) - 1
			for j in range(1, len(line_list)):
				node_o = int(line_list[j].strip('\n'))
				node_link_o[i][node_o] = 1
		else:
			pass
	f.close()

f2 = open("./NodeLink.txt", 'w')
print("---------Node to Node Link Situation-----------", file = f2)
for i in range(0, node_num):
	line = "node " + str(i) + ":  "
	print(node_link_o[i])
	print(max_len[i])
	for node_o in node_link_o[i]:
		line += str(node_o) + "  "
	print(line, file = f2)

f2.close()
