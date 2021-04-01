import math
import os
import sys
from enum import Enum

class layerType(Enum):
	CONV = 0
	POOL = 1
	FC = 2

class layerModel:
	def __init__(self,layer_type, i_H, i_W, i_ch, w_size, stride, padding, o_ch):
		self.layer_type = layer_type
		self.i_H = i_H
		self.i_W = i_W
		self.i_ch = i_ch
		self.w_size = w_size
		self.stride = stride
		self.padding = padding
		self.o_ch = o_ch

		self.o_W = int((i_W + 2*padding - w_size)/stride) + 1
		self.o_H = int((i_H + 2*padding - w_size)/stride) + 1

	# 层内总的乘法数目
	def calComputationNum(self):
		# 区分池化与卷积、全连接
		if self.layer_type.value != 1:
			c_num = self.o_H * self.o_W *self.o_ch * self.w_size * self.w_size * self.i_ch
		else:
			# 当是池化时，假设最大值或者均值池化，都是输入特征值乘上一个权重（0、1、0.25……）
			c_num = self.i_H * self.i_W * self.i_ch
		return c_num
	
	# 层的输入神经元的数目
	def getInputNeuNum(self):
		i_num = self.i_H * self.i_W * self.i_ch
		return i_num
	
	# 层的每个输出神经元所需的计算量（乘法数目）
	def getNeuComputeNum(self):
		if self.layer_type.value != 1:
			c_num = self.w_size * self.w_size * self.i_ch
		else:
			c_num = self.w_size * self.w_size
		return c_num

	# 层的(输出)神经元数目
	def getLayerNeuNum(self):
		o_num = self.o_H * self.o_W * self.o_ch
		return o_num
	
	#连接数，即当前层与前一层的link数量，未考虑偏置
	def getLinkNum(self):
		l_num = self.w_size * self.w_size * self.o_H * self.o_W * self.o_ch
		return l_num

class DNNModel:
	def __init__(self, name):
		self.name = name
		self.layer_num = 0
		self.layer_list = {}

	def addLayer(self, layer):
		self.layer_list[self.layer_num] = layer
		self.layer_num += 1

	def getTotalComputeNum(self):
		c_num = 0
		for i in range(0,self.layer_num):
			c_num += self.layer_list[i].calComputationNum()
		return c_num
	
#CONV只管W和H方向的，相同ch的神经元的连接应该也是相同的
class DNNNeuronModel:
	def __init__(self, DNN):
		self.DNN = DNN
		self.layer_num = DNN.layer_num

		#layer_neu_num：每一层的神经元数目；layer_type：层类型；layer_neu_compute_list：当前层输出一个神经元所需的计算量
		self.layer_neu_num = {}
		self.layer_type = {}
		self.layer_neu_compute_list = {}

		# link_list_i[layer][neuron] = i_neuron_id；当前层神经元与上一层的神经元的连接关系
		# link_list_o[layer][neuron] = o_neuron_id；上一层神经元与当前层的神经元的连接关系
		self.link_list_i = {}
		self.link_list_o = {}

		self.getLayerInfo()
		self.getLinkList()
	
	def getLayerInfo(self):
		for i in range(0, self.layer_num):
			self.layer_neu_num[i] = self.DNN.layer_list[i].getLayerNeuNum()
			self.layer_type[i] = self.DNN.layer_list[i].layer_type
			self.layer_neu_compute_list[i] = self.DNN.layer_list[i].getNeuComputeNum()

	def getConvPoolLinkList(self, layer):
		w_size = layer.w_size
		stride = layer.stride

		link_list_i = {}
		for i in range(0, layer.o_H):
			for j in range(0, layer.o_W):
				neu_id = j  + i * layer.o_W
				link_list_i[neu_id] = []
				for m in range(0, w_size):
					for n in range(0, w_size):
						link_neu_id = j * stride + n + ( i * stride + m) * layer.i_W
						link_list_i[neu_id].append(link_neu_id)
		#link关系，neu_id_ch = neu_id * o_ch + ch_o , link_neu_id_ch = link_neu_id * i_ch + ch_i
		#Conv每一个ch_o都与(0, i_ch)的ch_i相联系, Pool每个ch_o只与对应的ch_i相联系（ch_o=ch_i）
		return link_list_i

	def getConvLinkList(self, layer):
		i_neu_num = layer.getInputNeuNum()
		w_size = layer.w_size
		stride = layer.stride
		padding = layer.padding

		link_list_i = {}
		link_list_o = {}
		for i in range(0, i_neu_num):
			link_list_o[i] = []

		for i in range(0, layer.o_H):
			for j in range(0, layer.o_W):
				for ch_o in range(0, layer.o_ch):
					neu_id = (j  + i * layer.o_W) * layer.o_ch + ch_o
					link_list_i[neu_id] = []
					for m in range(0, w_size):
						for n in range(0, w_size):
							i_neu_x = j * stride - padding + n
							i_neu_y = i * stride - padding + m
							if i_neu_x >= 0 and i_neu_x < layer.i_W and i_neu_y >= 0 and i_neu_y < layer.i_H:
								for ch_i in range(0, layer.i_ch):
									i_neu_id = (i_neu_x + i_neu_y * layer.i_W) * layer.i_ch + ch_i
									link_list_i[neu_id].append(i_neu_id)
									link_list_o[i_neu_id].append(neu_id)
		#link关系，neu_id_ch = neu_id * o_ch + ch_o , link_neu_id_ch = i_neu_id * i_ch + ch_i
		#Conv每一个ch_o都与(0, i_ch)的ch_i相联系, Pool每个ch_o只与对应的ch_i相联系（ch_o=ch_i）
		return link_list_i, link_list_o
	
	def getConvLinkList_2(self, layer):
		w_size = layer.w_size
		stride = layer.stride
		padding = layer.padding

		link_list_i = {}
		link_list_o = {}

		for i in range(0, layer.i_H):
			for j in range(0, layer.i_W):
				i_neu_id = i * layer.i_W + j 
				link_list_o[i_neu_id] = []

		for i in range(0, layer.o_H):
			for j in range(0, layer.o_W):
				neu_id = j  + i * layer.o_W
				link_list_i[neu_id] = []
				for m in range(0, w_size):
					for n in range(0, w_size):
						i_neu_x = j * stride - padding + n
						i_neu_y = i * stride - padding + m
						if i_neu_x >= 0 and i_neu_x < layer.i_W and i_neu_y >= 0 and i_neu_y < layer.i_H:
							i_neu_id = i_neu_x + i_neu_y * layer.i_W
							link_list_i[neu_id].append(i_neu_id)
							link_list_o[i_neu_id].append(neu_id)
		#link关系，neu_id_ch = neu_id * o_ch + ch_o , link_neu_id_ch = i_neu_id * i_ch + ch_i
		#Conv每一个ch_o都与(0, i_ch)的ch_i相联系, Pool每个ch_o只与对应的ch_i相联系（ch_o=ch_i）
		return link_list_i, link_list_o
	
	def getPoolLinkList(self, layer):
		i_neu_num = layer.getInputNeuNum()
		w_size = layer.w_size
		stride = layer.stride

		link_list_i = {}
		link_list_o = {}
		for i in range(0, i_neu_num):
			link_list_o[i] = []

		for i in range(0, layer.o_H):
			for j in range(0, layer.o_W):
				for ch_o in range(0, layer.o_ch):
					neu_id = (j  + i * layer.o_W) * layer.o_ch + ch_o
					link_list_i[neu_id] = []
					for m in range(0, w_size):
						for n in range(0, w_size):
							link_neu_id = (j * stride + n + ( i * stride + m) * layer.i_W) * layer.o_ch + ch_o
							link_list_i[neu_id].append(link_neu_id)
							link_list_o[link_neu_id].append(neu_id)
		#link关系，neu_id_ch = neu_id * o_ch + ch_o , link_neu_id_ch = link_neu_id * i_ch + ch_i
		#Conv每一个ch_o都与(0, i_ch)的ch_i相联系, Pool每个ch_o只与对应的ch_i相联系（ch_o=ch_i）
		return link_list_i, link_list_o

	def getPoolLinkList(self, layer):
		i_neu_num = layer.getInputNeuNum()
		w_size = layer.w_size
		stride = layer.stride

		link_list_i = {}
		link_list_o = {}
		for i in range(0, i_neu_num):
			link_list_o[i] = []

		for i in range(0, layer.o_H):
			for j in range(0, layer.o_W):
				for ch_o in range(0, layer.o_ch):
					neu_id = (j  + i * layer.o_W) * layer.o_ch + ch_o
					link_list_i[neu_id] = []
					for m in range(0, w_size):
						for n in range(0, w_size):
							link_neu_id = (j * stride + n + ( i * stride + m) * layer.i_W) * layer.o_ch + ch_o
							link_list_i[neu_id].append(link_neu_id)
							link_list_o[link_neu_id].append(neu_id)
		#link关系，neu_id_ch = neu_id * o_ch + ch_o , link_neu_id_ch = link_neu_id * i_ch + ch_i
		#Conv每一个ch_o都与(0, i_ch)的ch_i相联系, Pool每个ch_o只与对应的ch_i相联系（ch_o=ch_i）
		return link_list_i, link_list_o
	
	def getPoolLinkList_2(self, layer):
		i_neu_num = layer.i_W * layer.i_H
		w_size = layer.w_size
		stride = layer.stride

		link_list_i = {}
		link_list_o = {}
		for i in range(0, i_neu_num):
			link_list_o[i] = []

		for i in range(0, layer.o_H):
			for j in range(0, layer.o_W):
				neu_id = j  + i * layer.o_W
				link_list_i[neu_id] = []
				for m in range(0, w_size):
					for n in range(0, w_size):
						link_neu_id = j * stride + n + ( i * stride + m) * layer.i_W
						link_list_i[neu_id].append(link_neu_id)
						link_list_o[link_neu_id].append(neu_id)
		#link关系，neu_id_ch = neu_id * o_ch + ch_o , link_neu_id_ch = link_neu_id * i_ch + ch_i
		#Conv每一个ch_o都与(0, i_ch)的ch_i相联系, Pool每个ch_o只与对应的ch_i相联系（ch_o=ch_i）
		return link_list_i, link_list_o

	def getFCLinkList(self, layer):
		i_neu_num = layer.getInputNeuNum()
		o_neu_num = layer.getLayerNeuNum()
		link_list_i = {}
		link_list_o = {}
		list_i = []
		list_o = []
		for i in range(0, i_neu_num):
			list_i.append(i)
		for i in range(0, o_neu_num):
			list_o.append(i)
		
		for i in range(0, i_neu_num):
			link_list_o[i] = list_o
		for i in range(0, o_neu_num):
			link_list_i[i] = list_i
		return link_list_i, link_list_o

	def getLinkList(self):
		for i in range(0,self.layer_num):
			layer_type = self.DNN.layer_list[i].layer_type
			if layer_type.value == 0:
				self.link_list_i[i], link_list_o = self.getConvLinkList_2(self.DNN.layer_list[i])
			elif layer_type.value == 1:
				self.link_list_i[i], link_list_o = self.getPoolLinkList_2(self.DNN.layer_list[i])
			elif layer_type.value == 2:
				#self.link_list_i[i], link_list_o = self.getFCLinkList_2(self.DNN.layer_list[i])
				pass
			if i != 0:
				self.link_list_o[i-1] = link_list_o

def list_min(list):
	l_min = list[0]
	for i in list:
		if list[i] < l_min:
			l_min = list[i]
	return l_min 

def DNN_input(file_name , DNN):
	layer_num = 0
	layer_list = {}
	f = open("./NN_input/" + file_name + ".txt")
	lines = f.readlines()
	for line in lines:
		if line.startswith("#"):
			pass
		else:
			line_item = line.split(" ")
			if line_item[1] == "CONV":
				layer_type = layerType.CONV
			elif line_item[1] == "POOL":
				layer_type = layerType.POOL
			else:
				layer_type = layerType.FC
			layer_list[layer_num] = layerModel(layer_type,int(line_item[2]),int(line_item[3]), int(line_item[4]), int(line_item[5]), int(line_item[6]), int(line_item[7]), int(line_item[8]))
			DNN.addLayer(layer_list[layer_num])
			layer_num += 1
	f.close()

def neuronGroup(NeuModel , group_size, PE_ability):
	n = 0
	# layer2node[layer] = node; neuron2node[layer][neuron] = node; neu_per_node[node_id] = neu_num;
	#node_neu_num [layer] = neu_num;
	layer2node = {}
	neuron2node = {}
	neu_per_node = {}
	cycle_per_neu_node = {}
	node_neu_num = {}
	layer_num = NeuModel.layer_num

	
	for i in range(0,layer_num):
		node_num = math.ceil( NeuModel.layer_neu_num[i] / group_size )
		node_neu_num[i] = math.ceil( NeuModel.layer_neu_num[i] / node_num )
		assert(node_neu_num[i] <= group_size)
		cycle_per_neuron = ( NeuModel.layer_neu_compute_list[i] / PE_ability )
		neuron2node[i] = {}

		neu_num = NeuModel.layer_neu_num[i]
		list = []
		for j in range(0,node_num):
			list.append(j+n)
			cycle_per_neu_node[j+n] = cycle_per_neuron
			if neu_num >= node_neu_num[i]:
				neu_per_node[j+n] = node_neu_num[i]
				neu_num -= node_neu_num[i]
				for m in range(0, node_neu_num[i]):
					neu_id = j * node_neu_num[i] + m
					neuron2node[i][neu_id] = j + n
			else:
				neu_per_node[j+n] = neu_num
				for m in range(0, neu_num):
					neu_id = j * node_neu_num[i] + m
					neuron2node[i][neu_id] = j + n			
		layer2node[i] = list
		n = n + node_num
	return n, layer2node, neuron2node, neu_per_node, cycle_per_neu_node, node_neu_num

# 产生关于每个节点一共需要等待的神经元的数目，和每个节点的产生的神经元的传输的目的节点的相关信息
def create_node_task_pre(DNNNeuronModel, layer2node, neuron2node, neu_per_node, node_neu_num):
	# node_neu_need[node] = neu_num; node_neu_dst[node][neu_id] = dst_list;
	node_neu_need = {}
	node_neu_dst = {}
	#print(layer2node)
	for i in layer2node:

		for j in layer2node[i]:
			node_neu_need[j] = 0
			node_neu_dst[j] = {}

		#if i != 0 and DNNNeuronModel.layer_type[i].value != 2:
		if i != 0:
			for neu in DNNNeuronModel.link_list_o[i-1]:
				neu_node = neuron2node[i-1][neu]
				neu_id = neu % node_neu_num[i-1]
				node_neu_dst[neu_node][neu_id] = []
				o_node_list = {}

				for o_neu in DNNNeuronModel.link_list_o[i-1][neu]:
					o_neu_node = neuron2node[i][o_neu]
					o_node_list[o_neu_node] = 1
				for o_node in o_node_list:
					node_neu_need[o_node] += 1
					node_neu_dst[neu_node][neu_id].append(o_node)
		#elif DNNNeuronModel.layer_type[i].value == 2:
			#for j in layer2node[i]:
			#	node_neu_need[j] = DNNNeuronModel.layer_neu_num[i-1]
			#for j in layer2node[i-1]:
			#	for neu_id in range(0, neu_per_node[j]):
			#		node_neu_dst[j][neu_id] = layer2node[i]
			print(list_min(node_neu_need))
	return node_neu_need, node_neu_dst

def create_node_task(DNNNeuronModel, layer2node, neuron2node, neu_per_node, node_neu_num, neu_per_packet):
	# node_neu_need[node] = neu_num; node_neu_dst[node][neu_id] = dst_list;
	node_neu_need = {}
	node_neu_dst = {}
	node_neu_need_node = {}
	#print(layer2node)
	for i in layer2node:

		for j in layer2node[i]:
			node_neu_need[j] = 0
			node_neu_dst[j] = {}
			node_neu_need_node[j] = {}
			if i > 0:
				for mm in layer2node[i-1]:
					node_neu_need_node[j][mm] = 0

		#if i != 0 and DNNNeuronModel.layer_type[i].value != 2:
		if i != 0:
			if DNNNeuronModel.layer_type[i].value == 0:
				i_ch = DNNNeuronModel.DNN.layer_list[i].i_ch
				o_ch = DNNNeuronModel.DNN.layer_list[i].o_ch
				for neu in DNNNeuronModel.link_list_o[i-1]:
					for ch_i in range(0, i_ch):
						neu_1 = neu * i_ch + ch_i
						neu_node = neuron2node[i-1][neu_1]
						neu_id = neu_1 % node_neu_num[i-1]
						node_neu_dst[neu_node][neu_id] = []
						o_node_list = {}

						for o_neu in DNNNeuronModel.link_list_o[i-1][neu]:
							for ch_o in range(0, o_ch):
								o_neu_1 = o_neu * o_ch + ch_o
								o_neu_node = neuron2node[i][o_neu_1]
								o_node_list[o_neu_node] = 1
						for o_node in o_node_list:
							node_neu_need_node[o_node][neu_node] += 1;
							#node_neu_need[o_node] += 1
							node_neu_dst[neu_node][neu_id].append(o_node)
				for j in layer2node[i]:
					for node in node_neu_need_node[j]:
						node_neu_need[j] += math.ceil(node_neu_need_node[j][node]/neu_per_packet)
			elif DNNNeuronModel.layer_type[i].value == 1:
				i_ch = DNNNeuronModel.DNN.layer_list[i].i_ch
				for neu in DNNNeuronModel.link_list_o[i-1]:
					for ch_i in range(0, i_ch):
						neu_1 = neu * i_ch + ch_i
						neu_node = neuron2node[i-1][neu_1]
						neu_id = neu_1 % node_neu_num[i-1]
						node_neu_dst[neu_node][neu_id] = []
						o_node_list = {}

						for o_neu in DNNNeuronModel.link_list_o[i-1][neu]:
							o_neu_1 = o_neu * i_ch + ch_i
							o_neu_node = neuron2node[i][o_neu_1]
							o_node_list[o_neu_node] = 1
						for o_node in o_node_list:
							node_neu_need_node[o_node][neu_node] += 1;
							#node_neu_need[o_node] += 1
							node_neu_dst[neu_node][neu_id].append(o_node)
				for j in layer2node[i]:
					for node in node_neu_need_node[j]:
						node_neu_need[j] += math.ceil(node_neu_need_node[j][node]/neu_per_packet)
			elif DNNNeuronModel.layer_type[i].value == 2:
				for j in layer2node[i]:
					for i_node in layer2node[i-1]:
						node_neu_need[j] += math.ceil(neu_per_node[i_node]/neu_per_packet)
				for j in layer2node[i-1]:
					for neu_id in range(0, neu_per_node[j]):
						node_neu_dst[j][neu_id] = layer2node[i]
	return node_neu_need, node_neu_dst

def send_neu_cycle(cycle_per_neu):
	neu_per_cycle = 1 / cycle_per_neu

	#result表示cycle_per_neu或者neu_per_cycle，为整数;
	#flag：0 = result是cycle_per_neu；1 = result是neu_per_cycle；
	result = 0
	flag = 0

	if cycle_per_neu < 1:
		if neu_per_cycle < 2:
			result = 1
			flag = 0
		else:
			result = math.ceil(neu_per_cycle)
			flag = 1
	else:
		result = math.ceil(cycle_per_neu)
		flag = 0

	return result, flag

def create_task_list( folder_name, node_num, node_neu_need, node_neu_dst, neu_per_node, cycle_per_neu_node):
	for node in range(0, node_num):
		f = open(folder_name + "/" + str(node) + ".txt",'w')
		#f = open("./" + folder_name + "/" + str(node) + ".txt",'w')
		# wait指令
		if node_neu_need[node] > 0:
			print ("wait "+str(node_neu_need[node]), file = f)

		# cal send指令
		result, flag = send_neu_cycle(cycle_per_neu_node[node])
		if flag == 0:
			cycle = result
			neu_num = 1
		else:
			cycle = 1
			neu_num = result
		if flag == 0:
			for neu_id in node_neu_dst[node]:
				print ("cal "+str(cycle), file = f)
				send = "send"
				for o_neu_node in node_neu_dst[node][neu_id]:
					send += " " + str(o_neu_node)
				print(send, file = f)
		else:
			neu_id = 0
			num = len(node_neu_dst[node])
			num_send = math.ceil(num / neu_num)
			for m in range(0, num_send):
				print ("cal 1", file = f)
				for n in range(0, neu_num):
					neu_id = n + m * neu_num
					if neu_id < num:
						send = "send"
						for o_neu_node in node_neu_dst[node][neu_id]:
							send += " " + str(o_neu_node)
						print(send, file = f)
					else:
						pass

		if len(node_neu_dst[node]) == 0:
			neuron_num = neu_per_node[node]
			cycle = math.ceil(neuron_num * cycle_per_neu_node[node])
			print ("cal "+str(cycle), file = f)
		print("finish", file = f)
		f.close()

def display_output(file_name, DNN, node_num, layer2node, cycle_per_neu_node, neu_per_node):
	f = open("./"+file_name,'w')
	print("----------------------------------------------------------------------------------", file = f)
	print ("total computation num = ",DNN.getTotalComputeNum(), file = f)
	for i in range(0, DNN.layer_num):
		print("layer num ",i,"  :", ";\tinput_H/W = ", DNN.layer_list[i].i_H,"\t,  input_ch = ",DNN.layer_list[i].i_ch,";\toutput_H/W = ",DNN.layer_list[i].o_H,"\t,  output_ch = ",DNN.layer_list[i].o_ch, file = f)
	print("----------------------------------------------------------------------------------", file = f)
	print("layer num = ", str(DNN.layer_num), file = f)
	print("----------------------------------------------------------------------------------", file = f)
	print("node_num = ", str(node_num), file = f)
	print("----------------------------------------------------------------------------------", file = f)
	print("layer to node : ", file = f)
	for i in layer2node:
		print("layer ",str(i)," : \t",layer2node[i], file = f)
	print("----------------------------------------------------------------------------------", file = f)
	print("cycle_per_neu_node : ", file = f)
	for i in cycle_per_neu_node:
		print("node",str(i)," : \t",cycle_per_neu_node[i], file = f)
	print("----------------------------------------------------------------------------------", file = f)	
	print("neu_per_node : ", file = f)
	for node in neu_per_node:
		print("node",str(node)," : \t",neu_per_node[node], file = f)
	print("----------------------------------------------------------------------------------", file = f)
#设置层的基本参数
# 建立DNN模型
DNN1 = DNNModel("DNN1")
DNN_input("resnet_18", DNN1)

# 建立ANN-like DNN模型
NeuronModel = DNNNeuronModel(DNN1)

#进行neuron分组
group_size = 40000
PE_ability = 400
neu_per_packet = 4
#layer2node: 节点编号对应的层号 ； neu_per_node：每个节点的分配的神经元的数目 
#cycle_per_neuron：发包的速度（生成一个neuron所需的周期）
node_num, layer2node, neuron2node, neu_per_node, cycle_per_neu_node, node_neu_num= neuronGroup(NeuronModel, group_size, PE_ability)
print(node_num)
#生成任务文件
node_neu_need, node_neu_dst = create_node_task(NeuronModel, layer2node, neuron2node, neu_per_node, node_neu_num, neu_per_packet)
create_task_list( "../task_resnet", node_num, node_neu_need, node_neu_dst, neu_per_node, cycle_per_neu_node)

# 输出
display_output("output_resnet", DNN1, node_num, layer2node, cycle_per_neu_node, neu_per_node)