import math
import os
import sys 

# configurations 
width_ifmap = 32
width_flit = 128


class cc_node:
    def __init__(self,id,OC,OH,IC,KH):
        self.id = id
        self.OC = OC
        self.OH = OH
        self.IC = IC
        self.KH = KH
        self.OW = 1



def main(cc_node_index_list,mem_node_list,IC,OC,OH,OW,KW,KH):
    OC1 = 2; IC1 = 2; OW1 = 1;  OH1 = 3; KW1= 1;   KH1 = 1;  
    OC0 =1 ; IC0 = 1; OW0 = OW; OH0 = 2; KW0 = KW; KH0 = KH;   # ROW stationary dataflow; each cluster: KH*OH PEs 
    
    OC2 = math.ceil(OC/OC1/OC0)
    IC2 = math.ceil(IC/IC1/IC0)
    OW2 = math.ceil(OW/OW1/OW0)
    OH2 = math.ceil(OH/OH1/OH0)
    KW2 = math.ceil(KW/KW1/KW0)
    KH2 = math.ceil(KH/KH1/KH0)
    
    cal_cycles = OW0*KW0*IC0*OC0
    assert(OC1*OH1*IC1*KH1 == len(cc_node_index_list))

    # generate cc nodes
    index = 0
    cc_node_list = []
    for oc1 in range (OC1):
        for oh1 in range (OH1):
            for ic1 in range (IC1):
                for kh1 in range (KH1):
                    the_cc_node = cc_node(cc_node_index_list[index],oc1,oh1,ic1,kh1)
                    cc_node_list.append(the_cc_node)
                    index += 1
    
    
    # TODO cal packets_ifmap
    packets_ifmap = math.ceil( OH0*OW0*IC0*width_ifmap/5.0/width_flit ) # 1 data packet = 5  flits
    assert (packets_ifmap > 0)
    packets_wgts = 0 # TODO not considered
    packte_ofmap = 0 # TODO not considered

    mem_node_index = 0
    tag = 100
    # generate task lists
    for oc2 in range (OC2):
        for oh2 in range (OH2):
            for ic2 in range (IC2):
                for kh2 in range (KH2):

                    ### NoC level
                    tag += 1
                    for cc in cc_node_list:
                        cc_node_file_name  = cc.id 

                        # cc waiting for ifmap
                        mem_node_index = (mem_node_index+1)% len(mem_node_list)
                        mem_node_file_name = mem_node_list[mem_node_index]

                        with open ("../standard_cpu_task/" + str(cc_node_file_name)+".txt",'a') as cc_file:
                            print ("wait "+str(packets_ifmap)+" "+str(mem_node_file_name)+" "+str(tag)+str(cc.IC)+str(cc.OW)+str(cc.OH),file = cc_file)
                        # cc cal
                            print ("cal "+str(cal_cycles), file = cc_file)


                        # cc psum TODO
                        # cc output results
    
    # finish
    for cc in cc_node_list:
        cc_node_file_name  = cc.id 
        with open ("../standard_cpu_task/" + str(cc_node_file_name)+".txt",'a') as cc_file:
            print ("finish", file = cc_file)
    
    for mc in mem_node_list:
        with open ("../standard_cpu_task/" + str(mc)+".txt",'w') as cc_file:
            print ("")
    

os.system("rm ../standard_cpu_task/*.txt") 
main(cc_node_index_list = [1,2,3,5,6,7,9,10,11,13,14,15],mem_node_list = [0,4,8,12],IC=4,OC=6,OH=12,OW=12,KW=3,KH=3)


    
