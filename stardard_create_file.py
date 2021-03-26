num_col_gem5 = 9
for node in range(0, num_col_gem5*num_col_gem5):
    f = open ("./standard_cpu_output/" + str(node)+".txt",'w')
    f2 = open ("./task_resnet/" + str(node)+".txt",'w')