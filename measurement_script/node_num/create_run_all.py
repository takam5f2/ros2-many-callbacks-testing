node_num_list = [10,20,40,70,100,150,200,225,250,275,300,325,350,375,400] 

for node_num in node_num_list:
    print(f"./run_vtune_systemwide.sh {node_num} pub_sub")
