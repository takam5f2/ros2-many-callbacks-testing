node_num_list = [10,20,40,80,160,300]
node_num_per_container_list = [1,2,4,8,16,32,64]

for node_num in node_num_list:
    for node_num_per_container in node_num_per_container_list:
        if node_num < node_num_per_container:
            continue
        print(f"./run_vtune_systemwide.sh {node_num} {node_num_per_container} multi_threaded")
