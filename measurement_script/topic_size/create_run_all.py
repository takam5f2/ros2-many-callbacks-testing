node_num_list = [10,20,40,80,160,250]
topic_size_MB_per_sec_list = [1,10,20,50,100,150,200]

for node_num in node_num_list:
    for topic_size_MB_sec in topic_size_MB_per_sec_list:
        pub_sub_num = node_num // 2
        topic_size_byte_per_sec_per_node = (topic_size_MB_sec * 1024 * 1024) // pub_sub_num
        print(f"./run_vtune_systemwide.sh {node_num} {topic_size_byte_per_sec_per_node}")
