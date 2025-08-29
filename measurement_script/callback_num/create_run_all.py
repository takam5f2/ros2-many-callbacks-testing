node_num_list = [10,20,40,80,160,300]
publish_num = [1000,2000,4000,6000,8000,10000,12000,14000,16000,18000,20000]

for node_num in node_num_list:
    for pub_num in publish_num:
        topic_hz = pub_num // node_num
        print(f"./run_vtune_systemwide.sh {node_num} {topic_hz} single_threaded")
