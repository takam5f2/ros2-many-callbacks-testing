node_num_list = [10, 20, 40, 70, 100, 150, 175, 200, 225, 250]
topic_num_per_node_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]

for node_num in node_num_list:
    for topic_num in topic_num_per_node_list:
        print(f"./run_measurement.sh {node_num} {topic_num}")
