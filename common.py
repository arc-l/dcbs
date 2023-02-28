
import numpy as np
import Grids
import pickle
import json



def write_data_to_csv(file_name,num_agents=None,success_rate=None,makespan_optimality=None,soc_optimality=None):
    with open(file_name,'w') as file:
        data_size=len(num_agents)
        file.write('num_agents,success_rate,mkpn,soc\n')
        for i in range(data_size):
            if num_agents is not None:
                file.write(str(num_agents[i]))
                file.write(",")
            if success_rate is not None:
                file.write(str(success_rate[i]))
                file.write(",")
            if makespan_optimality is not None:
                file.write(str(makespan_optimality[i]))
                file.write(',')
            if soc_optimality is not None:
                file.write(str(soc_optimality[i]))
                file.write('\n')

def write_multiple_csv(file_name,itemList,dataList):
    num_items=len(itemList)
    data_length=len(dataList[0])
    with open(file_name,'w') as file:
        for i,itemName in enumerate(itemList):
            file.write(itemName)
            if i==num_items-1:
                file.write('\n')
            else:
                file.write(',')
        for i in range(data_length):
            for j,data in enumerate(dataList):
                file.write(str(data[i]))
                if j==num_items-1:
                    file.write('\n')
                else:
                    file.write(',')
    
                




        

def generate_random_instance(graph,num_agents):
    starts=graph.getV()
    goals=graph.getV()
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    return starts[:num_agents],goals[:num_agents]