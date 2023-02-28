import networkx as nx
from typing import Tuple, List
import random
import matplotlib.pyplot as plt
from PIL import Image as im
import numpy as np
import yaml
import os
import errno


def generate_full_graph(x: int, y: int) -> nx.Graph:
    return nx.grid_graph(dim=[x, y], periodic=False)


def generate_general_graph(x: int, y: int, obs_pct: float, seed: int = 0) -> nx.Graph:
    random.seed(seed)
    # Start with a full graph
    graph = generate_full_graph(x, y)
    # Remove vertices until the graph is not connected
    while len(list(graph.nodes())) > x * y * (1 - obs_pct):
        node = random.choice(list(graph.nodes()))
        neighbors = list(graph.neighbors(node))
        temp_graph = graph.copy()
        temp_graph.remove_node(node)
        graph_connected_after_removal = True
        # Fast test for connection: check if the neighbors of the node to be removed are still connected.
        for i in range(1, len(neighbors)):
            if not nx.has_path(temp_graph, neighbors[0], neighbors[i]):
                graph_connected_after_removal = False
                break
        if graph_connected_after_removal:
            graph = temp_graph
    return graph


def generate_graph(x: int, y: int, obs: List) -> nx.Graph:
    seed=np.random.randint(1000)
    random.seed(seed)
    # Start with a full graph
    graph = generate_full_graph(x, y)
    # Remove vertices until the graph is not connected
    for o in obs:
        node = (o[0], o[1])
        graph.remove_node(node)
    return graph


def generate_warehouse_graph(
    num_shelf_x: int,
    num_shelf_y: int,
    shelf_x: int = 4,
    shelf_y: int = 2,
    corridor_x: int = 2,
    corridor_y: int = 2,
    border_x: int = 4,
    border_y: int = 4,
) -> nx.Graph:
    # Get full graph size
    x = num_shelf_x * shelf_x + border_x * 2 + (num_shelf_x - 1) * corridor_x
    y = num_shelf_y * shelf_y + border_y * 2 + (num_shelf_y - 1) * corridor_y
    # Construct a full graph
    graph = generate_full_graph(x, y)
    # Calculate all obstacle vertices
    obstacles = list()
    for i in range(num_shelf_x):
        for j in range(num_shelf_y):
            for k in range(shelf_x):
                for l in range(shelf_y):
                    obstacles.append(
                        (
                            border_y + (shelf_y + corridor_y) * j + l,
                            border_x + (shelf_x + corridor_x) * i + k,
                        )
                    )
    # Remove all obstacle nodes
    graph.remove_nodes_from(obstacles)
    return graph


def write_graph(graph: nx.Graph, file_name: str) -> None:
    if not os.path.exists(os.path.dirname(file_name)):
        try:
            os.makedirs(os.path.dirname(file_name))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    with open(file_name, "w") as file_content:
        file_content.write("type customized\n")
        nodes = list(graph.nodes())
        width = max([x for x, y in nodes]) + 1
        height = max([y for x, y in nodes]) + 1
        file_content.write("height " + str(height) + "\n")
        file_content.write("width " + str(width) + "\n")
        file_content.write("map\n")
        for y in range(height):
            for x in range(width):
                if graph.has_node((x, y)):
                    file_content.write(".")
                else:
                    file_content.write("@")
            file_content.write("\n")


def visualize_image(graph: nx.Graph, file_name: str):
    width = max([x for x, y in graph.nodes()]) + 1
    height = max([y for x, y in graph.nodes()]) + 1
    im_array = np.zeros([width, height], dtype=np.uint8)
    for x in range(0, width):
        for y in range(0, height):
            if (x, y) not in graph.nodes():
                im_array[x, y] = 0
            else:
                im_array[x, y] = 150
    data = im.fromarray(im_array)
    # H=width//2
    # W=height//2
    H=720
    W=720
    data = data.resize((H, W),resample=im.NEAREST)
    # pixels = list(data.getdata())
    # width, height = data.size
    # pixels = [pixels[i * width:(i + 1) * width] for i in range(height)]
    # # graph=nx.Graph(width,height)
    # obs_list=[]
    # for i in range(width):
    #     for j in range(height):
    #         if pixels[i][j]<10:
    #             obs_list.append((i,j))
    # graph_smaller=generate_graph(width,height,obs_list)
    # write_graph(graph_smaller,"./lak103f.map")

    data.save(file_name)
    # data.show()
    


def read_graph(file_name: str) -> nx.Graph:
    with open(file_name, "r") as file_content:
        lines = file_content.readlines()
        graph = generate_full_graph(int(lines[1].split()[1]), int(lines[2].split()[1]))
        obstacles = list()
        for y, line in enumerate(lines[4:]):
            for x, char in enumerate(line):
                if char != ".":
                    obstacles.append((x, y))
        graph.remove_nodes_from(obstacles)
        return graph


def generate_instance(graph: nx.Graph, n: int, seed: int = 0):
    # TODO customize to lifelong problem
    random.seed(seed)
    nodes = list(graph.nodes())
    starts = random.sample(nodes, n)
    goals = random.sample(nodes, n)
    return starts, goals


def write_instance(
    graph: nx.Graph,
    starts: List[Tuple[int, int]],
    goals: List[Tuple[int, int]],
    graph_name: str,
    file_name: str,
):
    if not os.path.exists(os.path.dirname(file_name)):
        try:
            os.makedirs(os.path.dirname(file_name))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    with open(file_name, "w") as file_content:
        file_content.write(graph_name + "\n")
        file_content.write(str(len(starts)) + "\n")
        for i in range(len(starts)):
            file_content.write(
                str(starts[i][0])
                + " "
                + str(starts[i][1])
                + " "
                + str(goals[i][0])
                + " "
                + str(goals[i][1])
                + "\n"
            )


def read_instance(file_name: str) -> Tuple[List[Tuple[int, int]]]:
    with open(file_name, "r") as file_content:
        lines = file_content.readlines()
        starts = list()
        goals = list()
        for line in lines[2:]:
            x1, y1, x2, y2 = line.split()
            starts.append((int(x1), int(y1)))
            goals.append((int(x2), int(y2)))
        return starts, goals


def visualize_graph(graph: nx.Graph):
    pos = {(x, y): (x, y) for x, y in graph.nodes()}
    nx.draw(graph, pos=pos, with_labels=True, node_color="lightgreen", node_size=1500)
    plt.show()


def generate_dataset():
    def generate_instances_for_single_graph(
        graph: nx.Graph, map_name: str, num_instances=30
    ):
        for robot_density in [0.1, 0.2, 0.3, 0.4, 0.5]:
            num_robots = int(robot_density * len(list(graph.nodes())))
            for i in range(30):
                instance_name = map_name + "-robot-" + str(num_robots) + "-" + str(i)
                folder_name=map_name
                starts, goals = generate_instance(graph, num_robots, i)
                write_instance(
                    graph,
                    starts,
                    goals,
                    map_name,
                    "data/problems/instances/"+folder_name+'/' + instance_name + ".scen",
                )

    # Randomly generated graphs
    for x in [5, 10, 25, 50]:
        for obs_pct in [0, 0.1, 0.2, 0.3]:
            map_name = (
                "random-"
                + str(x)
                + "x"
                + str(x)
                + "-obs_pct-"
                + "{:.1f}".format(obs_pct)
            )
            graph = generate_general_graph(x, x, obs_pct, 0)
            write_graph(graph, "data/problems/maps/" + map_name + ".map")
            generate_instances_for_single_graph(graph, map_name)
    # Warehouse style graphs
    for x in [4, 8, 12, 16]:
        map_name = "warehouse-" + str(x) + "x" + str(x)
        
        graph = generate_warehouse_graph(x, x)
        write_graph(graph, "data/problems/maps/" + map_name + ".map")
        generate_instances_for_single_graph(graph, map_name)
    # DAO maps: only generate instances
    # TODO load DAO maps


def generate_dataset_copy():
    def generate_instances_for_single_graph(
        graph: nx.Graph, map_name: str, num_instances=30
    ):
        for robot_density in [0.1, 0.2, 0.3, 0.4, 0.5]:
            num_robots = int(robot_density * len(list(graph.nodes())))
            for i in range(30):
                instance_name = map_name + "-robot-" + str(num_robots) + "-" + str(i)
                starts, goals = generate_instance(graph, num_robots, i)
                write_instance(
                    graph,
                    starts,
                    goals,
                    map_name,
                    "data/problems/instances/" + instance_name + ".scen",
                )

    for obs_pct in [0, 0.1, 0.2, 0.3]:
        x = 64
        y = 32
        map_name = (
            "random-" + str(x) + "x" + str(y) + "-obs_pct-" + "{:.1f}".format(obs_pct)
        )
        graph = generate_general_graph(x, y, obs_pct, 0)
        write_graph(graph, "data/problems/maps/" + map_name + ".map")
        generate_instances_for_single_graph(graph, map_name)


if __name__ == "__main__":
    # graph=generate_warehouse_graph(2,2)
    # visualize_graph(graph)
    # visualize_image(graph,'./tmp.png')
    # generate_dataset_copy()

    # graph=generate_full_graph(240,240)
    # graph=generate_general_graph(20,30,0.0,np.random.randint(1000))
    # graph=generate_warehouse_graph(3,3)
    # graph=read_graph("./lak103d.map")
    # write_graph(graph,'./warehouse.map')
    graph=read_graph("./warehouse.map")
    print(len(graph.nodes))
    # graph=generate_full_graph(20,20)
    # visualize_image(graph,'./20x20.png')
    # visualize_image(graph,'./tmp.png')
    #print("visualized")
    # generate_dataset()
    # pass
