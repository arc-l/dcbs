
import Grids
import MAPF
import time
from common import *
import scipy
import lbap
import math


def generate_graph_group_exchange():
    W = 9
    tmp_obstacles = [[i, j] for i in range(20, 40) for j in range(0, 60)]
    num_obstacles = 50
    np.random.shuffle(tmp_obstacles)
    obstacles = []
    for i in range(num_obstacles):
        obstacles.append(tmp_obstacles[i])
    graph = dict()
    graph["xmax"] = 60
    graph["ymax"] = 60
    graph["obstacles"] = obstacles
    with open("./instances/exchange_obs/graph_exchange.json", "w") as f:
        json.dump(graph, f)


def generate_gauss(num_robots, graph=None, mu=30, sigma=5):
    used = set()
    starts = []
    k = 0
    while k < num_robots:
        x = np.random.normal(mu, sigma)
        y = np.random.normal(mu, sigma)
        x, y = int(x), int(y)
        if graph is not None and graph.existNode(x, y) == False:
            continue
        if (x, y) not in used:
            used.add((x, y))
            starts.append((x, y))
            k += 1
    return starts


def generate_cluster(graph, num_agents, start):
    configs = []
    open = [start]
    while len(configs) < num_agents:
        v = open[0]
        open.pop(0)
        if (v.pos.x, v.pos.y) in configs:
            continue
        configs.append((v.pos.x, v.pos.y))
        for u in v.neighbor:
            if (u.pos.x, u.pos.y) not in configs:
                open.append(u)
        np.random.shuffle(open)
    if len(configs) != len(set(configs)):
        print(configs)
    assert(len(configs) == num_agents)
    np.random.shuffle(configs)
    assert(len(configs) == len(set(configs)))
    return configs


def generate_lak103d_json():
    num_agents = list(range(10, 101, 10))
    num_cases = 20
    graph = Grids.Grid("./instances/lak103d.map")
    for n in num_agents:
        for k in range(num_cases):
            nodes = graph.getV()
            np.random.shuffle(nodes)
            starts = generate_cluster(graph, n, nodes[0])
            goals = generate_cluster(graph, n, nodes[1])
            name = "./instances/lak103d/agents"+str(n)+"_"+str(k)+".json"
            data_dict = dict()
            data_dict["starts"] = starts
            data_dict["goals"] = goals
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_60x40_json():
    graph = Grids.Grid("./map/lak103f.map")
    node_size = graph.getNodesSize()
    num_agents = np.arange(0.1, 0.31, 0.02)
    num_agents = [int(a*node_size) for a in num_agents]
    num_cases = 20
    for n in num_agents:
        for k in range(num_cases):
            starts = graph.getV()
            goals = graph.getV()
            np.random.shuffle(starts)
            np.random.shuffle(goals)
            starts = starts[0:n]
            goals = goals[0:n]
            starts = [(s.pos.x, s.pos.y)for s in starts]
            goals = [(g.pos.x, g.pos.y) for g in goals]

            name = "./instances/lak103f_random/agents" + \
                str(n)+"_"+str(k)+".json"
            data_dict = dict()
            data_dict["starts"] = starts
            data_dict["goals"] = goals
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_json_random_third_no_obs():
    grid_size = [15, 30, 45, 60, 75]
    num_cases = 20

    for m in grid_size:
        num_agents = int(m*m/3)
        nodes = [(x, y) for x in range(m) for y in range(m)]
        for k in range(num_cases):
            data_dict = dict()
            np.random.shuffle(nodes)
            starts = nodes[0:num_agents].copy()
            np.random.shuffle(nodes)
            goals = nodes[0:num_agents].copy()
            name = "./instances/third_density/agents" + \
                str(num_agents)+"_"+str(k)+".json"

            data_dict["starts"] = starts
            data_dict["goals"] = goals
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_json_random_half_density_no_obs():
    grid_size = [6, 12, 18, 24, 30, 36]
    num_cases = 50

    for m in grid_size:
        num_agents = int(m*m/2)
        nodes = [(x, y) for x in range(m) for y in range(m)]
        for k in range(num_cases):
            data_dict = dict()
            np.random.shuffle(nodes)
            starts = nodes[0:num_agents].copy()
            np.random.shuffle(nodes)
            goals = nodes[0:num_agents].copy()
            name = "./instances/half_dense/agents" + \
                str(num_agents)+"_"+str(k)+".json"

            data_dict["starts"] = starts
            data_dict["goals"] = goals
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_json_random_30x30():
    m = 30
    num_agents = np.arange(0.5, 0.71, 0.02)
    num_agents = [int(a*m*m) for a in num_agents]
    num_cases = 20

    for num in num_agents:
        nodes = [(x, y) for x in range(m) for y in range(m)]
        for k in range(num_cases):
            data_dict = dict()
            np.random.shuffle(nodes)
            starts = nodes[0:num].copy()
            np.random.shuffle(nodes)
            goals = nodes[0:num].copy()
            name = "./instances/30x30/agents"+str(num)+"_"+str(k)+".json"

            data_dict["starts"] = starts
            data_dict["goals"] = goals
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_random_instance(graph, num_agents):
    starts = graph.getV()
    goals = graph.getV()
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    return starts[:num_agents], goals[:num_agents]


def manhattan_dist(n1, n2):
    return abs(n1[0]-n2[0])+abs(n1[1]-n2[1])


def umapf_lb(starts, goals):
    cost_matrix = [[start.pos.manhattanDist(
        goal.pos) for start in starts] for goal in goals]
    # print(cost_matrix)
    row_ind, col_ind = scipy.optimize.linear_sum_assignment(cost_matrix)
    # row_ind,col_ind=lbap.labp_solve(cost_matrix)
    # row_ind,col_ind,c_star=lbap.labp_solve(cost_matrix)
    makespan = 0
    for i, j in zip(row_ind, col_ind):
        makespan = max(starts[i].pos.manhattanDist(goals[j].pos), makespan)
    # print("makespan=",makespan)
    return makespan


def left_lower_dense(graph, num_agents):
    W = int(np.sqrt(num_agents))
    starts = [graph.getNode(x, y) for x in range(0, W) for y in range(0, W)]
    goals = starts.copy()
    # tmp_starts=[graph.getNode(x,y) for x in range(0,2*W,2) for y in range(0,2*W,2)]
    # tmp_goals=tmp_starts.copy()
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    return starts, goals


def center_corner_dense(graph, num_agents):
    W = int(np.sqrt(num_agents))
    starts = [(x, y) for x in range(0, W) for y in range(0, W)]
    goals = starts.copy()
    xmax = graph.getWidth()
    ymax = graph.getHeight()
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    starts = [(s[0]+int(xmax/2-W/2), s[1]+int(ymax/2-W/2)) for s in starts]
    goals = [(g[0]+int(xmax/2-W/2), g[1]+int(ymax/2-W/2)) for g in goals]
    starts = [graph.getNode(x, y) for (x, y) in starts]
    goals = [graph.getNode(x, y) for (x, y) in goals]
    return starts, goals


def generate_group_change(graph, num_agents):
    # graph 60x60
    W = int(np.sqrt(num_agents/2))
    # starts1=[(x+10,y+20) for x in range(0,W) for y in range(0,W)]
    # starts2=[(x+40,y+20) for x in range(0,W) for y in range(0,W)]
    tmp_starts1 = [graph.getNode(x+10, y+20)
                   for x in range(0, W) for y in range(0, W)]
    tmp_starts2 = [graph.getNode(x+40, y+20)
                   for x in range(0, W) for y in range(0, W)]
    tmp_goals1 = tmp_starts1.copy()
    tmp_goals2 = tmp_starts2.copy()
    np.random.shuffle(tmp_starts1)
    np.random.shuffle(tmp_starts2)
    np.random.shuffle(tmp_goals1)
    np.random.shuffle(tmp_goals2)
    starts = tmp_starts1+tmp_starts2
    goals = tmp_goals2+tmp_goals1
    # print(len(tmp_starts1),len(tmp_starts2))
    assert(len(starts) == num_agents)
    return starts, goals


def load_instance_json(graph, agents, index, pType="third_density"):
    # file_name="./instances/half_dense/agents"+str(agents)+"_"+str(index)+".json"
    file_name = "./instances/"+pType+"/agents" + \
        str(agents)+"_"+str(index)+".json"
    # file_name="./instances/corner_dense/agents"+str(agents)+"_"+str(index)+".json"
    with open(file_name, "r") as f:
        data_dict = json.load(f)
        starts = data_dict['starts']
        goals = data_dict["goals"]
        starts = [graph.getNode(s[0], s[1]) for s in starts]
        goals = [graph.getNode(g[0], g[1]) for g in goals]
    return starts, goals


def expansion(graph, num_agents):
    W = int(np.sqrt(num_agents/2))
    # ratio=1.5
    # starts1=[(x+10,y+20) for x in range(0,W) for y in range(0,W)]
    # starts2=[(x+40,y+20) for x in range(0,W) for y in range(0,W)]
    tmp_starts1 = [graph.getNode(2*x+10, 2*y+20)
                   for x in range(0, W) for y in range(0, W)]
    tmp_starts2 = [graph.getNode(2*x+40, 2*y+20)
                   for x in range(0, W) for y in range(0, W)]
    tmp_goals1 = tmp_starts1.copy()
    tmp_goals2 = tmp_starts2.copy()
    np.random.shuffle(tmp_starts1)
    np.random.shuffle(tmp_starts2)
    np.random.shuffle(tmp_goals1)
    np.random.shuffle(tmp_goals2)
    starts = tmp_starts1+tmp_starts2
    goals = tmp_goals2+tmp_goals1
    # print(len(tmp_starts1),len(tmp_starts2),num_agents)
    assert(len(starts) == num_agents)
    return starts, goals


def benchmarkMRPP(graph, function_generate_starts_goals, num_agents: list, num_cases=20, file_name=None, function_arg=None, pType="lak103d"):
    mkpn_data = []
    soc_data = []
    rate_data = []
    comp_time_data = []
    nodes_data = []
    agents_data = []
    itemList = ['num_agents', 'mkpn', 'soc',
                'runtime', 'success_rate', 'nodes_expanded']
    for agent in num_agents:
        mkpn_sum = 0
        soc_sum = 0
        rate_sum = 0
        count = 0
        time_sum = 0
        nodes_sum = 0
        # if graph is None :
        #     m=int(np.sqrt(agent*3))
        #     print("grid size=",m)
        #     graph=Grids.Grid(m,m,[])

        for k in range(num_cases):
            # print(agent,k)
            if function_arg is None:
                starts, goals = function_generate_starts_goals(graph, agent)
            else:
                starts, goals = function_generate_starts_goals(
                    graph, agent, k, pType)
            # tmp_starts,tmp_goals=expansion(graph,agent)
            t3=time.time()
            tmp_starts=MAPF.greedy(graph,starts,0.6)
            tmp_goals=MAPF.greedy(graph,goals,0.6)
            mkpn1=umapf_lb(starts,tmp_starts)
            mkpn2=umapf_lb(goals,tmp_goals)
            t4=time.time()
            starts=tmp_starts
            goals=tmp_goals
            solver=MAPF.ECBS(graph,starts,goals)
            # assert(len(tmp_starts)==len(starts))
            # assert(len(tmp_goals)==len(goals))

            
        

            # solver=MAPF.CBDFS(graph,starts,goals)
            # solver=MAPF.DDM(graph,starts,goals)
            # solver = MAPF.DDECBS(graph, starts, goals)
            # print(len(starts),len(goals))
            # solver=MAPF.DDECBS(graph,starts,goals)
            
            t1 = time.time()
            solver.solve()
            t2 = time.time()
            time_sum += t2-t1+t4-t3
            solution = solver.getSolution()
            if solution.size() > 1 and (t2-t1) < 60 and solver.getLowerBoundMakespan() != 0:
                mkpn_lb = solver.getLowerBoundMakespan()
                mkpn_sum+=(solution.getMakespan()+mkpn1+mkpn2)/mkpn_lb
                # mkpn_sum += (solution.getMakespan())/mkpn_lb
                soc_lb = solver.getLowerBoundSOC()
                # soc_sum += (solution.getSOC())/soc_lb
                soc_sum+=(solution.getSOC()+(mkpn1+mkpn2)*agent)/soc_lb
                nodes_sum += solver.getNumExpansions()
                rate_sum += 1
                count += 1
                # print(mkpn_lb,solution.getMakespan(),soc_lb,solution.getSOC())
                # print(mkpn_sum,soc_sum,rate_sum)
            else:
                if solver.getLowerBoundMakespan() == 0:
                    rate_sum += 1
                # mkpn_lb=solver.getLowerBoundMakespan()
                # soc_lb=solver.getLowerBoundSOC()
                # if soc_lb==0:
                #     soc_sum+=1
                #     mkpn_sum+=1
                # else:
                #     soc_sum+=256*agent/soc_lb
                #     mkpn_sum+=256/mkpn_lb
                nodes_sum += solver.getNumExpansions()
            # exit(0)
        print("agents=", agent, "mkpn=", mkpn_sum/num_cases, "soc=", soc_sum/num_cases,
              "rate=", rate_sum/num_cases, "nodes_expanded=", nodes_sum/num_cases)
        if count != 0:
            mkpn_data.append(mkpn_sum/count)
            soc_data.append(soc_sum/count)
        else:
            mkpn_data.append(np.nan)
            soc_data.append(np.nan)
        comp_time_data.append(time_sum/num_cases)
        rate_data.append(rate_sum/num_cases)
        nodes_data.append(nodes_sum/num_cases)
        agents_data.append(agent)
        dataList = [agents_data, mkpn_data, soc_data,
                    comp_time_data, rate_data, nodes_data]
        if file_name is not None:
            write_multiple_csv(file_name, itemList, dataList)
        # del graph
        # graph=None


def generate_corner_dense_json():
    num_agents = [4, 9, 16, 25, 36, 49, 64, 81]
    num_cases = 50
    graph = Grids.Grid(20, 20, [])
    for k in range(num_cases):
        for agent in num_agents:
            starts, goals = left_lower_dense(graph, agent)
            data_dict = dict()
            name = "./instances/corner_dense/agents" + \
                str(agent)+"_"+str(k)+".json"

            data_dict["starts"] = [(start.pos.x, start.pos.y)
                                   for start in starts]
            data_dict["goals"] = [(goal.pos.x, goal.pos.y) for goal in goals]
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_center_dense_json():
    num_agents = [m**2 for m in range(2, 21, 2)]
    num_cases = 20
    graph = Grids.Grid(60, 60, [])
    for k in range(num_cases):
        for agent in num_agents:
            starts, goals = center_corner_dense(graph, agent)
            data_dict = dict()
            name = "./instances/center_dense/agents" + \
                str(agent)+"_"+str(k)+".json"

            data_dict["starts"] = [(start.pos.x, start.pos.y)
                                   for start in starts]
            data_dict["goals"] = [(goal.pos.x, goal.pos.y) for goal in goals]
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def group_change_json():
    num_agents = [4, 9, 16, 25, 36, 49, 64, 81]
    num_cases = 20
    with open("./instances/exchange_obs/graph_exchange.json", "r") as f:
        graph = json.load(f)
        xmax = graph["xmax"]
        ymax = graph["ymax"]
        obstacles = graph["obstacles"]
    print(xmax, ymax, obstacles)
    graph = Grids.Grid(xmax, ymax, obstacles)
    # graph=Grids.Grid(60,60,[])
    for k in range(num_cases):
        for agent in num_agents:
            starts, goals = generate_group_change(graph, 2*agent)
            data_dict = dict()
            name = "./instances/exchange_obs/agents" + \
                str(2*agent)+"_"+str(k)+".json"
            data_dict["starts"] = [(start.pos.x, start.pos.y)
                                   for start in starts]
            data_dict["goals"] = [(goal.pos.x, goal.pos.y) for goal in goals]
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_ring_json():
    for m in range(30, 91, 30):
        for k in range(20):
            starts = []
            goals = []
            data_dict = dict()
            for i in range(1, m, 3):
                low_x = i
                high_x = m-1-i
                low_y = i
                high_y = m-1-i
                start_set = set()
                start_set.update([(i, low_y) for i in range(low_x, high_x+1)])
                start_set.update([(i, high_y) for i in range(low_x, high_x+1)])
                start_set.update([(low_x, i) for i in range(low_y, high_y+1)])
                start_set.update([(high_x, i) for i in range(low_y, high_y+1)])
                starts.extend(list(start_set))
            # starts.extend([(6,7),(8,7)])
            for start in starts:

                goal = (m-1-start[0], m-1-start[1])
                goals.append(goal)
            name = "./instances/ring/agents"+str(int(m*m/3))+"_"+str(k)+".json"
            data_dict["starts"] = starts
            data_dict["goals"] = goals
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_outer_ring_json():
    for m in range(30, 181, 30):

        for k in range(10):
            starts = []
            goals = []
            data_dict = dict()
            for i in [1]:

                low_x = i
                high_x = m-1-i
                low_y = i
                high_y = m-1-i
                start_set = set()
                start_set.update([(i, low_y) for i in range(low_x, high_x+1)])
                start_set.update([(i, high_y) for i in range(low_x, high_x+1)])
                start_set.update([(low_x, i) for i in range(low_y, high_y+1)])
                start_set.update([(high_x, i) for i in range(low_y, high_y+1)])
                starts.extend(list(start_set))
            # starts.extend([(6,7),(8,7)])
            nodes = [(x, y) for x in range(m) for y in range(m)]
            np.random.shuffle(nodes)
            goals = nodes[0:len(starts)]
            # for start in starts:

            #     goal=(m-1-start[0],m-1-start[1])
            #     goals.append(goal)
            name = "./instances/ring_random/agents"+str(m)+"_"+str(k)+".json"
            data_dict["starts"] = starts
            data_dict["goals"] = goals
            print(len(starts), len(goals), m)
            # print(starts,goals)
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def test_ddm():
    graph = Grids.Grid(10, 10, [])
    starts, goals = generate_random_instance(graph, 2)
    ddm_solver = MAPF.DDM(graph, starts, goals)
    ddm_solver.solve()
    solution = ddm_solver.getSolution()


def test_random():
    graph = Grids.Grid(30, 30, [])
    num_agents = list(range(50, 310, 50))
    benchmarkMRPP(graph, generate_random_instance, num_agents, 20,
                  "./data/HighLevelRandomPrioritizeConflcit.csv")


def test_corner():
    graph = Grids.Grid(20, 20, [])
    num_agents = [4, 9, 16, 25, 36, 49]
    # benchmarkMRPP(graph,left_lower_dense,num_agents,20,"./data/CornerDenseControl.csv")
    benchmarkMRPP(graph, load_instance_json, num_agents, 20,
                  "./data/ddecbsCorner.csv", function_arg=1)


def test_center_corner():
    graph = Grids.Grid(60, 60, [])
    num_agents = [m**2 for m in range(2, 17, 2)]
    benchmarkMRPP(graph, load_instance_json, num_agents, 20,
                  "./data/ddecbs_center_w2.csv", function_arg=1, pType="center_dense")


def test_half_dense():
    m = [6, 12, 18, 24]
    num_agents = [int(x**2/2) for x in m]
    benchmarkMRPP(None, load_instance_json, num_agents, 20,
                  "./data/weightedConflicthalfECBSw2.csv", function_arg=1)


def test_third_dense():
    m = [6, 12, 18, 24, 30, 36]
    num_agents = [int(x**2/3) for x in m]
    benchmarkMRPP(None, load_instance_json, num_agents, 20,
                  "./data/thirdDenseECBSw2.csv", function_arg=1, pType="third_density")


def test_lak103d():
    graph = Grids.Grid("./instances/lak103d.map")
    num_agents = list(range(10, 81, 10))
    benchmarkMRPP(graph, load_instance_json, num_agents, 20,
                "./data/lak103dDDECBS.csv", function_arg=1, pType="lak103d")


def test_30x30():
    m = 20
    graph = Grids.Grid(m, m, [])
    # num_agents=list(range(150,301,10))
    num_agents=[4,9,16,25,36,49,64,81]
    # num_agents = np.arange(0.5, 0.71, 0.020)
    # num_agents = [int(a*m*m) for a in num_agents]
    benchmarkMRPP(graph, load_instance_json, num_agents, 20,
                "./data/iros/corner_dense_secbs06.csv", function_arg=1, pType="corner_dense")


def test_group_change():
    graph = Grids.Grid(60, 60, [])
    num_agents = [2*4, 2*9, 2*16, 2*25, 2*36, 2*49, 2*64, 2*81]
    benchmarkMRPP(graph, load_instance_json, num_agents, 20,
                  "./data/TBgroupECBSw2.csv", function_arg=1)


def test_obs_group_change():
    num_agents = [2*4, 2*9, 2*16, 2*25, 2*36, 2*49, 2*64, 2*81]
    with open("./instances/exchange_obs/graph_exchange.json", "r") as f:
        graph = json.load(f)
        xmax = graph["xmax"]
        ymax = graph["ymax"]
        obstacles = graph["obstacles"]
    graph = Grids.Grid(xmax, ymax, obstacles)
    benchmarkMRPP(graph, load_instance_json, num_agents, 20,
                  "./data/groupChangesOBS.csv", function_arg=1)


def visualizer_lak103d():
    # graph=Grids.Grid("./map/20x20.map")
    # graph = Grids.Grid(60, 60, [])
    graph=Grids.Grid("./map/warehouse.map")
    num_agents=211
    # num_agents = 260
    starts, goals = load_instance_json(
        graph, num_agents, 1, pType="warehouse")

    # exit(0)

    solver = MAPF.ECBS(graph, starts, goals)
    # solver=MAPF.CBDFS(graph,starts,goals)
    # solver=MAPF.DDM(graph,starts,goals)
    # solver=MAPF.DDECBS(graph,starts,goals)
    solver.solve()
    solution = solver.getSolution()
    paths = [[] for i in range(num_agents)]
    for i in range(num_agents):
        pi = solution.getPath(i)
        for t in range(len(pi)):
            paths[i].append((pi[t].pos.x, pi[t].pos.y))
    data_dict = dict()
    data_dict["xmax"] = graph.getHeight()
    data_dict["ymax"] = graph.getWidth()
    obstacles = []
    for x in range(graph.getHeight()):
        for y in range(graph.getWidth()):
            if graph.existNode(x, y) == False:
                obstacles.append((x, y))

    data_dict["obstacles"]=obstacles
    data_dict["paths"]=paths
    # file_name="gauss_lak103f_demo_ddm.json"
    # with open(file_name,"w") as f:
    #     json.dump(data_dict,f)


def visualizer_demo():
    # with open("./instances/exchange_obs/graph_exchange.json","r") as f:
    #     graph=json.load(f)
    #     xmax=graph["xmax"]
    #     ymax=graph["ymax"]
    #     obstacles=graph["obstacles"]
    xmax = 20
    ymax = 20
    obstacles = []
    graph = Grids.Grid(xmax, ymax, obstacles)
    num_agents = 49
    starts, goals = load_instance_json(graph, num_agents, 1, pType="corner_dense")
    t0 = time.time()
    # solver=MAPF.ECBS(graph,starts,goals)

    # solver=MAPF.CBDFS(graph,starts,goals)
    # solver = MAPF.DDECBS(graph, starts, goals)
    solver=MAPF.DDM(graph,starts,goals)
    solver.solve()
    t1 = time.time()
    num_agents = len(starts)
    solution = solver.getSolution()
    paths = [[] for i in range(num_agents)]
    for i in range(num_agents):
        pi = solution.getPath(i)
        for t in range(len(pi)):
            paths[i].append((pi[t].pos.x, pi[t].pos.y))

    data_dict = dict()
    data_dict["xmax"] = xmax
    data_dict["ymax"] = ymax
    data_dict["obstacles"] = obstacles
    data_dict["paths"] = paths
    data_dict["makespan"] = solution.getMakespan() / solver.getLowerBoundMakespan()
    data_dict["soc"] = solution.getSOC()/solver.getLowerBoundSOC()
    data_dict["runtime"] = t1-t0
    file_name = "corner_dense_demo_ddm.json"
    with open(file_name, "w") as f:
        json.dump(data_dict, f)


def test_json():
    data_dict = dict()
    data_dict["data"] = [np.nan, np.nan, np.nan]
    with open("./test.json", "w") as f:
        json.dump(data_dict, f)

    with open("./test.json", "r") as f:
        test_data = json.load(f)
    # print(math.isnan(test_data['data'][0]))


def test_umpaf_corner_dense():
    graph = Grids.Grid(20, 20, [])
    num_agents = [4, 9, 16, 25, 36, 49, 64, 81]
    # benchmarkMRPP(graph,left_lower_dense,num_agents,20,"./data/CornerDenseControl.csv")
    benchmarkMRPP(graph, load_instance_json, num_agents, 20,
                  "./data/umapfEcbsCorner0D3.csv", function_arg=1, pType="corner_dense")


def test_umpaf_center_dense():
    graph = Grids.Grid(60, 60, [])
    num_agents = [m**2 for m in range(2, 17, 2)]
    # benchmarkMRPP(graph,left_lower_dense,num_agents,20,"./data/CornerDenseControl.csv")
    benchmarkMRPP(graph, load_instance_json, num_agents, 20,
                  "./data/ddm_center.csv", function_arg=1, pType="center_dense")


def test_jj():
    graph = Grids.Grid("./map/lak103f.map")
    # graph=Grids.Grid(20,20,[])
    # print("graph loaded", graph.getWidth(), graph.getHeight())
    node_size = graph.getNodesSize()
    # num_agents=[0.05,0.1,0.15,0.2,0.25]
    # num_agents=np.arange(0.4,0.5,0.01)

    num_agents = np.arange(0.1, 0.31, 0.02)
    # num_agents=list(range(200,281,8))
    num_agents = [int(a*node_size) for a in num_agents]
    benchmarkMRPP(graph, load_instance_json, num_agents, 20,
                "./data/iros/lak103f_poc20.csv", function_arg=1, pType="lak103f_random")


def test_gauss():
    # graph = Grids.Grid("./map/lak103f.map")
    graph=Grids.Grid(60,60,[])
    print("graph loaded", graph.getWidth(), graph.getHeight())
    node_size = graph.getNodesSize()
    
    # num_agents=np.arange(0.4,0.5,0.01)
    # num_agents=[0.1,0.15,0.2,0.25]
    # num_agents = np.arange(0.1, 0.31, 0.02)
    num_agents=list(range(200,281,8))
    # num_agents = [int(a*node_size) for a in num_agents]
    benchmarkMRPP(graph, load_instance_json, num_agents, 10,
                "./data/iros/20x20_ddecbs_bounded.csv", function_arg=1, pType="20x20")


def test_grids():
    position = Grids.Pos(1, 2)
    position.println()
    graph = Grids.Grid("./map/arena.map")
    starts = []
    goals = []
    for i in range(5):
        starts.append(graph.getNode(i))
        goals.append(graph.getNode(i))


if __name__ == "__main__":
    # test_dense_demo()
    # group_change_json()
    # test_obs_group_change()
    # visualizer_demo()
    # generate_60x40_json()
    # generate_json_random_30x30()
    # generate_center_dense_json()
    # test_lak103d(
    # generate_json_random_third_no_obs()
    # visualizer_lak103d()
    # test_json()
    # test_group_change()
    # generate_lak103d_json()
    # generate_graph_group_exchange()
    # test_random()
    # generate_json_random_30x30()
    # test_corner()
    # generate_ring_json()
    # generate_outer_ring_json()
    # test_umpaf_corner_dense()
    # test_umpaf_center_dense()
    # generate_json_random_third_no_obs()
    # test_third_dense()
    test_30x30()
    # pass
    # generate_json_random_half_density_no_obs()
    # generate_corner_dense_json()
    # test_half_dense()
    # test_ddm()
    # test_center_corner()
    # test_jj()
    # generate_60x40_json()
