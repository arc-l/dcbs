
import Grids
import MAPF
import time
from common import *
import scipy
import lbap
import math
import re
from debugger import *
import matplotlib.pyplot as plt
from MCP import *


# plt.rcParams['font.family'] = "Times New Roman"
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = "serif"
LINE_WIDTH=3
MARKER_SIZE=3
plt.rcParams['font.serif'] = "Times"
plt.rcParams.update(
        {
            'xtick.labelsize': 22,
            'ytick.labelsize': 22,
            "legend.borderpad": 0.2,  ## border whitespace
            "legend.labelspacing": 0.2,  ## the vertical space between the legend entries
            "legend.handlelength": 1,  ## the length of the legend lines
            "legend.handleheight": 0.7,  ## the height of the legend handle
            "legend.handletextpad": 0.2,  ## the space between the legend line and legend text
            "legend.borderaxespad": 0.5,  ## the border between the axes and legend edge
            "legend.columnspacing": 1.0,  ## column separation
            "legend.framealpha": 0.8
        }
    )
font1 = {'family' : 'Serif',
'weight' : 'normal',
'size'   : 20,
}


def match_txt():
    plt.figure(figsize=(15,4))
    plt.subplot(1,2,1)
    bar_width=0.3

    x_data=[1,2,3,4,5,6,7,8,9,10]
    x_index=np.arange(10)

    iters_dfs=[2500,2400,1700,8000,2000,2300,2000,5900,1800,5000]
    iters_soc=[12000,12000,2200,8000,5950,1700,1000,8000,2000,8000]

    # x1,x2,x3 = [np.random.randn(n) for n in [10000, 5000, 2000]]

 

    rects=plt.bar(x_index,iters_dfs,width=bar_width,alpha=0.8,label="dfs", edgecolor='black')
    rects1=plt.bar(x_index+bar_width,iters_soc,width=bar_width,alpha=0.8,label="soc", edgecolor='black')
    plt.ylabel("iterations of stagnation",fontsize=28)
    plt.xlabel("instance index",fontsize=28)
    plt.xticks(x_index+bar_width/2,x_data)
    plt.legend(prop=font1)
    plt.subplot(1,2,2)
    pattern=re.compile(r"conflicts: \d+")
    time1=[]
    time2=[]
    k=0
    conflicts_data=[]
    conflicts_data2=[]
    zero_data=[]
    for i,line  in enumerate(open("./1.txt")):
        for match in re.finditer(pattern,line):
            conflicts=int(match.group()[10:])
            print("num conflicts=",conflicts)
            time1.append(k)
            k+=1
            conflicts_data.append(conflicts)
            zero_data.append(0)
    k=0
    for i,line  in enumerate(open("./2.txt")):
        for match in re.finditer(pattern,line):
            conflicts=int(match.group()[10:])
            conflicts_data2.append(conflicts)
            time2.append(k)
            k+=1
            
            print("num conflicts=",conflicts)
    while len(conflicts_data)<len(conflicts_data2):
        conflicts_data.append(conflicts_data[-1])
        time1.append(time1[-1]+1)
        zero_data.append(0)
    l1,=plt.plot(time1,conflicts_data,linewidth=LINE_WIDTH)
    l2,=plt.plot(time2,conflicts_data2,linewidth=LINE_WIDTH)
    plt.legend(handles=[l1,l2],labels=["dfs","soc"],prop=font1,loc="center",bbox_to_anchor=(0.4, 0.2))
    plt.plot(time1,zero_data,linestyle='--')
    plt.xlabel("iteration",fontsize=28)
    plt.ylabel("num of conflicts",fontsize=28)
    plt.yscale("symlog")
    plt.savefig("ecbs_stuck.pdf",bbox_inches="tight",pad_inches=0.05)
    plt.show()
            # print('Found on line %s: %s' % (i+1, match.group()))



def main():
    xmax=60
    ymax=60
    obstacles=[]
    graph=Grids.Grid(xmax,ymax,obstacles)
    num_agents=260
    starts,goals=load_instance_json(graph,num_agents,2,pType="60x60gauss_random")
    # tmp_starts=MAPF.greedy(graph,starts,0.4)
    tmp_starts=starts
    tmp_goals=MAPF.greedy(graph,goals,0.4)
    solver=MAPF.FlowBasedUMRPP(graph,starts,tmp_starts)
    solution_s=solver.solveWeighted()
    num_agents=len(starts)   
    del solver
    solver=MAPF.FlowBasedUMRPP(graph,goals,tmp_goals)
    solution_g=solver.solveWeighted()
    new_goals=solution_g.last()
    solution_g=solution_g.reverse()
    new_starts=solution_s.last()
    new_starts=starts
    del solver
    print("start ecbs")
    solver=MAPF.ECBS(graph,new_starts,new_goals)
    solver.solve()
    solution_ecbs=solver.getSolution()
    paths=[[] for i in range(num_agents)]
    for i in range(num_agents):
        # pi=solution_s.getPath(i)
        pe=solution_ecbs.getPath(i)
        pg=solution_g.getPath(i)
        # for t in range(len(pi)):
        #     paths[i].append((pi[t].pos.x,pi[t].pos.y))
        for t in range(len(pe)):
            paths[i].append((pe[t].pos.x,pe[t].pos.y))
        for t in range(len(pg)):
            paths[i].append((pg[t].pos.x,pg[t].pos.y))
    # mcpSolver=MCP(paths)
    # mcpSolver.mcpSolveCycle()
        
    data_dict=dict()
    data_dict["xmax"]=xmax
    data_dict["ymax"]=ymax
    data_dict["obstacles"]=obstacles
    # data_dict["paths"]=mcpSolver.plans
    data_dict["paths"]=paths
    # data_dict["makespan"]=solution.getMakespan()/solver.getLowerBoundMakespan()
    # data_dict["soc"]=solution.getSOC()/solver.getLowerBoundSOC()

    file_name="gauss_random_demo_secbs.json"
    with open(file_name,"w") as f:
        json.dump(data_dict,f)



def generate_json_gauss60x60():
    num_agents=list(range(100,301,10))
    num_cases=20
    graph=Grids.Grid(60,60,[])
    for n in num_agents:
        for k in range(num_cases):
            # starts=generate_gauss(n,graph)
            nodes=[(x,y) for x in range(60) for y in range(60)]
            np.random.shuffle(nodes)
            starts=nodes[:n]
            goals=generate_gauss(n,graph)
            name="./instances/60x60gauss_random/agents"+str(n)+"_"+str(k)+".json"
            data_dict=dict()
            data_dict["starts"]=starts
            data_dict["goals"]=goals
            with open(name, "w") as f:
                print(name)
                json.dump(data_dict,f)




def test_converge(graph, function_generate_starts_goals, num_agents: list, num_cases=20, file_name=None, function_arg=None, pType="lak103d"):
    mkpn_data = []
    soc_data = []
    rate_data = []
    iter_data=[]
    comp_time_data = []
    nodes_data = []
    agents_data = []
    itemList = ['num_agents', 'mkpn', 'soc',
                'runtime', 'success_rate', 'NOC_stagnation']
    for agent in num_agents:
        mkpn_sum = 0
        soc_sum = 0
        rate_sum = 0
        count = 0
        time_sum = 0
        nodes_sum = 0
        noc_sum=0
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
            # t3=time.time()
            # tmp_starts=MAPF.greedy(graph,starts,0.6)
            # tmp_goals=MAPF.greedy(graph,goals,0.6)
            # mkpn1=umapf_lb(starts,tmp_starts)
            # mkpn2=umapf_lb(goals,tmp_goals)
            # t4=time.time()
            # starts=tmp_starts
            # goals=tmp_goals
            # solver=MAPF.ECBS(graph,starts,goals)
            # assert(len(tmp_starts)==len(starts))
            # assert(len(tmp_goals)==len(goals))

            
        

            # solver=MAPF.CBDFS(graph,starts,goals)
            # solver=MAPF.DDM(graph,starts,goals)
            solver = MAPF.DDECBS(graph, starts, goals)
            # print(len(starts),len(goals))
            # solver=MAPF.DDECBS(graph,starts,goals)
            
            t1 = time.time()
            solver.solve()
            t2 = time.time()
            time_sum += t2-t1#+t4-t3
            solution = solver.getSolution()
            if solution.size() > 1 and (t2-t1) < 60 and solver.getLowerBoundMakespan() != 0:
                mkpn_lb = solver.getLowerBoundMakespan()
                # mkpn_sum+=(solution.getMakespan()+mkpn1+mkpn2)/mkpn_lb
                mkpn_sum += (solution.getMakespan())/mkpn_lb
                soc_lb = solver.getLowerBoundSOC()
                soc_sum += (solution.getSOC())/soc_lb
                # soc_sum+=(solution.getSOC()+(mkpn1+mkpn2)*agent)/soc_lb
                nodes_sum += solver.getNumExpansions()
                rate_sum += 1
                count += 1
                noc_sum+=solver.get_stagnation()
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
        iter_data.append(noc_sum/num_cases)
        dataList = [agents_data, mkpn_data, soc_data,
                    comp_time_data, rate_data, iter_data]
        if file_name is not None:
            write_multiple_csv(file_name, itemList, dataList)
    


def test_noc():
    m = 20
    # graph = Grids.Grid(m, m, [])
    graph=Grids.Grid("./map/warehouse.map")
    num_agents=[172,177,181,185,190,194,198,203,207,211]
    
    # num_agents = np.arange(0.5, 0.71, 0.020)
    # num_agents = [int(a*m*m) for a in num_agents]
    test_converge(graph, load_instance_json, num_agents, 20,
                "./data/iros/noc_soc.csv", function_arg=1, pType="warehouse")

if __name__=="__main__":
    # main()
    match_txt()
    # kit_demo()
    # test_noc()
    # generate_json_gauss60x60()