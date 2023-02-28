import numpy as np
import  json

MAX_HORIZON=10000

class Vertex(object):
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.visiting_agents=[]
    

class MCP(object):
    def __init__(self,original_paths) -> None:
        self.original_paths=original_paths
        self.vertices=dict()
        self.plans=[[self.original_paths[i][0]] for i in range(len(self.original_paths))]
        self.location_id=dict()
        self.old_location_id=dict()
        self.v_obs=set()
        self.e_obs=set()
        # self.id_location=dict()
        self.moved=dict()
        self.cycle_check_stack=dict()

    def get_next_v(self,agent):
        if agent in self.moved:
            return self.plans[agent][-1]
        else:
            return self.original_paths[agent][0]

    def init_schedule(self):
        makespan=len(self.original_paths[0])
        num_robots=len(self.original_paths)
        for t in range(1,makespan):
            for i in range(num_robots):
                last_v=self.original_paths[i][t-1]
                v=self.original_paths[i][t]
                if v not in self.vertices:
                    self.vertices[v]=Vertex(v[0],v[1])
                vertex=self.vertices[v]
                if v!=last_v:
                    self.vertices[v].visiting_agents.append(i)


    def remove_waiting_for_one_path(self,path):
        i=0
        new_path=[]
        if len(path)==0:
            return
        last_v=path[0]
        new_path.append(last_v)
        while i<len(path):
            v=path[i]
            if v!=last_v:
                last_v=v
                new_path.append(v)
            i=i+1
        return new_path

    def remove_waiting_status(self):
        for i in range(len(self.original_paths)):
            self.original_paths[i]=self.remove_waiting_for_one_path(self.original_paths[i])
            starti=self.original_paths[i][0]
            self.location_id[self.original_paths[i][0]]=i
            self.original_paths[i].pop(0)
            # self.vertices[starti].visiting_agents.pop(0)
        # print("debug paths")
        # for i in range(len(self.original_paths)):
        #     print(i,self.original_paths[i])
        # print("========!!!!!!")
        # print()
        
    def mpcSolve(self):
        self.init_schedule()
        self.remove_waiting_status()
        # self.remove_waiting_status()
        k=0
        while self.check_arrive_goals()==False:
            self.moved.clear()
            self.v_obs.clear()
            self.old_location_id=self.location_id.copy()
            k=k+1
            # print(k,'*****************')
            if k>MAX_HORIZON:
                break
            for agent in range(len(self.original_paths)):
                self.move(agent)
            # print()
    
    def mcpSolveCycle(self):
        self.init_schedule()
        self.remove_waiting_status()
        k=0
        while self.check_arrive_goals()==False:
            self.moved.clear()
            self.cycle_check_stack.clear()
            self.v_obs.clear()
            self.old_location_id=self.location_id.copy()
            k=k+1
            if k>MAX_HORIZON:
                break
            for agent in range(len(self.original_paths)):
                self.move_with_cycle(agent)
    
    def save_plans_as_json(self,file_name):
        sol=dict()
        sol["paths"]=self.plans
        
        with open(file_name,"w") as fp:
            json.dump(sol,fp)

    def forward_agent(self,agent,curr_v,next_v):
        if self.location_id[curr_v]==agent:
            self.location_id.pop(curr_v)
        self.location_id[next_v]=agent
        self.original_paths[agent].pop(0)
        self.vertices[next_v].visiting_agents.pop(0)
        self.moved[agent]=True
        self.plans[agent].append(next_v)
        self.v_obs.add(next_v)

    def wait_agent(self,agent,curr_v):
        self.moved[agent]=False
        self.plans[agent].append(curr_v)
        self.v_obs.add(curr_v)


    def check_perpendicular(self,v1,v2,v3):
        x1,y1=v1
        x2,y2=v2
        x3,y3=v3
        if (x2-x1)*(y3-y2)-(y2-y1)*(x3-x2)!=0:
            print("perpendicular following conflicts")
            return True
        return False

    def check_arrive_goals(self):
        for p in self.original_paths:
            if len(p)!=0:
                return False
        return True

    def move(self,agent):
        # print("moving agent",agent)
        if agent in self.moved:
            return self.moved[agent]
        if len(self.original_paths[agent])==0:
            self.moved[agent]=False
            return False
        next_v=self.original_paths[agent][0]
        curr_v=self.plans[agent][-1]
        # print(agent,curr_v,next_v,self.vertices[next_v].visiting_agents)
        if agent==self.vertices[next_v].visiting_agents[0]:
            if next_v not in self.old_location_id:
                if  next_v not in self.v_obs:
                    self.forward_agent(agent,curr_v,next_v)
                    return True
                else:
                    self.wait_agent(agent,curr_v)
                    return False
            else:
                aj=self.old_location_id[next_v]
                # print(aj,agent,self.original_paths[aj])
                # if len(self.original_paths[aj])!=0:
                # print("debug",aj,agent,self.original_paths[aj],self.original_paths[agent])
                # vj=self.original_paths[aj][0]
                vj=self.get_next_v(aj)
                flag=self.move(aj)
                if flag==False:
                    self.wait_agent(agent,curr_v)
                    return False
                else:
                    # if  self.check_perpendicular(curr_v,next_v,vj)==False and next_v not in self.v_obs:
                    if  next_v not in self.v_obs:
                        self.forward_agent(agent,curr_v,next_v)
                        return True
                    else:
                        self.wait_agent(agent,curr_v)
                        return False
        else:
            print(agent,curr_v,next_v,self.vertices[next_v].visiting_agents)
            print("not the correct ordering ")
            self.wait_agent(agent,curr_v)
            return False
    
    def move_with_cycle(self,agent):
        if agent in self.moved:
            return self.moved[agent]
        if len(self.original_paths[agent])==0:
            self.moved[agent]=False
            return False
        self.cycle_check_stack[agent]=True
        next_v=self.original_paths[agent][0]
        curr_v=self.plans[agent][-1]
        # print(agent,curr_v,next_v,self.vertices[next_v].visiting_agents)
        if agent==self.vertices[next_v].visiting_agents[0]:
            if next_v not in self.old_location_id:
                if  next_v not in self.v_obs:
                    self.forward_agent(agent,curr_v,next_v)
                    return True
                else:
                    self.wait_agent(agent,curr_v)
                    return False
            else:
                aj=self.old_location_id[next_v]
                # print(aj,agent,self.original_paths[aj])
                # if len(self.original_paths[aj])!=0:
                # print("debug",aj,agent,self.original_paths[aj],self.original_paths[agent])
                # vj=self.original_paths[aj][0]
                vj=self.get_next_v(aj)
                flag=self.move(aj)
                if flag==False:
                    self.wait_agent(agent,curr_v)
                    return False
                else:
                    # if  self.check_perpendicular(curr_v,next_v,vj)==False and next_v not in self.v_obs:
                    if  next_v not in self.v_obs:
                        self.forward_agent(agent,curr_v,next_v)
                        return True
                    else:
                        self.wait_agent(agent,curr_v)
                        return False
        else:
            print(agent,curr_v,next_v,self.vertices[next_v].visiting_agents)
            print("not the correct ordering ")
            self.wait_agent(agent,curr_v)
            return False



if __name__=="__main__":
    f1=open("./ddm.json")
    ps=(json.load(f1))['paths']
    paths=[[tuple(v) for v in p] for p in ps]
    mcpSolver=MCP(paths)
    mcpSolver.mpcSolve()
    mcpSolver.save_plans_as_json("./ddm_mcp.json")




