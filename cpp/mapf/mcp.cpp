#include "mcp.hpp"




MCP::MCP(const Plan & p){
    int num_robots=p.size();
    
    Node *last_v,*v;
    next_config=p.get(0);
    plans.add(next_config);
    original_paths=std::vector<std::queue<Node*>>(num_robots,std::queue<Node*>());

    for(int t=1;t<=p.getMakespan();t++){
        for(int i=0;i<num_robots;i++){
            last_v=p.get(t-1,i);
            v=p.get(t,i);
            if(vertices.find(v)==vertices.end()){
                vertices[v]=VertexMCP(v);
            }
            if(v!=last_v){
                vertices[v].visiting_agents.push(i);
            }
        }
    }

    //remove waiting status
    for(int i=0;i<num_robots;i++){
        int t=0;
        last_v=p.get(0,i);
        while(t<p.getMakespan()){
            v=p.get(t,i);
            if(v!=last_v){
                last_v=v;
                original_paths[i].push(v);
            }
            t++;
        }
        auto start_i=plans.get(0,i);
        location_id[start_i]=i;
        original_paths[i].pop();
    }
}


Node *MCP::get_next_v(int agent){
    if(moved.find(agent)!=moved.end()) return plans.last(agent);
    else return original_paths[agent].front();
}




void MCP::mcpSolveCycle(){
    int k=0;
    int num_robots=original_paths.size();
    next_config=Config(num_robots);

    while(check_arrive_goals()==false){
        moved.clear();
        cycle_check_stack.clear();
        next_config=plans.last();
        vertexObstacles.clear();
        old_location_id=location_id;
        k++;
        for(int agent=0;agent<num_robots;agent++){
            move_with_cycle(agent);
        }
    }
}


void MCP::forward_agent(int agent,Node*curr_v,Node* next_v){
    if(location_id[curr_v]==agent){
        location_id.erase(curr_v);
    }
    location_id[next_v]=agent;
    original_paths[agent].pop();
    vertices[next_v].visiting_agents.pop();
    moved[agent]=true;
    next_config[agent]=next_v;
    vertexObstacles.insert(next_v);
}


void MCP::wait_agent(int agent,Node*curr_v){
    moved[agent]=false;
    next_config[agent]=curr_v;
    vertexObstacles.insert(curr_v);
}


bool MCP::check_arrive_goals(){
    for(int i=0;i<original_paths.size();i++){
        if(original_paths[i].empty()==false) return false;
    }
    return true;
}


void MCP::move_all_robots_in_cycle(int robot){
    std::set<int> visited;
    int curr=robot;
    Node *curr_v,*next_v;
    while(true){
        if(visited.find(curr)!=visited.end())break;
        visited.insert(curr);
        curr_v=next_config[curr];
        next_v=original_paths[curr].front();
        forward_agent(curr,curr_v,next_v);
        curr=old_location_id[next_v];
    }

}

bool MCP::move_with_cycle(int agent){
    bool flag;
    Node *next_v,*curr_v;
    if(moved.find(agent)!=moved.end()) return moved[agent];
    if(original_paths[agent].empty()==true){
        moved[agent]=false;
        return false;
    }
    next_v=original_paths[agent].front();
    curr_v=next_config[agent];
    if(agent==vertices[next_v].visiting_agents.front()){
        if(old_location_id.find(next_v)==old_location_id.end()){
            if(vertexObstacles.find(next_v)==vertexObstacles.end()){
                forward_agent(agent,curr_v,next_v);
                return true;
            }
            else{
                wait_agent(agent,curr_v);
                return false;
            }
        }
        else{
            if(cycle_check_stack.find(agent)!=cycle_check_stack.end()){
                move_all_robots_in_cycle(agent);
                return true;
            }
            cycle_check_stack.insert(agent);
            int aj=old_location_id[next_v];
            flag=move_with_cycle(aj);
            if(moved.find(agent)!=moved.end()){
                return moved[agent];
            }
            if(flag==false){
                wait_agent(agent,curr_v);
                return false;
            }
            else{
                if(vertexObstacles.find(next_v)==vertexObstacles.end()){
                    forward_agent(agent,curr_v,next_v);
                    return true;
                }
                else{
                    wait_agent(agent,curr_v);
                    return false;
                }
            }
        }
    }
    else{
        wait_agent(agent,curr_v);
        return false;
    }
    

}


