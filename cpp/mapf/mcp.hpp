
#pragma once
#include "solver.hpp"
#include <set>
#include<map>
#include <list>

struct VertexMCP{
    Node *v;
    std::queue<int> visiting_agents;
    VertexMCP(Node * v){
        this->v=v;
    }
};

class MCP{
public:
    // using vobs=std::tuple<Node*,int>; //v,t
    // using eobs=std::tuple<Node*,Node*,int>; //u,v,t
    MCP(const Plan &p);
    void solve();
    Plan getSolution(){
        return plans;
    }


private:
    std::vector<std::queue<Node*>> original_paths;
    std::set<Node*> vertexObstacles;

    std::map<int,bool> moved;
    std::set<int> cycle_check_stack;
    std::map<Node*,int> location_id;
    std::map<Node*,int> old_location_id;
    std::map<Node*,VertexMCP> vertices;

    Plan plans;
    Config next_config;


private:
    

    Node * get_next_v(int agent);




    void mcpSolveCycle();

    void forward_agent(int agent, Node* curr_v,Node* next_v);

    void wait_agent(int agent, Node* curr_v);

    bool check_arrive_goals();


    void move_all_robots_in_cycle(int robot);

    bool move_with_cycle(int robot);






};