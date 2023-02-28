/**
 * @file flow.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-07
 * max-flow based mrpp min-makespan
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "solver.hpp"
#include <ortools/graph/max_flow.h>
#include <ortools/graph/min_cost_flow.h>


class FlowBasedUMRPP:public Solver{
public:
    // const int None=-1;
    using flowNode=std::tuple<int,int,int>;// u, t, in or out
    // const int source=-985;
    // const int goal=-211;
    FlowBasedUMRPP();
    FlowBasedUMRPP(Problem *problem);
    FlowBasedUMRPP(Grid*graph,Config starts,Config goals);
    ~FlowBasedUMRPP();

    //solve maxflow
    void run();

    //solve min-cost maxflow
    Plan solveWeighted();

private:

    void prepare(int timestep);


    void add_edge(flowNode & u,flowNode &v);
    void insert_node(int &id,flowNode &node);

    void resolveEdgeConflicts(Paths &result);
    void retrievePaths(operations_research::MaxFlow &flow,Paths &result);
    void retrievePaths(operations_research::MinCostFlow &flow,Paths &result);

    void switchPaths(int i,int j,int t,Paths &result);

    std::vector<int>startNodes;
    std::vector<int>endNodes;
    std::vector<int>capacities; //capacity is 1

    std::unordered_map<int,flowNode> id_node;
    std::map<flowNode,int> node_id;

    void evaluateLB();
    void evaluateLBfast();
    int source_id;
    int sink_id;

    int makespanLB;

};