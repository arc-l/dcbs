/**
 * @file flow.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "maxflow.hpp"
#include <set>
// #define DEBUG true
#define IN 0
#define OUT 1

namespace OR=operations_research;


/**
 * @brief Construct a new Flow MAPF:: Flow M A P F object
 * 
 * @param starts 
 * @param goals 
 * @param graph 
 */
FlowBasedUMRPP::FlowBasedUMRPP(Grid*graph,Config starts,Config goals):Solver(graph,starts,goals){

}

/// @brief 
/// @param p 
FlowBasedUMRPP::FlowBasedUMRPP(Problem* p):Solver(p){
    
}




FlowBasedUMRPP::~FlowBasedUMRPP(){
    
}


void FlowBasedUMRPP::run(){
    evaluateLB();
    int timeStep=makespanLB;
    bool solved=true;
    Paths result;
    while(true){
        // std::cout<<"preparing model for timestep="<<timeStep<<std::endl;
        prepare(timeStep);
        // std::cout<<"prepared model for timestep="<<timeStep<<std::endl;
        OR::StarGraph gs(node_id.size(), startNodes.size());
        OR::MaxFlow max_flow(&gs,source_id,sink_id);
        for(int i=0;i<startNodes.size();i++){
            OR::ArcIndex arc=gs.AddArc(startNodes[i],endNodes[i]);
            max_flow.SetArcCapacity(arc,1); //unit capacity
        }
        max_flow.Solve();
        OR::FlowQuantity total_flow = max_flow.GetOptimalFlow();
        std::cout<<"the max flow quantity="<<total_flow<<"   num of robots="<<P->getNum()<<std::endl;
        if(total_flow==P->getNum()) {     //solved
            retrievePaths(max_flow,result);
            resolveEdgeConflicts(result);
            // std::cout<<"edge conflicts resolved!"<<std::endl;
            // return result;
        }
        timeStep++;
    }
}



/**
 * @brief 
 * 
 * @param id 
 * @param node 
 */
void FlowBasedUMRPP::insert_node(int &id,flowNode &node){
    if(node_id.find(node)==node_id.end()){
        node_id[node]=id;
        id_node[id]=node;
        id++;
    }
}



void FlowBasedUMRPP::add_edge(flowNode & u,flowNode &v){
    startNodes.push_back(node_id[u]);
    endNodes.push_back(node_id[v]);
}

/**
 * @brief 
 * 
 */
void FlowBasedUMRPP::evaluateLB(){
    makespanLB=0;
    for(int i=0;i<P->getNum();i++){
        int mki=9999996;
        for(int j=0;j<P->getNum();j++){
            int dij=P->getStart(i)->manhattanDist(P->getGoal(i));
            mki=std::min(mki,dij);
        }
        makespanLB=std::max(mki,makespanLB);
    }
}

/**
 * @brief 
 * 
 * @param timeStep 
 * @param result 
 * @return true 
 * @return false 
 */
void FlowBasedUMRPP::prepare(int timeStep){
    startNodes.clear();
    endNodes.clear();
    node_id.clear();
    id_node.clear();
    //out=0 in=1
    std::set<Node*> reachable_vertices;
    int id=0;
    for(int i=0;i<P->getNum();i++) reachable_vertices.insert(P->getStart(i));
    for(int t=0;t<timeStep;t++){
        std::set<Node*> next_reachable_vertices;
        for(auto v1: reachable_vertices){
            flowNode node1={v1->id,t,OUT};
            insert_node(id,node1);
            // auto nbrs=graph->getNeighbors(v1);
            auto nbrs=v1->neighbor;
            // std::cout<<"neighor size="<<nbrs.size()<<"  "<<v1->print()<<std::endl;
            nbrs.push_back(v1);
            for(auto v2: nbrs){
                flowNode node2={v2->id,t+1,IN};
                insert_node(id,node2);
                add_edge(node1,node2);
                next_reachable_vertices.insert(v2);
            }
            next_reachable_vertices.insert(v1);
            // flowNode node2={v1->id,t,IN}
        }
        for(auto &v:next_reachable_vertices){
            flowNode node2={v->id,t+1,IN};
            flowNode node3={v->id,t+1,OUT};
            insert_node(id,node3);
            add_edge(node2,node3);
        }
        reachable_vertices=next_reachable_vertices;
    }
    
    // add source and sink
    flowNode source={-985,-985,OUT};
    flowNode sink={-211,-211,IN};
    insert_node(id,source);
    source_id=node_id[source];
    insert_node(id,sink);
    sink_id=node_id[sink];
    for(int r=0;r<P->getNum();r++){
        flowNode sr={P->getStart(r)->id,0,OUT};
        flowNode gr={P->getGoal(r)->id,timeStep,OUT};
        // assert(node_id.find(sr)!=node_id.end());
        // assert(node_id.find(gr)!=node_id.end());
        add_edge(source,sr);
        add_edge(gr,sink);
    }
}   


/**
 * @brief 
 * 
 * @param max_flow 
 * @param result 
 */
void FlowBasedUMRPP::retrievePaths(OR::MaxFlow &max_flow,Paths &result){
    std::map<int,int> adj_list;

    auto starGraph=max_flow.graph();
    for(int i=0;i<startNodes.size();i++){
        if(fabs(max_flow.Flow(i)-1)<1e-2){
            auto head_i=starGraph->Head(i);
            auto tail_i=starGraph->Tail(i);
            
            auto head_node=id_node[head_i];
            auto tail_node=id_node[tail_i];
            // std::cout<<" head : ["<<graph->getVertex(head_node.first)->print()<<","<<head_node.second<<"] ";
            // std::cout<<"tail: ["<<graph->getVertex(tail_node.first)->print()<<","<<tail_node.second<<"] ";
            // std::cout<<"flow quatity: "<<max_flow.Flow(i)<<std::endl;
            adj_list.insert({tail_i,head_i});
        }
    }
    result=Paths(P->getNum());
    for(int i=0;i<P->getNum();i++){
        Path pi;
        // std::cout<<"robot "<<i<<std::endl;
        flowNode si={P->getStart(i)->id,0,OUT};
        flowNode sink={-211,-211,IN};
        int current_id=node_id[si];
        while(current_id!=node_id[sink]){
            // int timestep=id_node[current_id].second;
            auto flownode=id_node[current_id];
            int vid=std::get<0>(flownode);
            int in_or_out=std::get<2>(flownode);
            if(in_or_out==OUT) {
                pi.push_back(P->getG()->getNode(vid));
                // result[i].push_back( graph->getVertex(vid));
            }
            // std::cout<<"current id="<< current_id<<"  timestep="<<timestep<<"  "<<graph->getVertex(id_node[current_id].first)->print()<<"   "<<current_id<<"  -->   "<<adj_list[current_id] <<std::endl;
            current_id=adj_list[current_id];

        }
        result.insert(i,pi);
    }
    // for(int i=0;i<starts.size();i++){
    //     std::cout<<result[i].size()<<std::endl;
    // }
    
    // for(int i=0;i<startNodes.size();i++){
    //     goals[i]=result[i].back();
    // }
}



/**
 * @brief 
 * find if there is any edge conflict and resolve them by switching tails
 * @param result 
 */
void FlowBasedUMRPP::resolveEdgeConflicts(Paths &result){
    for(int t=1;t<result.getMakespan();t++){
        for(int i=0;i<result.size();i++){
            for(int j=i+1;j<result.size();j++){
                // if(result[i][t]==result[j][t-1] and result[i][t-1]==result[j][t]){
                if(result.get(i,t)==result.get(j,t-1) and result.get(i,t-1)==result.get(j,t)){
                    // std::cout<<"find edge conflict! resolving...."<<std::endl;
                    switchPaths(i,j,t,result);
                }
            }
        }
    }
    //update the goals
    // for(int i=0;i<starts.size();i++){
    //     goals[i]=result[i].back();
    // }
}


/**
 * @brief 
 * 
 * @param i 
 * @param j 
 * @param t 
 * @param result 
 */
void FlowBasedUMRPP::switchPaths(int i,int j,int t,Paths &result){
    // Path pj=result[j];
    Path pi=result.get(i);
    Path pj=result.get(j);
    // assert(result[j].size()==result[i].size());
    // assert(result[0].size()==result[i].size());
    // std::cout<<result[i].size()<<"    "<<result[j].size()<<std::endl;
    for(int k=t;k<result.getMakespan();k++){
        pj[k]=result.get(i,k);
        pi[k]=result.get(j,k);
    }
    result.insert(i,pi);
    result.insert(j,pj);
}


/**
 * @brief 
 * 
 * @return Paths 
 */
Plan FlowBasedUMRPP::solveWeighted(){
    auto getWeight=[&](int head,int tail){
        auto node1=id_node[head];
        auto node2=id_node[tail];
        if(std::get<0>(node1)==std::get<0>(node2)) return 0;
        return 1;
    };
    evaluateLB();
    int timeStep=makespanLB;
    bool solved=true;
    Paths result;
    while(true){
        // std::cout<<"preparing model for timestep="<<timeStep<<std::endl;
        prepare(timeStep);
        // std::cout<<"prepared model for timestep="<<timeStep<<std::endl;
        OR::StarGraph gs(node_id.size(), startNodes.size());
        OR::MinCostFlow min_cost_flow(&gs);
        std::vector<int> sink_arcs;
        for(int i=0;i<startNodes.size();i++){
            OR::ArcIndex arc=gs.AddArc(startNodes[i],endNodes[i]);
            min_cost_flow.SetArcCapacity(arc,1); //unit capacity
            min_cost_flow.SetArcUnitCost(arc,getWeight(startNodes[i],endNodes[i]));
            if(endNodes[i]==sink_id) sink_arcs.push_back(i);
        }
        int numRobots=P->getNum();
        min_cost_flow.SetNodeSupply(source_id,numRobots);
        min_cost_flow.SetNodeSupply(sink_id,-numRobots);

        min_cost_flow.Solve();
        int total_flow=0;
        for(auto arc_id:sink_arcs){
            total_flow+=min_cost_flow.Flow(arc_id);
        }
        // FlowQuantity total_flow = max_flow.GetOptimalFlow();
        std::cout<<"the max flow quantity="<<total_flow<<"   num of robots="<<P->getNum()<<std::endl;
        if(total_flow==numRobots) {     //solved
            retrievePaths(min_cost_flow,result);
            resolveEdgeConflicts(result);
            // std::cout<<"edge conflicts resolved!"<<std::endl;
            return pathsToPlan(result);
        }
        timeStep++;
    }
    return Plan();  //failed
}

/**
 * @brief 
 * 
 * @param max_flow 
 * @param result 
 */
void FlowBasedUMRPP::retrievePaths(OR::MinCostFlow &max_flow,Paths &result){
    std::map<int,int> adj_list;
    auto starGraph=max_flow.graph();
    for(int i=0;i<startNodes.size();i++){
        if(fabs(max_flow.Flow(i)-1)<1e-2){
            auto head_i=starGraph->Head(i);
            auto tail_i=starGraph->Tail(i);
            
            auto head_node=id_node[head_i];
            auto tail_node=id_node[tail_i];
            // std::cout<<" head : ["<<graph->getVertex(head_node.first)->print()<<","<<head_node.second<<"] ";
            // std::cout<<"tail: ["<<graph->getVertex(tail_node.first)->print()<<","<<tail_node.second<<"] ";
            // std::cout<<"flow quatity: "<<max_flow.Flow(i)<<std::endl;
            adj_list.insert({tail_i,head_i});
        }
    }
    result=Paths(P->getNum());

    for(int i=0;i<P->getNum();i++){
        // std::cout<<"robot "<<i<<std::endl;
        Path pi;
        flowNode si={P->getStart(i)->id,0,OUT};
        flowNode sink={-211,-211,IN};
        int current_id=node_id[si];
        while(current_id!=node_id[sink]){
            // int timestep=id_node[current_id].second;
            auto flownode=id_node[current_id];
            int vid=std::get<0>(flownode);
            int in_or_out=std::get<2>(flownode);
            if(in_or_out==OUT) pi.push_back(P->getG()->getNode(vid));
            // std::cout<<"current id="<< current_id<<"  timestep="<<timestep<<"  "<<graph->getVertex(id_node[current_id].first)->print()<<"   "<<current_id<<"  -->   "<<adj_list[current_id] <<std::endl;
            current_id=adj_list[current_id];
        }
        result.insert(i,pi);
    }
    // for(int i=0;i<starts.size();i++){
    //     std::cout<<result[i].size()<<std::endl;
    // }
    
    // for(int i=0;i<startNodes.size();i++){
    //     goals[i]=result[i].back();
    // }
}