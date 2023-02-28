#include"ddecbs.hpp"
#include "json.hpp"
#include <iostream>
#include <fstream>
#include <set>
#include "greedy.hpp"


// Node* greedy_choose(Grid * graph,Config starts, int agent,double preferred_density){

// }






void debug_expansion(){
    nlohmann::json datas;
    std::ifstream ifs("./instances/corner_dense/agents25_11.json");
    datas= nlohmann::json ::parse(ifs);
    Config starts,goals;
    Grid* graph=new Grid(20,20,{});
    std::vector<std::vector<int>> starts_vec=datas["starts"];
    std::vector<std::vector<int>> goals_vec=datas["goals"];
    for(int k=0;k<starts_vec.size();k++){
        assert(starts_vec[k][0]>=0 and starts_vec[k][1]>=0);
        assert(goals_vec[k][0]>=0 and goals_vec[k][1]>=0);
        starts.push_back(graph->getNode(starts_vec[k][0],starts_vec[k][1]));
        goals.push_back(graph->getNode(goals_vec[k][0],goals_vec[k][1]));
    }


    Config balanced_starts=greedy(graph,starts,0.5);
    int k=0;
    for(auto bs:balanced_starts){
        assert(bs!=nullptr);
        auto sk=starts[k];
        printf("%d: (%d,%d)==> (%d,%d)\n",k,sk->pos.x,sk->pos.y,bs->pos.x,bs->pos.y);
        k++;
    }
}


void debug_ddm(){
    nlohmann::json datas;
    std::ifstream ifs("./instances/corner_dense/agents25_11.json");
    datas= nlohmann::json ::parse(ifs);
    Config starts,goals;
    Grid* graph=new Grid(20,20,{});
    std::vector<std::vector<int>> starts_vec=datas["starts"];
    std::vector<std::vector<int>> goals_vec=datas["goals"];
    for(int k=0;k<starts_vec.size();k++){
        assert(starts_vec[k][0]>=0 and starts_vec[k][1]>=0);
        assert(goals_vec[k][0]>=0 and goals_vec[k][1]>=0);
        starts.push_back(graph->getNode(starts_vec[k][0],starts_vec[k][1]));
        goals.push_back(graph->getNode(goals_vec[k][0],goals_vec[k][1]));
    }
    DDECBS* solver=new DDECBS(graph,starts,goals);
    solver->solve();
    delete solver;
}



int main(){
    debug_expansion();
}




