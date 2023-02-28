#include <cassert>
#include "greedy.hpp"




Config greedy(Grid *graph,Config starts,double preferred_density){
    int num_agents=starts.size();
    Config balanced_starts;
    // std::map<Node*,int> position_map;
    std::map<int,int> num_neighboring_agents;
    std::map<Node*,int> node_area;
    // std::map<Node*,Nodes> obs_nodes;
    int obs_radius=2;

    auto cal_neighboring_area=[&](Node *nn){
        if(node_area.find(nn)!=node_area.end())return node_area[nn];
        int areas=0;
        std::queue<Node*> open;
        open.push(nn);
        std::set<Node*> closed;
        while(open.empty()==false){
            areas++;
            auto nt=open.front();
            open.pop();
            closed.insert(nt);
            for(auto nbr:nt->neighbor){
                if(closed.find(nbr)!=closed.end()) continue;
                if(nbr->pos.manhattanDist(nn->pos)>obs_radius) continue;
                open.push(nbr);
            }
        }
        node_area[nn]=areas;
        // printf("neighboring are=%d\n",areas);
        return areas;

    };

    auto check_density=[&](Node * proposed){
        std::pair<bool,std::vector<int>> results;
        std::vector<int> nbr_agents;
        double count=0;
        double area=cal_neighboring_area(proposed);
        for(int j=0;j<balanced_starts.size();j++){
        
            if(proposed->manhattanDist(balanced_starts[j])<=obs_radius){
                double area2=cal_neighboring_area(balanced_starts[j]);
                double nb_agents=num_neighboring_agents[j]+1;
                if(nb_agents/area2>preferred_density) {
                    results={false,{}};
                    // printf("agent %d exceed max robot density, %f\n",j,nb_agents/area2);
                    return results;
                }
                count++;
                nbr_agents.push_back(j);
            }
        
        }
        if(count/area<preferred_density) {
            // printf("robot density= %f,count=%f,area=%f\n",count/area,count,area);
            results={true,nbr_agents};
        }
        else {
            // printf("exceed max robot density, %f\n",count/area);
            results={false,{}};
        }
        return results;
        // return nbr_agents;
    };



    auto greedy_choose_one=[&](int i){
        Node *n1=nullptr;
        std::queue<Node*> open;
        open.push(starts[i]);
        std::set<Node*> closed;
        while(open.empty()==false){
            auto nt=open.front();
            open.pop();
            // printf("current node=(%d,%d)\n",nt->pos.x,nt->pos.y);
            // assert(closed.find(nt)==closed.end());
            if(closed.find(nt)!=closed.end()) continue;
            
        
            auto checking=check_density(nt);
            // printf("checking.first=%d\n",checking.first);
            if(checking.first==true and std::find(balanced_starts.begin(),balanced_starts.end(),nt)==balanced_starts.end()){
                // printf("??????????\n");
                auto nbr_agents=checking.second;

                for(auto agent: nbr_agents){
                    num_neighboring_agents[agent]++;
                }
                num_neighboring_agents[i]=nbr_agents.size();
                return nt;
            }
            closed.insert(nt);
            for(auto nc:nt->neighbor){
                if(closed.find(nc)!=closed.end()) continue;
    
                open.push(nc);
            }

        }
        return n1;
    };

    for(int i=0;i<starts.size();i++){
        
        auto si=greedy_choose_one(i);
        assert(si!=nullptr);

        balanced_starts.push_back(si);
        // position_map[si]=i;
    }
    return balanced_starts;

    
}  




Config a_star_greedy(Grid *graph,Config starts,Config goals,double preferred_density){
    int num_agents=starts.size();
    Config balanced_starts;
    // std::map<Node*,int> position_map;
    std::map<int,int> num_neighboring_agents;
    std::map<Node*,int> node_area;
    // std::map<Node*,Nodes> obs_nodes;
    int obs_radius=2;

    using AStarNode=std::pair<Node*,int>;

    auto compare_open=[&](AStarNode n1,AStarNode n2){
        if(n1.second!=n2.second) return n1.second>n2.second;
    };

    auto cal_neighboring_area=[&](Node *nn){
        if(node_area.find(nn)!=node_area.end())return node_area[nn];
        int areas=0;
        std::queue<Node*> open;
        open.push(nn);
        std::set<Node*> closed;
        while(open.empty()==false){
            areas++;
            auto nt=open.front();
            open.pop();
            closed.insert(nt);
            for(auto nbr:nt->neighbor){
                if(closed.find(nbr)!=closed.end()) continue;
                if(nbr->pos.manhattanDist(nn->pos)>obs_radius) continue;
                open.push(nbr);
            }
        }
        node_area[nn]=areas;
        // printf("neighboring are=%d\n",areas);
        return areas;

    };

    auto check_density=[&](Node * proposed){
        std::pair<bool,std::vector<int>> results;
        std::vector<int> nbr_agents;
        double count=0;
        double area=cal_neighboring_area(proposed);
        for(int j=0;j<balanced_starts.size();j++){
        
            if(proposed->manhattanDist(balanced_starts[j])<=obs_radius){
                double area2=cal_neighboring_area(balanced_starts[j]);
                double nb_agents=num_neighboring_agents[j]+1;
                if(nb_agents/area2>preferred_density) {
                    results={false,{}};
                    // printf("agent %d exceed max robot density, %f\n",j,nb_agents/area2);
                    return results;
                }
                count++;
                nbr_agents.push_back(j);
            }
        
        }
        if(count/area<preferred_density) {
            // printf("robot density= %f,count=%f,area=%f\n",count/area,count,area);
            results={true,nbr_agents};
        }
        else {
            // printf("exceed max robot density, %f\n",count/area);
            results={false,{}};
        }
        return results;
        // return nbr_agents;
    };



    auto greedy_choose_one=[&](int i){
        std::priority_queue<AStarNode> open;
        std::set<Node*> closed;
        Node* nn=nullptr;
        open.push({starts[i],starts[i]->manhattanDist(goals[i])});
        while(open.empty()==false){
            auto nt=open.top();
            open.pop();
            // printf("current node=(%d,%d)\n",nt->pos.x,nt->pos.y);
            // assert(closed.find(nt)==closed.end());
            if(closed.find(nt.first)!=closed.end()) continue;
            

            auto checking=check_density(nt.first);
            bool unused=std::find(balanced_starts.begin(),balanced_starts.end(),nt.first)==balanced_starts.end();
            if(unused) nn=nt.first;
            // printf("checking.first=%d\n",checking.first);
            if(checking.first==true and unused){
                // printf("??????????\n");
                auto nbr_agents=checking.second;

                for(auto agent: nbr_agents){
                    num_neighboring_agents[agent]++;
                }
                num_neighboring_agents[i]=nbr_agents.size();
                return nt.first;
            }
            closed.insert(nt.first);
            for(auto nc:nt.first->neighbor){
                if(closed.find(nc)!=closed.end()) continue;
                int f=nc->manhattanDist(starts[i])+nc->manhattanDist(goals[i]);
    
                open.push({nc,f});
            }

        }
        return nn;
    };

    for(int i=0;i<starts.size();i++){
        
        auto si=greedy_choose_one(i);
        assert(si!=nullptr);

        balanced_starts.push_back(si);
        // position_map[si]=i;
    }
    return balanced_starts;

    
}  