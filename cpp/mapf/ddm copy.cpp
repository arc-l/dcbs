#include "ddm.hpp"
#include <fstream>
#include <cassert>

#include <sstream>


const double sub_optimality=2.0;

std::vector<std::unordered_map<std::string, std::string>> ddm_database;


DDM::DDM(Problem *p):Solver(p){
}

DDM::DDM(Grid *graph,Config starts,Config goals):Solver(graph,starts,goals){

}

/// @brief 
void DDM::load_database() {
    if (!ddm_database.empty()) return;  // Resolve multi-calls
    // The database is organized as 1 empty entry, 6 entries for 2x3 graph, and
    // 8 entries for 3x3 graph with an obstacle
    ddm_database = std::vector<std::unordered_map<std::string, std::string>>(
        15, std::unordered_map<std::string, std::string>());
    // Load 2x3 database
    for (size_t i = 1; i < 7; i++) {
        std::string file_name = "data/ddm-database/2x3-" + std::to_string(i) + ".db";
        std::ifstream file_reader(file_name);
        std::string temp_line;
        while (std::getline(file_reader, temp_line)) {
            std::string initial_goal, path;
            std::stringstream lineStream(temp_line);
            lineStream >> initial_goal;
            lineStream >> path;
            ddm_database[i][initial_goal] = path;
        }
    }
    // Load 3x3 database
    for (size_t i = 7; i < 15; i++) {
        std::string file_name =
            "database/3x3-obs-" + std::to_string(i - 6) + ".db";
        std::ifstream file_reader(file_name);
        std::string temp_line;
        while (std::getline(file_reader, temp_line)) {
            std::string initial_goal, path;
            std::stringstream lineStream(temp_line);
            lineStream >> initial_goal;
            lineStream >> path;
            ddm_database[i][initial_goal] = path;
        }
    }
    assert(ddm_database[1].size() == 36);
    // std::cout<<"database loaded!"<<std::endl;
}

/// @brief 
/// @return 
Paths DDM::getInitialPaths(){
    Paths paths(P->getNum());
    for(int i=0;i<P->getNum();i++){
        auto path=getInitialPath(i);
        paths.insert(i,path);
    }
    return paths;
}

Path DDM::getInitialPath(int id){
    Node *s=P->getStart(id);
    Node *g=P->getGoal(id);
    return getInitialPath(id,s,g);
}

/// @brief 
/// @param id 
/// @param paths 
/// @return 
Path DDM::getInitialPath(int id,Node *s,Node *g){

    Nodes config_g = P->getConfigGoal();

    Path path = {s};
    Node *p = s;

    

    while (p!=g)
    {

        p = *std::min_element(p->neighbor.begin(), p->neighbor.end(),
                              [&](Node *a, Node *b)
                            {
                                if (pathDist(id, a) != pathDist(id, b))
                                    return pathDist(id, a) < pathDist(id, b);
                                if (a != g && inArray(a, config_g))
                                    return false;
                                if (b != g && inArray(b, config_g))
                                    return true;
                                return false;
                            });
        path.push_back(p);
    }
    // std::cout<<"finish calculating the path"<<std::endl;
    return path;
    
}




void DDM::run(){
    load_database();
    assert(!ddm_database.empty());
    auto initial_paths=getInitialPaths();
    // auto initial_paths=getInitialPathsFocal();
    initial_paths.shrink();
    auto result=simulate(initial_paths,max_steps_allowed);
    solution= result.first;
}


std::pair<Plan,bool> DDM::simulate(const Paths &initial_paths,int max_steps)
{
    auto occupied_subgraphs =
        std::vector<Protected_Subgraph>();  // Subgraphs used for collision
                                        // resolution
    Plan plan;

    plan.add(P->getConfigStart());
    std::vector<Path> future_paths(P->getNum());
    for(int i=0;i<initial_paths.size();i++){
        future_paths[i]=initial_paths.get(i);
        std::reverse(future_paths[i].begin(),future_paths[i].end());   
        // assert(future_paths[i].empty()==false);
        future_paths[i].pop_back();    
    }



    int iteration=0;
    while(true){
        // std::cout<<"current iteration="<<iteration<<std::endl;
        iteration++;

        bool future_paths_all_empty = true;
        for (size_t i = 0; i < P->getNum(); i++){
            if (!future_paths[i].empty()) {
                future_paths_all_empty = false;
                // for(int k=future_paths[i].size()-1;k>=0;k--){
                //     std::cout<<"("<<future_paths[i][k]->pos.x<<","<<future_paths[i][k]->pos.y<<")   ";
                // }
                // std::cout<<"goal of agent"<<i<<"=("<<P->getGoal(i)->pos.x<<","<<P->getGoal(i)->pos.y<<")   "
                // <<"current=("<<plan.last(i)->pos.x<<","<<plan.last(i)->pos.y<<")"<<std::endl;
                break;
            }
        }
        if(future_paths_all_empty)return {plan,true};
        if(iteration>max_steps) return {plan,false};


        for(int i=0;i<P->getNum();i++){
            if(future_paths[i].empty()) {
                assert(plan.last(i)!=nullptr);
                future_paths[i].push_back(plan.last(i));
            }
        }

        // Find the next step according to future paths
        auto next_step = Config(P->getNum());
        for (size_t i = 0; i < P->getNum(); i++)
            next_step[i] = future_paths[i].back();

         // Find all collisions
        auto collided_robots =
            std::vector<std::pair<size_t, size_t>>();  // Robots with
                                                       // conflicts
        
        for (size_t i = 0; i < P->getNum(); i++)
            for (size_t j = i + 1; j < P->getNum(); j++)
                if (next_step[i] == next_step[j] ||
                    (plan.last(i) == next_step[j] &&
                    plan.last(j) == next_step[i]))
                    collided_robots.push_back(std::make_pair(i, j));

        // Random shuffle collisions for more diversified sollision
        // resolution
        numConflictsResolved+=collided_robots.size();
        std::shuffle(collided_robots.begin(), collided_robots.end(),
                    std::default_random_engine(rand()));
        
        // Resolve collisions one by one
        if (!collided_robots.empty()) {
            // std::cout<<"collisions found"<<std::endl;

            for (auto collided_robot_pair : collided_robots) {
                // Find the colliding robots, with the one with a longer
                // future path in the front
                size_t r1 = collided_robot_pair.first;
                size_t r2 = collided_robot_pair.second;

                if (future_paths[r1].size() < future_paths[r2].size())
                    std::swap(r1, r2);

                // Find a 2x3 graph

                Grid *grid=dynamic_cast<Grid*>(P->getG());


                // printf("collided  (%d,%d) and (%d,%d)\n",plan.last(r1)->pos.x,plan.last(r1)->pos.y,
                //     plan.last(r2)->pos.x,plan.last(r2)->pos.y);
                // assert(plan.empty()==false);
                assert(plan.last(r1)!=nullptr);
                assert(plan.last(r2)!=nullptr);
                
                std::vector<SubGraph> subgraph_candidates =
                    get_all_possible_2x3(plan.last(r1)->pos, plan.last(r2)->pos,
                                         grid);  // All possible 2x3 graphs
                                              // w.r.task. environment obstacles
                // printf("subgraphs number=%d\n",subgraph_candidates.size());
                // Choose the 2x3 graph that does not intersect with current
                // 2x3 graphs in use
                size_t chosen_subgraph_candidate_index = 0;
                for (; chosen_subgraph_candidate_index <subgraph_candidates.size();chosen_subgraph_candidate_index += 1) {
                    bool no_intersection =
                        true;  // Flag to tell if the candidate has
                               // intersection with any existing subgraphs
                    for (auto occupied_subgraph : occupied_subgraphs)
                        if (subgraph_candidates[chosen_subgraph_candidate_index]
                                .intersect_with(occupied_subgraph.graph)) {
                            no_intersection = false;
                            break;
                        }
                    if (no_intersection) break;
                }

                // If a valid 2x3 graph is not found, move on to the next
                // collision; otherwise start collision resolution
                if (chosen_subgraph_candidate_index ==
                    subgraph_candidates.size())
                    continue;
                SubGraph& subg =
                    subgraph_candidates[chosen_subgraph_candidate_index];

                // Find all robots in the subgraph. The movement of these
                // robots will be affected by the collision resolution
                // process. Sort them to give priority to robots with a
                // longer residual path
                std::vector<size_t> affected_robots({r1, r2});
                for (size_t i = 0; i < P->getNum(); i++)
                    if (i != r1 && i != r2 && subg.contains(plan.last(i)->pos))
                        affected_robots.push_back(i);
                std::vector<Path>* pp = &future_paths;
                std::sort(affected_robots.begin(), affected_robots.end(),
                        [&pp](size_t a, size_t b) {
                            return (*pp)[a].size() > (*pp)[b].size();
                        });

                // Find temporary starts and goals for each robot.
                auto potential_sub_goals =
                    subg.get_nodes();  // Nodes in the subgraph not yet
                                       // assigned to the robots as goals
                Config sub_starts_global(
                    affected_robots.size());  // Starts of the robots in the
                                              // subproblem in global scope
                Config sub_goals_global(
                    affected_robots.size());  // Goal of the robots in the
                                              // subproblem in global scope
                for (size_t i = 0; i < affected_robots.size(); i++) {
                    sub_starts_global[i] = plan.last(affected_robots[i]);
                    // Find the robot's the last waypoint in the subgraph.
                    // Here we use a lookahead of at most 5 steps to speed
                    // up.
                    size_t last_waypoint_index = std::max(
                        0, int(future_paths[affected_robots[i]].size()) - 5);
                    bool found_subgoal = false;
                    for (; last_waypoint_index <future_paths[affected_robots[i]].size();last_waypoint_index++){
                        printf("subg lo=(%d,%d),hi=(%d,%d),contaning(%d,%d)\n",subg.lo.x,subg.lo.y,subg.hi.x,subg.hi.y,future_paths[affected_robots[i]][last_waypoint_index]->pos.x,future_paths[affected_robots[i]][last_waypoint_index]->pos.y);
                        if (subg.contains(future_paths[affected_robots[i]][last_waypoint_index]->pos)) {
                            sub_goals_global[i] =
                                future_paths[affected_robots[i]]
                                            [last_waypoint_index];
                            assert(sub_goals_global[i]!=nullptr);
                            assert(future_paths[affected_robots[i]]
                                            [last_waypoint_index]!=nullptr);
                            found_subgoal = true;
                            std::cout<<"found subgoal for robot" <<i<<std::endl;
                            break;
                        }
                        std::cout<<"????????"<<std::endl;
                    }
                    if(sub_goals_global[i]==nullptr) std::cout<<"null "<<i<<std::endl;
                    assert(sub_goals_global[i]!=nullptr);
                    auto it = std::find(potential_sub_goals.begin(),
                                        potential_sub_goals.end(),
                                        sub_goals_global[i]->pos);
                    // Find if the desired goal is occupied. If goal not
                    // occupied, assign.
                    if (found_subgoal && it != potential_sub_goals.end()) {
                        potential_sub_goals.erase(it);
                        // Update future path
                        // TODO STO: update path
                        future_paths[affected_robots[i]].erase(
                            future_paths[affected_robots[i]].begin() +
                                last_waypoint_index,
                            future_paths[affected_robots[i]].end());
                        if (future_paths[affected_robots[i]].empty()) {
                            // TODO check if this is necessary
                            future_paths[affected_robots[i]].push_back(
                                P->getGoal(affected_robots[i]));
                            // TODO STO: update path
                        }

                        continue;
                    }
                    // If desired goal occupied, try to move the robot to a
                    // random vertex
                    // TODO replace with some near vertex?
                    std::shuffle(potential_sub_goals.begin(),
                                potential_sub_goals.end(),
                                std::default_random_engine(rand()));
                    if (grid->is_blocked(potential_sub_goals.back()))
                        potential_sub_goals.pop_back();
                    auto psg=potential_sub_goals.back();
                    sub_goals_global[i] = grid->getNode(psg.x,psg.y);
                    assert(potential_sub_goals.empty()==false);
                    potential_sub_goals.pop_back();
                    if (sub_goals_global[i] != sub_starts_global[i]) {
                        future_paths[affected_robots[i]].clear();
                        int id=affected_robots[i];
                        if (sub_goals_global[i] !=P->getGoal(id)) {
                            future_paths[id]=getInitialPath(id,sub_goals_global[i],P->getGoal(id));
                            // assert(future_paths[id[])
                            // Paths remain_paths(P->getNum());
                            // for(int k=0;k<P->getNum();k++){
                                
                            
                            //     auto p=future_paths[k];
                            //     if(p.empty()==false)
                            //         std::reverse(p.begin(),p.end());
                            //     else
                            //         p.push_back(plan.last(k));
                            //     remain_paths.insert(k,p);
                            // }
                            
                    
                            // future_paths[id]=getFocalPath(id,sub_goals_global[i],P->getGoal(id),remain_paths);
                            std::reverse(future_paths[id].begin(),future_paths[id].end());
                            // if(future_paths[id].empty()){
                            //     printf("debug!!! id=%d, (%d,%d) to (%d,%d) \n",id,sub_goals_global[i]->pos.x,
                            //         sub_goals_global[i]->pos.y,P->getGoal(id)->pos.x,P->getGoal(id)->pos.y);
                            // }
                            assert(future_paths[id].empty()==false);
                            future_paths[id].pop_back();
            
                        }
                    }
                }
                // Get local starts and goals
                size_t num_affected_robots = affected_robots.size();
                auto sub_starts_local = std::vector<size_t>();
                auto sub_goals_local = std::vector<size_t>();
                // std::cout<<"deeeeeeebuggggging"<<std::endl;
                subg.get_nodes();
                if (subg.small) {
                    if (subg.hi.x - subg.lo.x ==
                        2)  // Subgraph landscape placement
                        for (size_t i = 0; i < affected_robots.size(); i++) {
                            sub_starts_local.push_back(
                                (sub_starts_global[i]->pos.x - subg.lo.x) +
                                (sub_starts_global[i]->pos.y - subg.lo.y) * 3);
                            sub_goals_local.push_back(
                                (sub_goals_global[i]->pos.x - subg.lo.x) +
                                (sub_goals_global[i]->pos.y - subg.lo.y) * 3);
                        }
                    else {  // Subgraph portrait placement
                        size_t converter[] = {0, 3, 1, 4, 2, 5};
                        for (size_t i = 0; i < affected_robots.size(); i++) {
                            sub_starts_local.push_back(
                                converter[(sub_starts_global[i]->pos.x - subg.lo.x) +
                                          (sub_starts_global[i]->pos.y - subg.lo.y) *
                                            2]);
                            sub_goals_local.push_back(
                                converter[(sub_goals_global[i]->pos.x - subg.lo.x) +
                                          (sub_goals_global[i]->pos.y - subg.lo.y) *
                                            2]);
                        }
                    }
                } else {
                    // 3x3 with an obstacle at corner.
                    std::vector<int> converter;
                    if (grid->is_blocked(subg.hi.x, subg.hi.y))
                        converter =
                            std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8});
                    if (grid->is_blocked(subg.lo.x, subg.lo.y))
                        converter =
                            std::vector<int>({8, 7, 6, 5, 4, 3, 2, 1, 0});
                    if (grid->is_blocked(subg.hi.x, subg.lo.y))
                        converter =
                            std::vector<int>({2, 5, 8, 1, 4, 7, 0, 3, 6});
                    if (grid->is_blocked(subg.lo.x, subg.hi.y))
                        converter =
                            std::vector<int>({6, 3, 0, 7, 4, 1, 8, 5, 2});
                    for (size_t i = 0; i < affected_robots.size(); i++) {
                        sub_starts_local.push_back(
                            converter[(sub_starts_global[i]->pos.x - subg.lo.x) +
                                      (sub_starts_global[i]->pos.y - subg.lo.y) *
                                        3]);
                        sub_goals_local.push_back(
                            converter[(sub_goals_global[i]->pos.x - subg.lo.x) +
                                      (sub_goals_global[i]->pos.y - subg.lo.y) * 3]);
                    }
                }

                // Sort starts and goals for database entires
                std::vector<size_t> sub_starts_local_sorted =
                    sub_starts_local;  // Sorted local starts
                std::sort(sub_starts_local_sorted.begin(),sub_starts_local_sorted.end());
                auto sort_index = std::vector<size_t>(num_affected_robots, 0);
                auto sub_goals_local_sorted = std::vector<size_t>(
                    num_affected_robots, 0);  // Sorted local goals
                for (size_t i = 0; i < num_affected_robots; i++) {
                    sort_index[i] = std::distance(
                        sub_starts_local.begin(),
                        std::find(sub_starts_local.begin(), sub_starts_local.end(),
                            sub_starts_local_sorted[i]));
                    sub_goals_local_sorted[i] = sub_goals_local[sort_index[i]];
                }

                // Database query
                std::string database_key;
                for (auto var : sub_starts_local_sorted)
                    database_key += std::to_string(var);
                for (auto var : sub_goals_local_sorted)
                    database_key += std::to_string(var);
                std::string solution_str;
                if (subg.small)
                    solution_str =
                        ddm_database[num_affected_robots][database_key];
                else{
                    // printf("subg size= %d,%d x %d, %d\n",subg.lo.x,subg.hi.x,subg.lo.y,subg.hi.y);
                    // std::cout<<"what's wrong?  "<<ddm_database.size()<<"  "<<num_affected_robots+6<<"   "<<database_key<<std::endl;
                    solution_str =
                        ddm_database[num_affected_robots + 6][database_key];
                }

                // Extract solution
                size_t path_length_subg =
                    solution_str.size() / num_affected_robots;
                auto temp_paths = std::vector<std::vector<size_t>>(
                    num_affected_robots,
                    std::vector<size_t>(path_length_subg + 2, 0));
                for (size_t t_ = 0; t_ < path_length_subg; t_++)
                    for (size_t i = 0; i < num_affected_robots; i++) {
                        temp_paths[sort_index[i]][t_ + 1] =
                            solution_str[t_ * num_affected_robots + i] - 48;
                    }
                for (size_t i = 0; i < num_affected_robots; i++) {
                    temp_paths[i][0] = sub_starts_local[i];
                    temp_paths[i][path_length_subg + 1] = sub_goals_local[i];
                }

                // Retrive solution and modify planned paths
                path_length_subg += 1;
                if (subg.small) {
                    if (subg.hi.x - subg.lo.x ==
                        2)  // Subgraph landscape placement
                        for (size_t t = path_length_subg; t > 0; t--) {
                            for (size_t i = 0; i < num_affected_robots; i++) {
                                future_paths[affected_robots[i]].push_back(
                                    grid->getNode(subg.lo.x + temp_paths[i][t] %3,
                                        subg.lo.y + temp_paths[i][t] / 3));
                                
                                assert(future_paths[affected_robots[i]].back()!=nullptr);
                            }
                        }
                    else {  // Subgraph portrait placement
                        static int converter[] = {0, 2, 4, 1, 3, 5};
                        for (size_t t = path_length_subg; t > 0; t--) {
                            for (size_t i = 0; i < num_affected_robots; i++) {
                                temp_paths[i][t] = converter[temp_paths[i][t]];
                                future_paths[affected_robots[i]].push_back(
                                    grid->getNode(subg.lo.x + temp_paths[i][t] % 2,
                                        subg.lo.y + temp_paths[i][t] / 2));
                                assert(future_paths[affected_robots[i]].back()!=nullptr);
                            }
                        }
                    }
                } else {
                    // 3x3 with an obstacle at corner.
                    std::vector<int> converter;
                    if (grid->is_blocked(subg.hi.x, subg.hi.y))
                        converter =
                            std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8});
                    if (grid->is_blocked(subg.lo.x, subg.lo.y))
                        converter =
                            std::vector<int>({8, 7, 6, 5, 4, 3, 2, 1, 0});
                    if (grid->is_blocked(subg.lo.x, subg.hi.y))
                        converter =
                            std::vector<int>({2, 5, 8, 1, 4, 7, 0, 3, 6});
                    if (grid->is_blocked(subg.hi.x, subg.lo.y))
                        converter =
                            std::vector<int>({6, 3, 0, 7, 4, 1, 8, 5, 2});
                    for (size_t t = path_length_subg; t > 0; t--) {
                        for (size_t i = 0; i < num_affected_robots; i++) {
                            temp_paths[i][t] = converter[temp_paths[i][t]];
                            future_paths[affected_robots[i]].push_back(
                                grid->getNode(subg.lo.x + temp_paths[i][t] % 3,
                                    subg.lo.y + temp_paths[i][t] / 3));
            
                            if(grid->getNode(subg.lo.x + temp_paths[i][t] % 3,subg.lo.y + temp_paths[i][t] / 3)==nullptr) {
                                std::cout<<"not exist  ("<<subg.lo.x + temp_paths[i][t] %3<<","<<subg.lo.y + temp_paths[i][t] / 3<<")"<<std::endl;
                            }
                            std::cout<<"add ("<<subg.lo.x + temp_paths[i][t] %3<<","<<subg.lo.y + temp_paths[i][t] / 3<<std::endl;
                            assert(future_paths[affected_robots[i]].back()!=nullptr);
                        }
                    }
                }
                // Mark subg as occupied
                occupied_subgraphs.push_back(Protected_Subgraph(
                    subg, affected_robots, path_length_subg));
            }
        }

        // Execute paths: preparation

        Config next_config(P->getNum(),nullptr);
        for (size_t i = 0; i < P->getNum(); i++){
            // if(future_paths[i].back()==nullptr){
            //     std::cout<<"null future paths "<<future_paths[i].size()<<std::endl;
            // }
            assert(future_paths[i].back()!=nullptr);
            next_step[i] = future_paths[i].back();


        }

        // Execute paths: execute protected paths (robots in subgraphs), and
        // deal with all non-protected paths (robots not in subgraphs)
        auto non_protected_robots = std::vector<size_t>(P->getNum());
        for (size_t i = 0; i < P->getNum(); i++)
            non_protected_robots[i] = i;
        for (size_t i = 0; i < occupied_subgraphs.size(); i++)
            for (size_t j = 0; j < occupied_subgraphs[i].robots.size(); j++) {
                // Execute protected robots
                // std::cout<<"protected robot "<< occupied_subgraphs[i].robots[j]<<" executed"<<std::endl;
                next_config[occupied_subgraphs[i].robots[j]] =
                    next_step[occupied_subgraphs[i].robots[j]];
                assert(future_paths[occupied_subgraphs[i].robots[j]].empty()==false);
                future_paths[occupied_subgraphs[i].robots[j]].pop_back();

                // Non-protected
                non_protected_robots.erase(find(
                    non_protected_robots.begin(), non_protected_robots.end(),
                    occupied_subgraphs[i].robots[j]));
            }

        // Execute paths: non protected robots
        // Iterate through subgraphs to see if a robot needs to be stopped
        for (size_t r : non_protected_robots)
            for (size_t i = 0; i < occupied_subgraphs.size(); i++)
                if (occupied_subgraphs[i].graph.contains(next_step[r]->pos)) {
                    // Outlier: if the robot does not affect the collision
                    // resolution, it can move into the subgraph
                    if (occupied_subgraphs[i].delay == 1)
                        if (std::find(next_config.begin(), next_config.end(),
                                    next_step[r]) == next_config.end())
                            break;
                    // Otherwise, the robot stays still
                    if (!(next_step[r] == plan.last(r))) {
                        assert(plan.last(r)!=nullptr);
                        future_paths[r].push_back(plan.last(r));
                    }
                    next_step[r] = plan.last(r);
                    break;
                }

        // Execute paths: non protected robots
        // Recursively stop the other robots
        auto flow =
            std::map<Node*, std::vector<std::pair<size_t, Node*>>>();  // Key is a node, value is
                                                    // the indices and current
                                                    // Nodeations of the robots
                                                    // going into the node
        for (auto r : non_protected_robots) {
            auto it = flow.find(next_step[r]);
            if (it == flow.end())
                flow.insert(std::make_pair(
                    next_step[r],
                    std::vector<std::pair<size_t, Node*>>(
                        {std::make_pair(r, plan.last(r))})));
            else
                it->second.push_back(std::make_pair(r, plan.last(r)));
        }
        auto stopped_robots = std::vector<size_t>();
        for (auto it = flow.begin(); it != flow.end(); it++) {
            if (it->second.size() ==
                1) {  // The node has only one robot intended to move in
                auto it2 = flow.find(it->second[0].second);
                if (it2 == flow.end()) continue;
                // Here, if jump is false, there will be a head to head
                // collision. Thus robots must be stopped.
                bool jump = true;
                for (auto var : it2->second)
                    if (var.second == it->first) {
                        jump = false;
                        break;
                    }
                if (jump) continue;
            }
            // Recursively stop robots
            recursive_adder(flow, stopped_robots, it);
        }

        // Execute paths: non protected robots
        for (auto r : stopped_robots) {
            next_config[r] = plan.last(r);
            // std::cout<<"robot "<< r<<" executed"<<std::endl;
            assert(next_config[r]!=nullptr);
            if (future_paths[r].back() == plan.last(r)) {
                assert(future_paths[r].empty()==false);
                future_paths[r].pop_back();

            }
        }
        for (auto r : non_protected_robots){
            if (next_config[r]==nullptr) {
                // std::cout<<"robot "<< r<<" executed"<<std::endl;
                next_config[r] = next_step[r];
                assert(next_config[r]!=nullptr);
                assert(future_paths[r].empty()==false);
                future_paths[r].pop_back();
            }
            
        }
        // printf("stopping %d+%d=%d",stopped_robots.size(),non_protected_robots.size(),future_paths.size());
        for(int r=0;r<future_paths.size();r++){
            // if(next_config[r]==nullptr) std::cout<<"Null ptr "<<r<<std::endl;
            assert(next_config[r]!=nullptr);
        }

        // Remove duplicate step (if any)
        if (next_config != plan.last()) {
            plan.add(next_config);
        }

        // Remove the subgraphs that finished execution
        auto remove_list = std::vector<size_t>();
        for (size_t i = 0; i < occupied_subgraphs.size(); i++) {
            occupied_subgraphs[i].delay -= 1;
            if (occupied_subgraphs[i].delay == 0) remove_list.push_back(i);
        }
        std::sort(remove_list.begin(), remove_list.end());
        std::reverse(remove_list.begin(), remove_list.end());
        for (size_t i : remove_list) {
            occupied_subgraphs.erase(occupied_subgraphs.begin() + i);
        }
    }
    
}

std::vector<SubGraph> DDM::get_all_possible_2x3(const Pos n1, const Pos n2,
                                                            const Grid* g) {
    std::vector<std::pair<Pos, Pos>> node_pairs;
    node_pairs.reserve(8);
    std::vector<SubGraph> result;
    result.reserve(8);
    printf("get all possible 2x3 Pos n1= (%d,%d), Pos n2=(%d,%d)\n",n1.x,n1.y,n2.x,n2.y);
    int x_low = n1.x > n2.x ? n2.x : n1.x;
    int x_high = n1.x > n2.x ? n1.x : n2.x;
    int y_low = n1.y > n2.y ? n2.y : n1.y;
    int y_high = n1.y > n2.y ? n1.y : n2.y;
    // 3x3 with obs
    if (std::abs(n1.x - n2.x) == 1 && std::abs(n1.y - n2.y) == 1 &&
        (g->is_blocked(n1.x, n2.y) || g->is_blocked(n2.x, n1.y))) {
        node_pairs.push_back(
            std::make_pair(Pos(x_low - 1, y_low - 1), Pos(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low - 1), Pos(x_high + 1, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low - 1, y_low), Pos(x_high, y_high + 1)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low), Pos(x_high + 1, y_high + 1)));
    }

    if (!node_pairs.empty()) {
        // std::cout<<"NOde pairs is not empty, node_pairs size="<<node_pairs.size()<<std::endl;
        for (auto np : node_pairs){
            if (g->inBoundary(np.first.x,np.first.y) && g->inBoundary(np.second.x,np.second.y)) {
                auto subg = SubGraph(np.first, np.second);
                auto subg_nodes = subg.get_nodes();
            
                int num_blocked = 0;
                for (auto n : subg_nodes) {
                    if (g->is_blocked(n)) num_blocked++;
                    if (num_blocked > 1) break;
                }
                if (num_blocked == 1 && (g->is_blocked(subg_nodes[0]) ||
                                        g->is_blocked(subg_nodes[2]) ||
                                        g->is_blocked(subg_nodes[6]) ||
                                        g->is_blocked(subg_nodes[8]))) {
                    result.push_back(subg);                
                    break;
                }
            }
        }

        assert(result.size() == 1);
        return result;
    }
    if (n1.x == n2.x && std::abs(n1.y - n2.y) == 1) {
        node_pairs.push_back(
            std::make_pair(Pos(x_low - 2, y_low), Pos(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low - 1, y_low), Pos(x_high + 1, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low), Pos(x_high + 2, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low - 1, y_low - 1), Pos(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low - 1), Pos(x_high + 1, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low - 1, y_low), Pos(x_high, y_high + 1)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low), Pos(x_high + 1, y_high + 1)));
    } else if (n1.x == n2.x && std::abs(n1.y - n2.y) == 2) {
        node_pairs.push_back(
            std::make_pair(Pos(x_low - 1, y_low), Pos(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low), Pos(x_high + 1, y_high)));
    } else if (n1.y == n2.y && std::abs(n1.x - n2.x) == 1) {
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low - 2), Pos(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low - 1), Pos(x_high, y_high + 1)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low), Pos(x_high, y_high + 2)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low - 1, y_low - 1), Pos(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low - 1, y_low), Pos(x_high, y_high + 1)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low - 1), Pos(x_high + 1, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low), Pos(x_high + 1, y_high + 1)));
    } else if (n1.y == n2.y && std::abs(n1.x - n2.x) == 2) {
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low - 1), Pos(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low), Pos(x_high, y_high + 1)));
    } else {
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low - 1), Pos(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low), Pos(x_high, y_high + 1)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low - 1, y_low), Pos(x_high, y_high)));
        node_pairs.push_back(
            std::make_pair(Pos(x_low, y_low), Pos(x_high + 1, y_high)));
    }

    for (auto np : node_pairs) {
        auto subg = SubGraph(np.first, np.second);
        if (test_2x3_valid(g, subg)) result.push_back(subg);
    }
    std::shuffle(result.begin(), result.end(),
                std::default_random_engine(rand()));
    assert(result.size() != 0);
    return result;
}

bool DDM::test_2x3_valid(const Grid* g, const SubGraph& subg) {
    // printf("grid w=%d h=%d\n",g->getWidth(),g->getHeight());
    if (subg.lo.x < 0 || subg.lo.y < 0 || subg.hi.x >= g->getWidth() ||
        subg.hi.y >= g->getHeight())
        return false;

    for (auto var : subg.get_nodes())
        if (g->is_blocked(var.x,var.y)) return false;
    return true;
}












/// @brief 
/// @return 
Paths DDM::getInitialPathsFocal(){
    Paths paths(P->getNum());
    for(int i=0;i<P->getNum();i++){
        auto path=getFocalPath(i,paths);
        paths.insert(i,path);
    }
    return paths;
}

Path DDM::getFocalPath(int id,const Paths&paths){
    
    return getFocalPath(id,P->getStart(id),P->getGoal(id),paths);
}

/// @brief 
/// @param id 
/// @param paths 
/// @return 
Path DDM::getFocalPath(int id, Node *s, Node *g,const Paths&paths){
    // pre processing
    LibCBS::Constraints constraints;
    int max_constraint_time = 0;
    // f-value for online list
    FocalHeuristics f1Value;
    if (pathDist(id) > max_constraint_time)
    {
        f1Value = [&](FocalNode *n)
        { return n->g + pathDist(id, n->v); };
    }
    else
    {
        f1Value = [&](FocalNode *n)
        {
            return std::max(max_constraint_time + 1, n->g + pathDist(id, n->v));
        };
    }


    const int makespan = paths.getMakespan();


    updatePathTable(paths, id);



    // std::cout << "PathTable updated in " << dt << std::endl;
    FocalHeuristics f2Value = [&](FocalNode *n)
    {
        if (n->g == 0)
            return 0.0;
        // last node
        if (n->g > makespan)
        {
            if (PATH_TABLE[makespan][n->v->id] != Solver::NIL)
                return n->p->f2 + 1;
        }
        else
        {
            // vertex conflict
            if (PATH_TABLE[n->g][n->v->id] != Solver::NIL)
            {
                return n->p->f2 + 1;

                // swap conflict
            }
            else if (PATH_TABLE[n->g][n->p->v->id] != Solver::NIL &&
                    PATH_TABLE[n->g - 1][n->v->id] ==
                        PATH_TABLE[n->g][n->p->v->id])
            {
                return n->p->f2 + 1;
            }
        }
        return n->p->f2;
    };


    CompareFocalNode compareOPEN = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f1 != b->f1)
            return a->f1 > b->f1;

        return false;
    };

    CompareFocalNode compareFOCAL = [&](FocalNode *a, FocalNode *b)
    {
        if (a->f2 != b->f2)
            return a->f2 > b->f2;
        if (a->f1 != b->f1)
            return a->f1 > b->f1;
        if (a->g != b->g)
            return a->g < b->g;
        return false;
    };

    CheckFocalFin checkFocalFin = [&](FocalNode *n)
    {
        return n->v == g && n->g > max_constraint_time;
    };

    CheckInvalidFocalNode checkInvalidFocalNode = [&](FocalNode *m)
    {
        for (auto c : constraints)
        {
            if (m->g == c->t && m->v == c->v)
            {
                // vertex or swap conflict
                if (c->u == nullptr || c->u == m->p->v)
                    return true;
            }
        }
        return false;
    };

    auto p = getTimedPathByFocalSearch(s, g, sub_optimality, f1Value, f2Value,
                                    compareOPEN, compareFOCAL, checkFocalFin,
                                    checkInvalidFocalNode);

    return p;    
}



Path DDM::getTimedPathByFocalSearch(Node *const s, Node *const g, float w, // sub-optimality
        FocalHeuristics &f1Value, FocalHeuristics &f2Value,
        CompareFocalNode &compareOPEN, CompareFocalNode &compareFOCAL,
        CheckFocalFin &checkFocalFin,
        CheckInvalidFocalNode &checkInvalidFocalNode){

    auto getNodeName = [](FocalNode *n)
    {
        return std::to_string(n->v->id) + "-" + std::to_string(n->g);
    };

    std::vector<FocalNode *> GC; // garbage collection
    auto createNewNode = [&](Node *v, int g, double f1, double f2, FocalNode *p)
    {
        FocalNode *new_node = new FocalNode{v, g, f1, f2, p};
        GC.push_back(new_node);
        return new_node;
    };

    auto getPathFromFocalNode=[](FocalNode* _n){
        Path path;
        FocalNode *n=_n;
        while (n!=nullptr)
        {
            path.push_back(n->v);
            n=n->p;
        }
        std::reverse(path.begin(),path.end());
        return path;
        
    };

    // OPEN, FOCAL, CLOSE
    std::priority_queue<FocalNode *, std::vector<FocalNode *>, CompareFocalNode>
        OPEN(compareOPEN);
    std::unordered_map<std::string, bool> CLOSE;
    using FocalList = std::priority_queue<FocalNode *, std::vector<FocalNode *>,
                                        CompareFocalNode>;
    FocalList FOCAL(compareFOCAL);

    // initial node
    FocalNode *n;
    n = createNewNode(s, 0, 0, 0, nullptr);
    n->f1 = f1Value(n);
    n->f2 = f2Value(n);
    OPEN.push(n);
    FOCAL.push(n);
    int f1_min = n->f1;

    // main loop
    bool invalid = true;
    while (!OPEN.empty())
    {
        
        // check time limit
        // if (overCompTime())
        //     break;
        // std::cout<<"?????   "<<OPEN.size()<<std::endl;
        /*
         * update FOCAL list
         * see the high-level search
         */
        while (!OPEN.empty() && CLOSE.find(getNodeName(OPEN.top())) != CLOSE.end())
            OPEN.pop();
        if (OPEN.empty())
            break; // failed
        if (f1_min != OPEN.top()->f1)
        {
            f1_min = OPEN.top()->f1;
            float f1_bound = f1_min * w;
            std::vector<FocalNode *> tmp;
            FocalList EMPTY(compareFOCAL);
            FOCAL = EMPTY;
            while (!OPEN.empty())
            {
                FocalNode *top = OPEN.top();
                OPEN.pop();
                // already searched by focal
                if (CLOSE.find(getNodeName(top)) != CLOSE.end())
                    continue;
                tmp.push_back(top); // escape
                if ((float)top->f1 > f1_bound)
                    break;       // higher than f1_bound
                FOCAL.push(top); // lower than f1_bound
            }
            for (auto ele : tmp)
                OPEN.push(ele); // back
        }

        // focal minimum node
        n = FOCAL.top();
        // printf("exploring node (%d,%d), goal=(%d,%d)\n",n->v->pos.x,n->v->pos.y,g->pos.x,g->pos.y);
        FOCAL.pop();
        if (CLOSE.find(getNodeName(n)) != CLOSE.end())
            continue;
        CLOSE[getNodeName(n)] = true;

        // check goal condition
        if (checkFocalFin(n))
        {
            invalid = false;
            break;
        }

        // expand
        Nodes C = n->v->neighbor;
        C.push_back(n->v);
        for (auto u : C)
        {
            int g_cost = n->g + 1;
            FocalNode *m = createNewNode(u, g_cost, 0, 0, n);
            // set heuristics
            m->f1 = f1Value(m);
            m->f2 = f2Value(m);
            // already searched?
            if (CLOSE.find(getNodeName(m)) != CLOSE.end())
                continue;
            // check constraints
            if (checkInvalidFocalNode(m))
                continue;
            // update open list
            OPEN.push(m);
            if (m->f1 <= f1_min * w)
                FOCAL.push(m);
        }
    }

    Path path;
    // success
    if (!invalid){
        path = getPathFromFocalNode(n);
    }else{
        printf("Cannot find a path\n");
    }
        
    // Pathret = std::make_tuple(path, f1_min);

    // free
    for (auto p : GC)
        delete p;

    return path;
        

}

