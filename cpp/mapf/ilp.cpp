#include "ilp.hpp"

IlpSolver::IlpSolver()
{
}

IlpSolver::~IlpSolver()
{
}

void IlpSolver::solve(OneShotTask &task, Graph &graph, size_t split)
{
    if (split == 0)
        solve_original(task, graph);
    else
        solve_split();
}

void IlpSolver::solve_original(OneShotTask &task, Graph &graph)
{
    // auto path_planner =
    //     SingleRobotPathPlanner<OmniDirectionalRobot::State,
    //                            OmniDirectionalRobot::Action,
    //                            OmniDirectionalRobot::Environment>(graph);
    // auto path_lengths = std::vector<size_t>();
    // // First step: calculate paths for each robot independently
    // for (size_t i = 0; i < task.num_robots; i++)
    //     path_lengths.push_back(path_planner.reversed_search(
    //                                            task.starts[i], task.goals[i])
    //                                .size());
    // Second step: Solve ILP
    // size_t time_step = *std::max_element(path_lengths.begin(), path_lengths.end()) - 1;
    // std::cout<<graph.adj_list[Node(0,0)].size()<<std::endl;
    int time_step=0;
    while (true)
    {
        GRBModel model = prepare_model(time_step, task, graph);
        model.optimize();
        double obj_val = model.get(GRB_DoubleAttr_ObjVal);
        // std::cout<<"Current timestep="<<time_step<<"    "<<obj_val<<std::endl;
        if (obj_val == task.num_robots)
        {
            // std::cout<<"SOlution found"<<std::endl;
            retrive_paths(model, time_step, task, graph);
            return;
        }
        
        time_step += 1;
    }
}

void IlpSolver::solve_split()
{
    // //cout << "Called" << endl;
    // Single_Path_Finder path_finder = Single_Path_Finder();
    // path_finder.g = p->g;
    // std::vector<std::vector<size_t>> individual_paths = std::vector<std::vector<size_t>>();
    // std::vector<std::pair<size_t, size_t>> path_lengths = std::vector<std::pair<size_t, size_t>>();
    // size_t max_length = 0;
    // for (size_t i = 0; i < task.num_robots; i++)
    // {
    //     size_t nsz = path_finder.solve_single(task.starts[i], task.goals[i]);
    //     path_lengths.push_back(make_pair(i, nsz));
    //     individual_paths.push_back(path_finder.path);
    // }
    // std::sort(path_lengths.begin(), path_lengths.end(), [](auto &left, auto &right) { return left.second > right.second; });
    // std::vector<size_t> middle_config = std::vector<size_t>(task.num_robots, p->g->x_size * p->g->y_size + 1);
    // for each (auto var in path_lengths)
    // {
    //     size_t i = var.first;
    //     size_t desired_pt = individual_paths[i][individual_paths[i].size() / 2];
    //     std::vector<size_t> pool = std::vector<size_t>();
    //     auto it = find(middle_config.begin(), middle_config.end(), desired_pt);
    //     while (it != middle_config.end())
    //     {
    //         pool.insert(pool.end(), p->g->neighbors[desired_pt].begin(), p->g->neighbors[desired_pt].end());
    //         desired_pt = pool[0];
    //         pool.erase(pool.begin());
    //         it = find(middle_config.begin(), middle_config.end(), desired_pt);
    //     }
    //     middle_config[i] = desired_pt;
    // }
    // //for each (auto var in middle_config)
    // //	cout << var << " ";
    // //cout << endl;
    // IlpSolver solver_1 = IlpSolver();
    // IlpSolver solver_2 = IlpSolver();
    // Problem p1 = Problem();
    // Problem p2 = Problem();
    // p1.g = p->g;
    // p2.g = p->g;
    // p1.num_robots = task.num_robots;
    // p2.num_robots = task.num_robots;
    // p1.starts = task.starts;
    // p1.goals = middle_config;
    // p2.starts = middle_config;
    // p2.goals = task.goals;
    // solver_1.p = &p1;
    // solver_2.p = &p2;
    // solver_1.split = split - 1;
    // solver_2.split = split - 1;
    // solver_1.start_time = start_time;
    // solver_2.start_time = start_time;
    // if (split <= 4)
    // {
    //     std::thread t1(&IlpSolver::solve_helper, &solver_1);
    //     std::thread t2(&IlpSolver::solve_helper, &solver_2);
    //     t1.join();
    //     t2.join();
    // }
    // else
    // {
    //     solver_1.solve_helper();
    //     solver_2.solve_helper();
    // }
    // //cout << "Combine paths" << endl;
    // final_paths = solver_1.final_paths;
    // final_paths.pop_back();
    // final_paths.insert(final_paths.end(), solver_2.final_paths.begin(), solver_2.final_paths.end());
}

GRBModel IlpSolver::prepare_model(size_t time_steps, OneShotTask &task, Graph &graph)
{
    bool relaxed = false;
    // auto start_time = std::chrono::high_resolution_clock::now();
    edge_var_map = std::map<id_type, GRBVar>();
    edge_time_vector_map = std::map<id_type, std::vector<GRBVar>>();
    time_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    time_out_vector_map = std::map<id_type, std::vector<GRBVar>>();
    robot_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    robot_out_vector_map = std::map<id_type, std::vector<GRBVar>>();
    GRBModel model = GRBModel(*envp);
    model.set(GRB_IntParam_OutputFlag, 0);
    // Set variables
    std::set<Node> reachable_vertices;
    std::set<Node> new_vertices;
    for (size_t r = 0; r < task.num_robots; r++)
    {
        reachable_vertices = std::set<Node>();
        reachable_vertices.insert(task.starts[r]);
        for (size_t t = 1; t < time_steps + 1; t++)
        {
            new_vertices = std::set<Node>();
            for (auto n : reachable_vertices)
            {
                for (auto nbr : graph.adj_list[n])
                {
                    // Check goal reachability
                    if (get_manhattan_distance(nbr, task.goals[r]) > time_steps - t)
                        continue;
                    new_vertices.insert(nbr);
                    id_type id = get_id(r, n, nbr, t - 1);
                    GRBVar x = model.addVar(0.0, 1.0, 0.0, relaxed ? GRB_CONTINUOUS : GRB_BINARY, id);
                    edge_var_map.insert(std::make_pair(id, x));
                    store_var_for_vertices(x, n, nbr, t - 1, t);
                    store_var_for_robots(x, r, n, nbr, t - 1, t);
                    store_var_for_edges(x, n, nbr, t - 1);
                }
                // Check goal reachability
                if (get_manhattan_distance(n, task.goals[r]) > time_steps - t)
                    continue;
                new_vertices.insert(n);
                id_type id = get_id(r, n, n, t - 1);
                GRBVar x = model.addVar(0.0, 1.0, 0.0, relaxed ? GRB_CONTINUOUS : GRB_BINARY, id);
                edge_var_map.insert(std::make_pair(id, x));
                store_var_for_vertices(x, n, n, t - 1, t);
                store_var_for_robots(x, r, n, n, t - 1, t);
            }
            reachable_vertices = new_vertices;
        }
    }
    // Set objective
    GRBLinExpr obj_expr = GRBLinExpr();
    for (size_t r = 0; r < task.num_robots; r++)
    {
        id_type id = get_id(r, task.goals[r], task.starts[r], time_steps);
        GRBVar x;
        x = model.addVar(0.0, 1.0, 0.0, relaxed ? GRB_CONTINUOUS : GRB_BINARY, id);
        edge_var_map.insert(std::make_pair(id, x));
        store_var_for_vertices(x, task.goals[r], task.starts[r], time_steps, 0);
        store_var_for_robots(x, r, task.goals[r], task.starts[r], time_steps, 0);
        obj_expr += x;
        //model.addConstr(x, GRB_EQUAL, 1);
    }
    model.setObjective(obj_expr, GRB_MAXIMIZE);
    // Set constraints
    for (size_t t = 0; t <= time_steps; t++)
        for (size_t v = 0; v < graph.nodes.size(); v++)
        {
            auto it = time_in_vector_map.find(get_id(99999, graph.nodes[v], Node(), t));
            if (it == time_in_vector_map.end())
                continue;
            GRBLinExpr expr = GRBLinExpr();
            for (size_t i = 0; i < it->second.size(); i++)
                expr += it->second[i];
            if (expr.size() > 1)
                model.addConstr(expr, GRB_LESS_EQUAL, 1);
        }
    for (size_t r = 0; r < task.num_robots; r++)
    {
        for (size_t t = 0; t <= time_steps; t++)
        {
            for (size_t v = 0; v < graph.nodes.size(); v++)
            {
                GRBLinExpr expr = GRBLinExpr();
                auto it = robot_in_vector_map.find(get_id(r, graph.nodes[v], Node(), t));
                if (it != robot_in_vector_map.end())
                    for (size_t i = 0; i < it->second.size(); i++)
                        expr -= it->second[i];
                it = robot_out_vector_map.find(get_id(r, graph.nodes[v], Node(), t));
                if (it != robot_out_vector_map.end())
                    for (size_t i = 0; i < it->second.size(); i++)
                        expr += it->second[i];
                if (expr.size() > 0)
                    model.addConstr(expr, GRB_EQUAL, 0);
            }
        }
    }
    for (auto it = edge_time_vector_map.begin(); it != edge_time_vector_map.end(); it++)
    {
        GRBLinExpr expr = GRBLinExpr();
        for (size_t i = 0; i < it->second.size(); i++)
            expr += it->second[i];
        if (expr.size() > 1)
            model.addConstr(expr, GRB_LESS_EQUAL, 1);
    }
    //model.write("m.lp");
    model.update();
    return model;
}

void IlpSolver::retrive_paths(GRBModel &model, size_t time_steps, OneShotTask &task, Graph &graph)
{
    auto vars = model.getVars();
    final_paths = std::vector<std::vector<Node>>(task.num_robots, std::vector<Node>(time_steps + 1, Node()));
    for (size_t r = 0; r < task.num_robots; r++)
    {
        //std::cout<<" Robot "<<r<<" #####################"<<std::endl;
        final_paths[r][0] = task.starts[r];
        for (size_t t = 1; t < time_steps + 1; t++)
        {
            //std::cout<<" timestep "<<t<<std::endl;
            Node v1 = final_paths[r][t - 1];
            std::vector<Node> nbrs = graph.adj_list[v1];
            nbrs.push_back(v1);
            bool feasible=false;
            int num_feasible=0;
            for (size_t i = 0; i < nbrs.size(); i++)
            {
                try
                {
                    if (fabs(model.getVarByName(get_id(r, v1, nbrs[i], t - 1)).get(GRB_DoubleAttr_X) -1.0)<1e-2)
                    {
                       // std::cout<<"neighbor "<<i<<"  "<<nbrs[i]<<"  size="<<nbrs.size()<<"  variable = "<<model.getVarByName(get_id(r, v1, nbrs[i], t - 1)).get(GRB_DoubleAttr_X)<<std::endl;
                        final_paths[r][t] = nbrs[i];
                        num_feasible++;
                        feasible=true;
                        break;
                    }
                    else{
                        //std::cout<<"neighbor  "<<nbrs[i]<<" var name="<<get_id(r, v1, nbrs[i], t - 1)<<" variable value= "<<model.getVarByName(get_id(r, v1, nbrs[i], t - 1)).get(GRB_DoubleAttr_X)<<std::endl;
                    }
                }
                catch (const GRBException &)
                {
                    // std::cout<<"GRBE exception"<<std::endl;
                    continue;
                }
            
            }
            // std::cout<<"current vertex is "<<v1<<std::endl;
            // std::cout<<"num feasible ="<<num_feasible<<std::endl;
            assert(num_feasible==1);
            assert(feasible);
            
        }
        // if(final_paths[r].back()!=task.goals[r])
        //     std::cout<<final_paths[r].back()<<"    "<<task.goals[r]<<std::endl;
        // assert(final_paths[r].back()==task.goals[r]);
    }
}

inline IlpSolver::id_type IlpSolver::get_id(size_t r, Node v1, Node v2, size_t t)
{
    return std::to_string(v2.x) + "-" + std::to_string(v2.y) + "-" + std::to_string(v1.x) + "-" + std::to_string(v1.y) + "-" + std::to_string(r) + "-" + std::to_string(t);
}

inline void IlpSolver::store_var_for_vertices(GRBVar &var, Node v1, Node v2, size_t t1, size_t t2)
{
    id_type id = get_id(99999, v1, Node(), t1);
    auto it = time_out_vector_map.find(id);
    if (it == time_out_vector_map.end())
        time_out_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    time_out_vector_map.find(id)->second.push_back(var);
    id = get_id(99999, v2, Node(), t2);
    it = time_in_vector_map.find(id);
    if (it == time_in_vector_map.end())
        time_in_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    time_in_vector_map.find(id)->second.push_back(var);
}

inline void IlpSolver::store_var_for_robots(GRBVar &var, size_t r, Node v1, Node v2, size_t t1, size_t t2)
{
    id_type id = get_id(r, v1, Node(), t1);
    auto it = robot_out_vector_map.find(id);
    if (it == robot_out_vector_map.end())
        robot_out_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    robot_out_vector_map.find(id)->second.push_back(var);
    id = get_id(r, v2, Node(), t2);
    it = robot_in_vector_map.find(id);
    if (it == robot_in_vector_map.end())
        robot_in_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    robot_in_vector_map.find(id)->second.push_back(var);
}

inline void IlpSolver::store_var_for_edges(GRBVar &var, Node v1, Node v2, size_t t)
{
    id_type id = get_id(99999, v1, v2, t);
    if (v2 < v1)
        id = get_id(99999, v2, v1, t);
    auto it = edge_time_vector_map.find(id);
    if (it == edge_time_vector_map.end())
        edge_time_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    edge_time_vector_map.find(id)->second.push_back(var);
}
