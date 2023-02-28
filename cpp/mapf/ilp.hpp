#pragma once
#include "gurobi_c++.h"
#include"solver.hpp"



static GRBEnv genv;

class IlpSolver:Solver
{
public:
    IlpSolver();
    ~IlpSolver();
    // void solve(OneShotTask &t, Graph &graph, size_t split = 0);
    
    GRBEnv *envp = &genv;

    typedef std::string id_type;

    
    


private:
    void run();
    void solve_original();
    void solve_original(Problem *p);
    void solve_split();
    GRBModel prepare_model(size_t time_steps);
    void retrive_paths(GRBModel &model, size_t time_steps, OneShotTask &t, Graph &graph);
    inline id_type get_id(size_t r, Node v1, Node v2, size_t t);
    inline void store_var_for_vertices(GRBVar &var, Node v1, Node v2, size_t t1, size_t t2);
    inline void store_var_for_robots(GRBVar &var, size_t r, Node v1, Node v2, size_t t1, size_t t2);
    inline void store_var_for_edges(GRBVar &var, Node v1, Node v2, size_t t);

    std::map<id_type, GRBVar> edge_var_map = std::map<id_type, GRBVar>();
    std::map<id_type, std::vector<GRBVar>> edge_time_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> time_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> time_out_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> robot_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> robot_out_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::vector<std::set<size_t>> reachability = std::vector<std::set<size_t>>();
    std::vector<std::vector<size_t>> individual_paths = std::vector<std::vector<size_t>>();
    std::vector<std::vector<Node>> final_paths;
};
