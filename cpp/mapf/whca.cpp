#include "whca.hpp"
#include <algorithm>
#include <random>

const std::string WHCA::SOLVER_NAME = "WHCA";
const int WHCA::DEFAULT_WINDOW = 10;
const int WHCA::DEFAULT_EXEC_WINDOW=5;


WHCA::WHCA(Problem *_P) : Solver(_P), table_goals(G->getNodesSize(), false)
{
    window = DEFAULT_WINDOW;
    exec_window=DEFAULT_EXEC_WINDOW;
    solver_name = SOLVER_NAME + "-" + std::to_string(window);
}

WHCA::WHCA(Graph *G,Config starts,Config goals,int max_comp_time)
    :Solver(G,starts,goals,max_comp_time),table_goals(G->getNodesSize(), false)
{
    window = DEFAULT_WINDOW;
    exec_window=DEFAULT_EXEC_WINDOW;
    solver_name = SOLVER_NAME + "-" + std::to_string(window);
}

void WHCA::run()
{
    // initialize

    Paths paths(P->getNum());
    
    for (int i = 0; i < P->getNum(); ++i)
    {
        paths.insert(i, {P->getStart(i)});
        table_goals[P->getGoal(i)->id] = true;
    }

    // initial prioritization, far agent is prioritized
    std::vector<int> ids(P->getNum());
    std::iota(ids.begin(), ids.end(), 0);

    // start planning
    int iteration = 0;
    while (true)
    {
        // info(" ", "elapsed:", getSolverElapsedTime(),
        //      ", timestep:", iteration * window);
        ++iteration;  
        // bool sorted=false;
        if (!disable_dist_init)
        {
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::shuffle (ids.begin(), ids.end(), std::default_random_engine(seed));
            // LS heuristic: short distance to go has lower priority
            // std::sort(ids.begin(), ids.end(),
            //         [&](int a, int b)
            //         { return pathDist(a, paths.last(a)) > pathDist(b, paths.last(b)); });
            // sorted=true;
            // std::sort(ids.begin(), ids.end(),
            //         [&](int a, int b)
            //         { return pathDist(a, paths.last(a)) < pathDist(b, paths.last(b)); });
            //  HS heuristic
        }
        bool check_goal_cond = true;
        Paths partial_paths(P->getNum());
        bool invalid = false;
  
      
        // if(!sorted){
        //     unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

        //     std::shuffle (ids.begin(), ids.end(), std::default_random_engine(seed));
        //     for(auto id:ids){
        //         std::cout<<id<<"  ";
        //     }
        //     std::cout<<std::endl;
        // }
        for (int j = 0; j < P->getNum(); ++j)
        {
            int i = ids[j];
            Node *s = *(paths.get(i).end() - 1);
            Node *g = P->getGoal(i);
            Path path;
            Path tmp_path = getPrioritizedPartialPath(i, s, g, partial_paths);
            // std::cout<<"tmp path size  ="<<tmp_path.size()<<std::endl;
            
            
            if (tmp_path.empty())
            { // failed
                invalid = true;
                // sorted=false;
                // std::cout<<"failed to find a path,current makespan="<<paths.getMakespan()<<std::endl;
                // goto shuffle;
                
                break;
            }
            for(int t=0;;t++){
                if(t>exec_window and t>=tmp_path.size()) break;
                path.push_back(tmp_path[i]);
            }

            // for(int i=0;i<=exec_window;i++)path.push_back(tmp_path[i]);
            path=tmp_path;
            partial_paths.insert(i, path);
            check_goal_cond &= (*(path.end() - 1) == g);
        }
        if (invalid){
            paths=Paths(P->getNum());
            break;
        }
        paths += partial_paths;

        // clear cache
        clearPathTable(partial_paths);

        // check goal condition
        if (check_goal_cond)
        {
            solved = true;
            break;
        }

        // check limitation
        if (overCompTime() || paths.getMakespan() > max_timestep)
        {   
            break;
        }
    }

    solution = pathsToPlan(paths);
}

void WHCA::setWindowSize(int window_size){
    this->window=window_size;
}

Path WHCA::getPrioritizedPartialPath(int id, Node *s, Node *g,
                                    const Paths &paths)
{
    const int makespan = paths.getMakespan();

    // pre processing
    int max_constraint_time = 0;
    for (int i = 0; i < P->getNum(); ++i)
    {
        Path p = paths.get(i);
        if (p.empty() || i == id)
            continue;
        for (int t = 0; t <= makespan; ++t)
        {
            if (paths.get(i, t) == g)
            {
                max_constraint_time = std::max(t, max_constraint_time);
            }
        }
    }

    

    // in this case, the greedy f-value fails a lot, different from HCA*
    AstarHeuristics fValue = [&](AstarNode *n)
    {
        return n->g + pathDist(id, n->v);
    };

    CompareAstarNode compare = [&](AstarNode *a, AstarNode *b)
    {
        if (a->f != b->f)
            return a->f > b->f;
        // tie-break, avoid goal locations of others
        if (a->v != g && table_goals[a->v->id])
            return true;
        if (b->v != g && table_goals[b->v->id])
            return false;
        if (a->g != b->g)
            return a->g < b->g;
        return false;
    };

    // different from HCA*
    CheckAstarFin checkAstarFin = [&](AstarNode *n)
    {
        return (n->v == g && n->g > max_constraint_time) || n->g >= window;
    };

    // fast collision checking
    CheckInvalidAstarNode checkInvalidAstarNode = [&](AstarNode *m)
    {
        if (m->g > window)
            return true;
        // last node
        if (makespan > 0)
        {
            if (m->g > makespan)
            {
                if (PATH_TABLE[makespan][m->v->id] != Solver::NIL)
                    return true;
            }
            else
            {
                // vertex conflict
                if (PATH_TABLE[m->g][m->v->id] != Solver::NIL)
                    return true;
                // swap conflict
                if (PATH_TABLE[m->g][m->p->v->id] != Solver::NIL &&
                    PATH_TABLE[m->g - 1][m->v->id] == PATH_TABLE[m->g][m->p->v->id])
                    return true;
            }
        }
        return false;
    };

    Path path = getPathBySpaceTimeAstar(s, g, fValue, compare, checkAstarFin, checkInvalidAstarNode, getRemainedTime());
    // std::cout<<"debug  "<<std::endl;
    // for(auto n:path){
    //     std::cout<<"("<<n->pos.x<<","<<n->pos.y<<"),";
    // }
    // std::cout<<std::endl;
    const int path_size = path.size();
    // format
    if (!path.empty() && path_size - 1 > window)
        path.resize(window + 1);

    // update path table
    updatePathTableWithoutClear(id, path, paths);

    return path;
}

void WHCA::setParams(int argc, char *argv[])
{
    struct option longopts[] = {
        {"window", required_argument, 0, 'w'},
        {"disable-dist-init", no_argument, 0, 'd'},
        {0, 0, 0, 0},
    };
    optind = 1; // reset
    int opt, longindex;
    while ((opt = getopt_long(argc, argv, "w:d", longopts, &longindex)) != -1)
    {
        switch (opt)
        {
        case 'w':
            window = std::atoi(optarg);
            if (window <= 0)
                halt("invalid window size.");
            solver_name = SOLVER_NAME + "-" + std::to_string(window);
            break;
        case 'd':
            disable_dist_init = true;
            break;
        default:
            break;
        }
    }
}

void WHCA::printHelp()
{
    std::cout << WHCA::SOLVER_NAME << "\n"
              << "  -w --window [INT]             "
              << "window size\n"
              << "  -d --disable-dist-init        "
              << "disable initialization of priorities "
              << "using distance from starts to goals" << std::endl;
}


std::tuple<Plan,bool,bool> WHCA::runOneStep(std::vector<double>priorities,int window_size,int exec_window){
    
    // std::cout<<"current position=("<<P->getStart(0)->pos.x<<","<<P->getStart(0)->pos.y<<")"<<std::endl;
    Paths paths(P->getNum());
    window=window_size;
    for(int i=0;i<P->getNum();i++){
        paths.insert(i,{P->getStart(i)});
        table_goals[P->getGoal(i)->id]=true;
    }
    std::vector<int> ids(P->getNum());
    std::iota(ids.begin(), ids.end(), 0);
    // initial prioritization, far agent is prioritized
    if(priorities.empty()==false){
        // std::sort(ids.begin(),ids.end(),
        //     [&](int a,int b)
        //     { return priorities[a]>priorities[b];});
        //    std::sort(ids.begin(), ids.end(),
        //             [&](int a, int b)
        //             { return pathDist(a, paths.last(a)) > pathDist(b, paths.last(b)); });

        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

        std::shuffle (ids.begin(), ids.end(), std::default_random_engine(seed));

    }
    else{
        if(!disable_dist_init){
            std::sort(ids.begin(), ids.end(),
                    [&](int a, int b)
                    { return pathDist(a, paths.last(a)) > pathDist(b, paths.last(b)); });
        }
    }
    bool check_goal_cond = true;
    Paths partial_paths(P->getNum());
    bool invalid = false;
    for (int j = 0; j < P->getNum(); ++j)
    {
        int i = ids[j];
        Node *s = *(paths.get(i).end() - 1);
        Node *g = P->getGoal(i);
        Path path = getPrioritizedPartialPath(i, s, g, partial_paths);
        if (path.empty())
        { // failed
            invalid = true;
           
            break;
        }
        // std::cout<<"????("<<s->pos.x<<","<<s->pos.y<<")   ("<<g->pos.x<<","<<g->pos.y<<")"<<std::endl;
        // for(auto &node:path){
        //     std::cout<<"("<<node->pos.x<<","<<node->pos.y<<"),";
        // }
        // std::cout<<"\n *******"<<std::endl;
        partial_paths.insert(i, path);
        check_goal_cond &= (*(path.end() - 1) == g);
    }
    if(invalid) return {Plan(),false,false};
    clearPathTable(partial_paths);
    return {pathsToPlan(partial_paths),true,check_goal_cond};

}



// used the HS for the imitation learning
std::vector<double>WHCA::getHeuristicPriority(){
    std::vector<double> p_values(P->getNum());
    std::iota(p_values.begin(), p_values.end(), 0);
    std::sort(p_values.begin(), p_values.end(),[&](int a, int b)
                    { return pathDist(a, P->getStart(a)) > pathDist(b,P->getGoal(b)); });
    for(int i=0;i<p_values.size();i++){
        p_values[i]/=P->getNum();
    }
    return p_values;

}