#include "cbdfs.hpp"
#include <cassert>
#include <set>
const float DEFAULT_SUB_OPTIMALITY = 1.50;
CBDFS::CBDFS(Problem *p) : Solver(p)
{
    sub_optimality = DEFAULT_SUB_OPTIMALITY;
}

CBDFS::CBDFS(Grid *graph, Config starts, Config goals) : Solver(graph, starts, goals)
{
    sub_optimality = DEFAULT_SUB_OPTIMALITY;
}

double CBDFS::cal_pref_density(HighLevelNode_p node)
{
    // preferred density
    int num_agents = P->getNum();
    double density_avg = num_agents / (P->getG()->getNodesSize());
    density_pref = 1.5 * density_avg;
    return density_pref;
}


void CBDFS::set_suboptimality(float w){
    this->sub_optimality=w;
}

double CBDFS::cal_priority(HighLevelNode_p node)
{
    // cal sum of diff agent density
    int makespan = node->paths.getMakespan();
    int num_agents = P->getNum();
    int scale = num_agents * num_agents * (P->getG()->getWidth() * P->getG()->getHeight());
    // int scale=num_agents*num_agents*()
    double sum_diff_density = 0;
    double result;
    density_pref=0;
    double lambda1 = 1, lambda2 = 1.0 / scale, lambda3 = 0.1;
    for (int t = 0; t < makespan; t++)
    {
        Config config_t;
        for (int i = 0; i < num_agents; i++)
        {
            config_t.push_back(node->paths.get(i, t));
        }
        for (int i = 0; i < num_agents; i++)
        {
            double local_density_i = cal_local_density(config_t, i);
            // sum_diff_density += relu(local_density_i, density_pref);
            sum_diff_density += local_density_i;
        }
    }
    // std::cout<<"sum diff density="<<sum_diff_density<<std::endl;
    // return lambda1 * node->paths.getSOC() + lambda2 * sum_diff_density + lambda3 * node->paths.countConflict();
    return sum_diff_density;
}

double CBDFS::cal_local_density(Config config, int agent_id)
{
    std::set<Node *> occupied;
    auto curr = config[agent_id];
    // run BFS to get area of FOV
    std::queue<Node *> open;
    std::set<Node *> closed;
    open.push(curr);
    double nodes_count = 0;
    while (open.empty() == false)
    {
        auto nxt = open.front();
        open.pop();
        closed.insert(nxt);
        nodes_count++;
        auto nbrs = nxt->neighbor;
        for (auto nbr : nbrs)
        {
            if (closed.find(nbr) != closed.end())
                continue;
            if (nbr->manhattanDist(curr) <= obs_radius)
                open.push(nbr);
        }
    }
    double agent_in_fov = 0;
    // get agents in the field of view
    for (int j = 0; j < config.size(); j++)
    {
        if (j == agent_id)
            continue;
        if (config[j]->manhattanDist(curr) <= obs_radius)
            agent_in_fov++;
    }
    return agent_in_fov / nodes_count;
}

std::function<bool(CBDFS::HighLevelNode_p a, CBDFS::HighLevelNode_p b)> CBDFS::tie_breaker_comparator()
{
    auto compare = [](HighLevelNode_p a, HighLevelNode_p b)
    {
        if (a->density_diff != b->density_diff)
            return a->density_diff > b->density_diff;
        if (a->soc != b->soc)
            return a->soc > b->soc;
        if (a->f != b->f)
            return a->f > b->f; // tie-breaker
                                // return a->priority > b->priority;
    };
    return compare;
}

CBDFS::CompareHighLevelNode CBDFS::getMainObjective()
{
    CompareHighLevelNode compare = [&](HighLevelNode_p a, HighLevelNode_p b)
    {
        if (a->LB != b->LB)
            return a->LB > b->LB;
        return false;
    };
    return compare;
}

CBDFS::CompareHighLevelNode CBDFS::getFocalObjective()
{
    CompareHighLevelNode compare = [&](HighLevelNode_p a, HighLevelNode_p b)
    {
        if (a->f != b->f)
            return a->f > b->f;
        if (a->soc != b->soc)
            return a->soc > b->soc;
        return false;
    };
    return compare;
}


// CBDFS::CompareHighLevelNode CBDFS::getFocalObjective()
// {
//     CompareHighLevelNode compare = [&](HighLevelNode_p a, HighLevelNode_p b)
//     {
        
//         if (a->f != b->f)
//             return a->f > b->f;
//         if(a->priority!=b->priority)
//             return a->priority>b->priority;
//         if (a->soc != b->soc)
//             return a->soc > b->soc;
//         return false;
//     };
//     return compare;
// }


void CBDFS::setInitialHighLevelNodeECBS(HighLevelNode_p n)
{
    Paths paths(P->getNum());
    // std::cout<<"num robots= "<<P->getNum()<<std::endl;
    std::vector<int> f_mins; // vector of costs for respective paths
    for (int i = 0; i < P->getNum(); ++i)
    {
        Path path = getInitialPathECBS(i, paths);
    
        paths.insert(i, path);
        f_mins.push_back(path.size() - 1);
    }
    // std::cout<<"debug??? "<<paths.size()<<"  "<<paths.getMakespan()<<std::endl;
    n->paths = paths;
    n->constraints = {};
    n->makespan = paths.getMakespan();
    n->soc = paths.getSOC();
    n->f = paths.countConflict();
    // n->f=CountWeightedConflicts(n->paths);
    n->valid = true;
    n->f_mins = f_mins;
    n->LB = n->soc; // initial lower bound
    //
    // n->priority=cal_priority(n);

}

void CBDFS::run()
{
    // high-level search
    CompareHighLevelNode compareOPEN = getMainObjective();
    CompareHighLevelNode compareFOCAL = getFocalObjective();

    // OPEN, FOCAL
    std::priority_queue<HighLevelNode_p, std::vector<HighLevelNode_p>,
                        CompareHighLevelNode>
        OPEN(compareOPEN);
    using FocalList =
        std::priority_queue<HighLevelNode_p, std::vector<HighLevelNode_p>,
                            CompareHighLevelNode>;
    FocalList FOCAL(compareFOCAL);

    // initial node
    HighLevelNode_p n = std::make_shared<HighLevelNode>();
    setInitialHighLevelNode(n);
    // setInitialHighLevelNodeECBS(n);


    OPEN.push(n);
    FOCAL.push(n);
    int LB_min = n->LB;

    // main loop
    int h_node_num = 1;
    node_expanded = 0;
    while (!OPEN.empty())
    {
        ++node_expanded;
        // check limitation
        if (overCompTime()){
            std::cout<<"run out of time"<<std::endl;
            break;
        }
        /*
         *  update focal list
         */
        // pop out invalid nodes
        while (!OPEN.empty() && !OPEN.top()->valid)
            OPEN.pop();
        // failure
        if (OPEN.empty())
            break;
        // when lower bound is updated
        if (LB_min != OPEN.top()->LB)
        {
            // new lower bound
            LB_min = OPEN.top()->LB;
            // admissible f-value in open list
            float LB_bound = LB_min * sub_optimality;
            // for escape
            std::vector<HighLevelNode_p> tmp;
            // clear focal list
            FocalList EMPTY(compareFOCAL);
            FOCAL = EMPTY;
            // insert nodes to focal list
            while (!OPEN.empty())
            {
                HighLevelNode_p top = OPEN.top();
                OPEN.pop();
                // already searched
                if (!top->valid)
                    continue;
                // escape
                tmp.push_back(top);
                // higher than LB_bound
                if ((float)top->LB > LB_bound)
                    break;
                // lower than LB_bound
                FOCAL.push(top);
            }
            // back
            for (auto ele : tmp)
                OPEN.push(ele);
        }

        // pickup one node
        assert(FOCAL.empty() == false);
        n = FOCAL.top();

        FOCAL.pop();
        n->valid = false; // closed

        info(" ", "elapsed:", getSolverElapsedTime(),
            ", explored_node_num:", node_expanded, ", nodes_num:", h_node_num,
            ", conflicts:", n->f, ", constraints:", n->constraints.size(),
            ", soc:", n->soc);
        std::cout<<std::endl;
        // check conflict
        LibCBS::Constraints constraints = LibCBS::getFirstConstraints(n->paths);
        // LibCBS::Constraints constraints=LibCBS::getConstraintsWithDensityControl(n->paths);
        if (constraints.empty())
        {
            // std::cout<<"nodes expanded="<<node_expanded<<std::endl;
            solved = true;
            break;
        }
        // if(n->f==1){
        //     break;
        //     solution = pathsToPlan(n->paths);

        // }

        // create new nodes
        for (auto c : constraints)
        {
            LibCBS::Constraints new_constraints = n->constraints;
            new_constraints.push_back(c);
            HighLevelNode_p m = std::make_shared<HighLevelNode>(
                n->paths, new_constraints, n->makespan, n->soc, n->f, n->LB,
                n->f_mins, true);
            invoke(m, c->id);
            if (!m->valid)
                continue;
            OPEN.push(m);
            if (m->LB <= LB_min * sub_optimality)
                FOCAL.push(m);
            ++h_node_num;
        }
    }

    // success
    if (solved)
        solution = pathsToPlan(n->paths);
}

Path CBDFS::getInitialPath(int id)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);
    Nodes config_g = P->getConfigGoal();

    Path path = {s};
    Node *p = s;
    while (p != g)
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

    return path;
}

Path CBDFS::getInitialPathECBS(int id, const Paths &paths)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);
    Nodes config_g = P->getConfigGoal();

    Path path = {s};
    Node *p = s;
    int t = 1;
    const int makespan = paths.getMakespan();
    const int num_agents = P->getNum();
    while (p != g)
    {
        p = *std::min_element(p->neighbor.begin(), p->neighbor.end(),
                              [&](Node *a, Node *b)
                            {
                                if (pathDist(id, a) != pathDist(id, b))
                                    return pathDist(id, a) < pathDist(id, b);
                                if (t <= makespan)
                                {
                                    Node *v;
                                    for (int i = 0; i < num_agents; ++i)
                                    {
                                        if (paths.empty(i))
                                            continue;
                                        v = paths.get(i, t);
                                        if (v == a)
                                            return false;
                                        if (v == b)
                                            return true;
                                    }
                                }
                                if (a != g && inArray(a, config_g))
                                    return false;
                                if (b != g && inArray(b, config_g))
                                    return true;
                                return false;
                            });
        path.push_back(p);
        ++t;
    }

    return path;
}

Path CBDFS::getInitialPathWithDensityControl(int id, const Paths &paths)
{
    // to do change to the method with density control
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);
    Nodes config_g = P->getConfigGoal();

    Path path = {s};
    Node *p = s;
    int t = 1;
    const int makespan = paths.getMakespan();
    const int num_agents = P->getNum();
    while (p != g)
    {
        p = *std::min_element(p->neighbor.begin(), p->neighbor.end(),
                              [&](Node *a, Node *b)
                            {
                                if (pathDist(id, a) != pathDist(id, b))
                                    return pathDist(id, a) < pathDist(id, b);
                                if (t <= makespan)
                                {
                                    Node *v;
                                    for (int i = 0; i < num_agents; ++i)
                                    {
                                        if (paths.empty(i))
                                            continue;
                                        v = paths.get(i, t);
                                        if (v == a)
                                            return false;
                                        if (v == b)
                                            return true;
                                    }
                                }
                                if (a != g && inArray(a, config_g))
                                    return false;
                                if (b != g && inArray(b, config_g))
                                    return true;
                                return false;
                            });
        path.push_back(p);
        ++t;
    }

    return path;
}


void CBDFS::invoke(HighLevelNode_p h_node, int id)
{
    auto res = getFocalPath(h_node, id);
    Path path = std::get<0>(res);
    int f_min = std::get<1>(res); // lower bound

    // failed to find path
    if (path.empty())
    {
        h_node->valid = false;
        return;
    }
    Paths paths = h_node->paths;
    paths.insert(id, path);
    // it is efficient to reuse past data
    h_node->f = h_node->f -
                h_node->paths.countConflict(id, h_node->paths.get(id)) +
                h_node->paths.countConflict(id, paths.get(id));


    // h_node->f=h_node->f-CountWeightedConflicts(id,h_node->paths,h_node->paths.get(id))
    //     +CountWeightedConflicts(id,h_node->paths,paths.get(id));
    
    // h_node->f=CountWeightedConflicts(h_node->paths);

    h_node->paths = paths;
    // h_node->priority=cal_priority(h_node);
    h_node->makespan = h_node->paths.getMakespan();
    h_node->soc = h_node->paths.getSOC();
    // update lower bound and f_min
    h_node->LB = h_node->LB - h_node->f_mins[id] + f_min;
    h_node->f_mins[id] = f_min;
}

std::tuple<Path, int> CBDFS::getFocalPath(HighLevelNode_p h_node, int id)
{
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);

    // pre processing
    LibCBS::Constraints constraints;
    int max_constraint_time = 0;
    for (auto c : h_node->constraints)
    {
        if (c->id == id)
        {
            constraints.push_back(c);
            if (c->v == g && c->u == nullptr)
            {
                max_constraint_time = std::max(max_constraint_time, c->t);
            }
        }
    }

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

    const auto paths = h_node->paths;
    const int makespan = paths.getMakespan();

    // update PATH_TABLE
    auto start = std::chrono::system_clock::now();
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

    FocalHeuristics densityControl = [&](FocalNode *n)
    {
        // std::cout<<"density control  "<<std::endl;
        if(n->g==0) return 0.0;
        Config next_config;
        int makespan = h_node->makespan;
        for (int i = 0; i < P->getNum(); i++)
        {
            if (n->g + 1 > makespan)
                next_config.push_back(h_node->paths.get(i, makespan));
            else
                next_config.push_back(h_node->paths.get(i, n->g + 1));
        }
        double local_density = cal_local_density(next_config, id);
        double lambda = 0.1;
        return local_density + n->p->f2;
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
    // clear used path table
    clearPathTable(paths);

    return p;
}

Path CBDFS::getPathFromFocalNode(CBDFS::FocalNode *_n)
{
    Path path;
    FocalNode *n = _n;
    while (n != nullptr)
    {
        path.push_back(n->v);
        n = n->p;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// return path and f_min
std::tuple<Path, int> CBDFS::getTimedPathByFocalSearch(
    Node *const s, Node *const g,
    float w, // sub-optimality
    FocalHeuristics &f1Value, FocalHeuristics &f2Value,
    CompareFocalNode &compareOPEN, CompareFocalNode &compareFOCAL,
    CheckFocalFin &checkFocalFin, CheckInvalidFocalNode &checkInvalidFocalNode)
{
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
        if (overCompTime())
            break;

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
    if (!invalid)
        path = getPathFromFocalNode(n);
    std::tuple<Path, int> ret = std::make_tuple(path, f1_min);

    // free
    for (auto p : GC)
        delete p;

    return ret;
}




void CBDFS::setInitialHighLevelNode(HighLevelNode_p n)
{
    // find initial paths
    std::cout<<"set initial highlevel node  "<<std::endl;
    Paths paths(P->getNum());
    std::vector<int> f_mins; 
    LibCBS::Constraints constraints;
    std::vector<int> ids(P->getNum());
    std::iota(ids.begin(), ids.end(), 0);
    // std::sort(ids.begin(), ids.end(),
    //         [&](int a, int b)
    //         { return pathDist(a, P->getGoal(a)) > pathDist(b, P->getGoal(b)); });
    
    for (auto i:ids)
    {
        auto data = getFocalPath(i, paths,constraints);
        auto path=std::get<0>(data);
        auto fmin=std::get<1>(data);
        // std::cout<<"path size for agent "<<i<<"  =  "<<path.size()<<std::endl;
        paths.insert(i, path);
        f_mins.push_back(fmin);
    }

    n->paths = paths;
    n->constraints = {};
    n->makespan = paths.getMakespan();
    n->soc = paths.getSOC();
    n->f = paths.countConflict();
    n->valid = true;
    n->f_mins = f_mins;
    n->LB = n->soc; // initial lower bound
}



std::tuple<Path, int>  CBDFS::getFocalPath(int id, Paths paths,const LibCBS::Constraints&constraints){
    Node *s = P->getStart(id);
    Node *g = P->getGoal(id);
    int max_constraint_time=0;
    for(auto c:constraints){
        if(c->id==id){
            if (c->v == g && c->u == nullptr)
            {
                max_constraint_time = std::max(max_constraint_time, c->t);
            }

        }
    }

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
    int makespan=paths.getMakespan();
    updatePathTable(paths,id);

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
    // clear used path table
    clearPathTable(paths);

    return p;
}


/// @brief 
/// @param id 
/// @param paths 
/// @param path 
/// @return 
double CBDFS::CountWeightedConflicts(int id, const Paths &paths,const Path &path){
    double cnt = 0;
    int makespan = paths.getMakespan();
    int num_agents = paths.size();
    const int path_size = path.size();
    std::vector<double>densities(num_agents);
    

    for (int t = 1; t < path_size; ++t)
    {
        Config current;
        for(int i=0;i<num_agents;i++){
            if(i==id){
                if(t>=path.size())
                    current.push_back(path.back());
                else   
                    current.push_back(path[t]);
                continue;
            }
            if(t>makespan)
                current.push_back(paths.get(i,makespan));
            else
                current.push_back(paths.get(i,t));
        }
        for(int i=0;i<num_agents;i++){
            densities[i]=cal_local_density(current,i);
        }
        for (int i = 0; i < num_agents; ++i)
        {
            if (i == id)
                continue;
        
            if (t > makespan)
            {
                if (path[t] == paths.get(i, makespan))
                {
                    cnt+=0.5*(densities[i]+densities[id]);
                    break;
                }
                continue;
            }
            // vertex conflict
            if (paths.get(i, t) == path[t])
            {
                // ++cnt;
                cnt+=0.5*(densities[i]+densities[id]);
                continue;
            }
            // swap conflict
            if (paths.get(i, t) == path[t - 1] && paths.get(i, t - 1) == path[t]){
                // ++cnt;
                cnt+=0.5*(densities[i]+densities[id]);
            }
                
        }
    }
    return cnt;
}


/// @brief 
/// @param paths 
/// @return 
double CBDFS::CountWeightedConflicts(const Paths &paths){
    double cnt = 0;
    int makespan = paths.getMakespan();
    int num_agents = paths.size();
    std::vector<double> densities(num_agents);
    for(int t=1;t<makespan;t++){
        Config current;
        for(int i=0;i<num_agents;i++){
            current.push_back(paths.get(i,t));
        }
        for(int i=0;i<num_agents;i++){
            densities[i]=cal_local_density(current,i);
        }
        for(int i=0;i<num_agents;i++){
            for(int j=i+1;j<num_agents;j++){
                if(paths.get(i,t)==paths.get(j,t)){
                    cnt+=0.5*(densities[i]+densities[j]);
                }
                if(paths.get(i,t-1)==paths.get(j,t) and paths.get(i,t)==paths.get(j,t-1)){
                    cnt+=0.5*(densities[i]+densities[j]);
                }
            }
        }
    }
    return cnt;
}