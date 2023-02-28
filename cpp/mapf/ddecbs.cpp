#include "ddecbs.hpp"

DDECBS::DDECBS(Problem *problem) : ECBS(problem)
{
    ddm_planner = std::make_shared<DDM>(problem);
    ddm_planner->load_database();
    ddm_planner->setDistanceTable(&distance_table);
}


DDECBS::DDECBS(Grid *graph, Config starts, Config goals) : ECBS(graph, starts, goals)
{
    ddm_planner = std::make_shared<DDM>(graph, starts, goals);
    ddm_planner->load_database();
    ddm_planner->setDistanceTable(&distance_table);
}

void DDECBS::run()
{
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
    OPEN.push(n);
    FOCAL.push(n);
    int LB_min = n->LB;

    // main loop
    int h_node_num = 1;
    numExpansions = 0;
    int last_num_conflicts=n->f;
    if(proportion>0){
        preferred_num_conflicts=n->f*proportion;
    }
    int count_stuck=0;
    auto t_start = Time::now();
    // std::cout<<"running DDECBS"<<std::endl;
    while (!OPEN.empty())
    {
        ++numExpansions;

        // check limitation
        // if (overCompTime())
        //     break;
        auto t_end=Time::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()>60000) break;

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
        n = FOCAL.top();
        FOCAL.pop();
        n->valid = false; // closed

        info(" ", "elapsed:", getSolverElapsedTime(),
            ", explored_node_num:", numExpansions, ", nodes_num:", h_node_num,
            ", conflicts:", n->f, ", constraints:", n->constraints.size(),
            ", soc:", n->soc);
        std::cout<<std::endl;

        //check if need to start ddm resolve?
    
        // check conflict
        LibCBS::Constraints constraints = LibCBS::getFirstConstraints(n->paths);
        if (constraints.empty())
        {
            solved = true;
            break;
        }
        if(abs(n->f-last_num_conflicts)<=3){
            count_stuck++;
        }
        else{
            count_stuck=0;
            last_num_conflicts=n->f;
        }

        if(count_stuck>=500||start_ddm_standard(n)==true){
            if(stagnation_iter==0)stagnation_iter= numExpansions;
            // return;
            count_stuck=0;
            std::cout<<"start repairing, number of conflicts="<<n->f<<std::endl;
            auto success=ddm_repair_procedure(n);
            if(success==true) return;
        }


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

bool DDECBS::ddm_repair_procedure(ECBS::HighLevelNode_p node)
{
    // printf("DEBUG ddm repair procedure started!\n");
    auto result=ddm_planner->simulate(node->paths);
    if(result.second==false) return false;
    else{
        auto resolved_plan=result.first;
        if(optimality_requirements(node,resolved_plan)==true){
            solution=resolved_plan;
            // std::cout<<"solved!"<<std::endl;
            return true;
        }
        else{
            return false;
        }

    }
}


bool DDECBS::start_ddm_standard(ECBS::HighLevelNode_p node){
    if(node->f<=preferred_num_conflicts) return true;
    return false;
}

bool DDECBS::optimality_requirements(ECBS::HighLevelNode_p node,const Plan& plan){
    // int fLB=0;
    // for(auto fi:node->f_mins){
    //     fLB+=fi;
    // }
    if(plan.getMakespan()<2*getLowerBoundMakespan()){
        return true;
    }
    else 
        return false;
}