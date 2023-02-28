#pragma once

#include "ecbs.hpp"
#include "ddm.hpp"

class DDECBS:public ECBS{
public:
    DDECBS(Problem*);
    DDECBS(Grid* graph, Config starts,Config goals);
    void setPreferredNOC(int NOC){ preferred_num_conflicts=NOC;}
    int get_stagnation(){
        // if(stagnation_iter==0) return numExpansions;
        // return stagnation_iter;
        return numExpansions;
    }
    

private:
    std::shared_ptr<DDM> ddm_planner;
    void run();
    int preferred_num_conflicts=20;
    double proportion=-1;
    bool ddm_repair_procedure(HighLevelNode_p node);
    bool optimality_requirements(HighLevelNode_p node,const Plan&);
    bool start_ddm_standard(HighLevelNode_p node);
    int stagnation_iter=0;
};