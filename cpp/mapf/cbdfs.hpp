#pragma once
#include "lib_cbs.hpp"
#include "solver.hpp"
#include <tuple>
#include <memory>

class CBDFS : public Solver
{
protected:
    int obs_radius = 3;
    double density_pref;
    struct HighLevelNode
    {
        Paths paths;
        LibCBS::Constraints constraints;
        int makespan;
        int soc;
        double f;                   // num of conflicts
        int LB;                  // lower bound
        std::vector<int> f_mins; // f_mins value in the low-level search
        bool valid;
        double density_diff = 0;
        double priority = 0;

        HighLevelNode() {}
        HighLevelNode(Paths _paths, LibCBS::Constraints _c, int _m, int _soc,
                    double _f, int _LB, std::vector<int> _f_mins, bool _valid)
            : paths(_paths),
            constraints(_c),
            makespan(_m),
            soc(_soc),
            f(_f),
            LB(_LB),
            f_mins(_f_mins),
            valid(_valid)
        {
        }
    };

    // used in the low-level search
    struct FocalNode
    {
        Node *v;      // location
        int g;        // in getTimedPath, g represents t
        double f1;    // used in open list
        double f2;    // used in focal list
        FocalNode *p; // parent
    };

public:
    void set_suboptimality(float w);
    using HighLevelNode_p = std::shared_ptr<HighLevelNode>;
    using CompareHighLevelNode = std::function<bool(HighLevelNode_p, HighLevelNode_p)>;

protected:
    CompareHighLevelNode getFocalObjective();
    Path getInitialPath(int id, const Paths &paths);

    CompareHighLevelNode getMainObjective();

    void invoke(HighLevelNode_p h_node, int id);
    void setInitialHighLevelNode(HighLevelNode_p n);
    void setInitialHighLevelNodeECBS(HighLevelNode_p n);
    void setInitialHighLevelNodeWithDensityControl(HighLevelNode_p n);
    Path getInitialPath(int id);
    Path getInitialPathECBS(int id, const Paths &paths);
    Path getInitialPathECBS(int id);
    Path getInitialPathWithDensityControl(int id, const Paths &paths);

    double cal_pref_density(HighLevelNode_p node);
    double cal_priority(HighLevelNode_p node);
    Path getConstrainedPath(HighLevelNode_p h_node, int id);
    Path getPathFromFocalNode(FocalNode *_n);
    std::tuple<Path, int> getFocalPath(int id, Paths paths,const LibCBS::Constraints &constraints);

    void run();
    double cal_local_density(Config config, int agent_id);

    using CompareFocalNode = std::function<bool(FocalNode *, FocalNode *)>;
    using CheckFocalFin = std::function<bool(FocalNode *)>;
    using CheckInvalidFocalNode = std::function<bool(FocalNode *)>;
    using FocalHeuristics = std::function<double(FocalNode *)>;
    float sub_optimality;

    std::tuple<Path, int> getFocalPath(HighLevelNode_p h_node, int id);
    std::tuple<Path, int> getTimedPathByFocalSearch(
        Node *const s, Node *const g, float w, // sub-optimality
        FocalHeuristics &f1Value, FocalHeuristics &f2Value,
        CompareFocalNode &compareOPEN, CompareFocalNode &compareFOCAL,
        CheckFocalFin &checkFocalFin,
        CheckInvalidFocalNode &checkInvalidFocalNode);

    std::function<bool(HighLevelNode_p, HighLevelNode_p)> tie_breaker_comparator();

private:
    inline double relu(double density, double density_pref)
    {
        if (density < density_pref)
            return 0;
        return density;
    }
    int node_expanded;
    
    double CountWeightedConflicts(const Paths &paths);
    double CountWeightedConflicts(int id,const Paths&paths,const Path &path);

    /* data */
public:
    CBDFS(Problem *p);
    CBDFS(Grid *graph, Config starts, Config goals);

    int getNumExpansions()
    {
        return node_expanded;
    }
};
