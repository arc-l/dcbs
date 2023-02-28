#pragma once
#include"solver.hpp"
#include <map>
#include "lib_cbs.hpp"
struct SubGraph{
    Pos lo;
    Pos hi;
    bool small=true;
    SubGraph(const Pos& lo_in, const Pos& hi_in) : lo(lo_in), hi(hi_in) {
        if (hi.x - lo.x + hi.y - lo.y == 4) small = false;
    };

    SubGraph(const SubGraph&that):
        lo(that.lo),hi(that.hi),small(that.small){};
    
    bool intersect_with(const SubGraph& that) {
        return !(hi.x < that.lo.x || that.hi.x < lo.x || hi.y < that.lo.y ||
                that.hi.y < lo.y);
    }

    bool contains(const Pos& that) {
        return (that.x >= lo.x && that.x <= hi.x && that.y >= lo.y &&
                that.y <= hi.y);
    }

    std::vector<Pos> get_nodes() const{
        int num_blocked=0;
        auto nodes = std::vector<Pos>();
        nodes.reserve((hi.x - lo.x + 1) * (hi.y - lo.y + 1));
        for (int i = lo.x; i <= hi.x; i++)
            for (int j = lo.y; j <= hi.y; j++) nodes.push_back(Pos(i, j));
        return nodes;
    }

};


struct Protected_Subgraph {
    SubGraph graph;              // The 2x3 graph
    std::vector<size_t> robots;  // Robots evolved in the graph
    int delay;  // Number of time steps before finishing collision resolution
    Protected_Subgraph(SubGraph& g_in, std::vector<size_t>& r_in, int d_in)
        : graph(g_in), robots(r_in), delay(d_in){};
};





class DDM:public Solver{

private:
    struct FocalNode
    {
        Node *v;      // location
        int g;        // in getTimedPath, g represents t
        double f1;    // used in open list
        double f2;    // used in focal list
        FocalNode *p; // parent
    };
    
    std::vector<std::unordered_map<std::string, std::string>>ddm_database;  // Solution database
    using CompareFocalNode = std::function<bool(FocalNode *, FocalNode *)>;
    using CheckFocalFin = std::function<bool(FocalNode *)>;
    using CheckInvalidFocalNode = std::function<bool(FocalNode *)>;
    using FocalHeuristics = std::function<double(FocalNode *)>;

    Path getTimedPathByFocalSearch(
        Node *const s, Node *const g, float w, // sub-optimality
        FocalHeuristics &f1Value, FocalHeuristics &f2Value,
        CompareFocalNode &compareOPEN, CompareFocalNode &compareFOCAL,
        CheckFocalFin &checkFocalFin,
        CheckInvalidFocalNode &checkInvalidFocalNode);
    void run();

public:

    DDM();
    DDM(Problem *);
    DDM(Grid *graph, Config starts,Config goals);
    void load_database();
    

    std::pair<Plan,bool> simulate(const Paths &initial_paths,int max_steps=300);
    int getNumExpansions(){return numConflictsResolved;}


private:
    std::vector<SubGraph>get_all_possible_2x3(const Pos  n1,const Pos n2,const Grid*g);
    Paths getInitialPaths();
    Paths getInitialPathsFocal();
    Path getInitialPath(int agent);
    Path getInitialPath(int agent,Node *start,Node *goal);
    Path getFocalPath(int agent,const Paths&paths);
    Path getFocalPath(int agent,Node *start, Node*goal,const Paths&paths);
    inline bool test_2x3_valid(const Grid* g, const SubGraph& subg);
    int max_steps_allowed=300;
    int numConflictsResolved=0;
    

};



/**
 * @brief Helper function to direct robot movement when they are prone to be
 * stopped
 *
 * @param flow:
 * @param stopped:
 * @param it:
 */
inline void recursive_adder(
    std::map<Node*, std::vector<std::pair<size_t, Node*>>>& flow,
    std::vector<size_t>& stopped,
    std::map<Node*, std::vector<std::pair<size_t, Node*>>,
                    std::hash<Node*>>::iterator it) {
    if (it == flow.end()) return;
    for (auto var : it->second) {
        if (find(stopped.begin(), stopped.end(), var.first) == stopped.end()) {
            stopped.push_back(var.first);
            recursive_adder(flow, stopped, flow.find(var.second));
        }
    }
}