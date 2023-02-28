/*
 * Implementation of Hierarchical Cooperative A* (HCA*)
 *
 * - ref
 * Silver, D. (2005).
 * Cooperative pathfinding.
 * In AIIDE’05 Proceedings of the First AAAI Conference on Artificial
 * Intelligence and Interactive Digital Entertainment (pp. 117–122).
 *
 */

#pragma once
#include "solver.hpp"

class WHCA : public Solver
{
public:
    static const std::string SOLVER_NAME;

private:
    int window; // window size
    static const int DEFAULT_WINDOW;
    static const int DEFAULT_EXEC_WINDOW;
    int exec_window;

    // option
    bool disable_dist_init = false;

    Path getPrioritizedPartialPath(int id, Node *s, Node *g, const Paths &paths);

    void run();

    // used for tie-break
    std::vector<bool> table_goals;

public:
    WHCA(Problem *_P);
    WHCA(Graph *G,Config starts,Config goals,int max_comp_time=300);
    //   WHCA(){}
    ~WHCA(){};
    // plan,deadlock_free, done
    std::tuple<Plan,bool,bool> runOneStep(std::vector<double> priorities={},int window_size=DEFAULT_WINDOW,int exec_window_size=DEFAULT_WINDOW);
    void setParams(int argc, char *argv[]);
    void setWindowSize(int window_size);
    std::vector<double> getHeuristicPriority();
    static void printHelp();
};
