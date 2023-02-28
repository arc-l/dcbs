#pragma once
#include "problem.hpp"


/*
 * array of configurations
 */

struct Plan
{
private:
    Configs configs; // main

public:
    ~Plan() {}

    // timestep -> configuration
    Config get(const int t) const;

    // timestep, agent -> location
    Node *get(const int t, const int i) const;

    // path
    Path getPath(const int i) const;

    Plan reverse();

    // path cost
    int getPathCost(const int i) const;

    void printPath(const int t);

    // last configuration
    Config last() const;

    // last configuration
    Node *last(const int i) const;

    std::vector<std::vector<int>> lastVec();

    // become empty
    void clear();

    // add new configuration to the last
    void add(const Config &c);

    // whether configs are empty
    bool empty() const;

    // configs.size
    int size() const;

    // size - 1
    int getMakespan() const;

    // sum of cost
    int getSOC() const;

    // join with other plan
    Plan operator+(const Plan &other) const;
    void operator+=(const Plan &other);

    // check the plan is valid or not
    bool validate(Problem *P) const;
    bool validate(const Config &starts, const Config &goals) const;

    // when updating a single path,
    // the path should be longer than this value to avoid conflicts
    int getMaxConstraintTime(const int id, Node *s, Node *g, Graph *G) const;
    int getMaxConstraintTime(const int id, Problem *P) const;

    // error
    void halt(const std::string &msg) const;
    void warn(const std::string &msg) const;


    static Plan concat(const Plan &p1, const Plan &p2);
};

using Plans = std::vector<Plan>;
