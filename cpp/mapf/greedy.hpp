#pragma once
#include "solver.hpp"
#include <map>
#include <set>


Config greedy(Grid *graph,Config starts,double preferred_density);

Config a_star_greedy(Grid* graph,Config starts, Config goals, double preferred_density);