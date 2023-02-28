#include "hca.hpp"

const std::string HCA::SOLVER_NAME = "HCA";

HCA::HCA(Problem* _P)
    : Solver(_P),
      table_starts(G->getNodesSize(), false),
      table_goals(G->getNodesSize(), false)
{
  solver_name = HCA::SOLVER_NAME;
}


Plan HCA::solveWithTotalPriority(std::vector<int> priorities){
  Paths paths(P->getNum());
  // create tables for tie-break
  for (int i = 0; i < P->getNum(); ++i) {
    table_starts[P->getStart(i)->id] = true;
    table_goals[P->getGoal(i)->id] = true;
  }

  // prioritization, far agent is prioritized
  std::vector<int> ids(P->getNum());
  std::iota(ids.begin(), ids.end(), 0);
  if(priorities.empty()==false){
    std::sort(ids.begin(),ids.end(),
              [&](int a,int b){return priorities[a]>priorities[b];});
  }
  else{
    if(!disable_dist_init){
      std::sort(ids.begin(), ids.end(),
              [&](int a, int b) { return pathDist(a) > pathDist(b); });
    }
  }

  //start planning
  bool invalid = false;
  for (int j = 0; j < P->getNum(); ++j) {
    const int i = ids[j];
    // get path
    // auto t0=Time::now();
    Nodes path = getPrioritizedPath(i, paths);
    // auto t1=Time::now();
    // std::chrono::duration<float> dt=t1-t0;
    if (path.empty()) {  // failed
      invalid = true;
      break;
    }

    // update reservation table
    paths.insert(i, path);

    // check limitation
    if (overCompTime() || paths.getMakespan() > max_timestep) {
      invalid = true;
      break;
    }
  }

  if (!invalid) {  // success
    solution = pathsToPlan(paths);
    solved = true;
    return solution;
  }
  else{
    solved=false;
    return Plan();
  }
}


void HCA::run()
{
  Paths paths(P->getNum());

  // create tables for tie-break
  for (int i = 0; i < P->getNum(); ++i) {
    table_starts[P->getStart(i)->id] = true;
    table_goals[P->getGoal(i)->id] = true;
  }

  // prioritization, far agent is prioritized
  std::vector<int> ids(P->getNum());
  std::iota(ids.begin(), ids.end(), 0); //0,1,2,....
  if (!disable_dist_init) {
    //original shorter with higher priority
    std::sort(ids.begin(), ids.end(),
              [&](int a, int b) { return pathDist(a) > pathDist(b); });
    // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    // // std::shuffle (ids.begin(), ids.end(), std::default_random_engine(seed));
    // std::sort(ids.begin(), ids.end(),
    //           [&](int a, int b) { return pathDist(a) < pathDist(b); });
  }

  // start planning
  bool invalid = false;
  for (int j = 0; j < P->getNum(); ++j) {
    const int i = ids[j];
    info(" ", "elapsed:", getSolverElapsedTime(),
         ", agent-" + std::to_string(i), "starts planning,",
         "init-dist:", pathDist(i), ", progress:", j + 1, "/", P->getNum());

    // get path
    // auto t0=Time::now();
    Nodes path = getPrioritizedPath(i, paths);
    // auto t1=Time::now();
    // std::chrono::duration<float> dt=t1-t0;
   
    if (path.empty()) {  // failed
      invalid = true;
      break;
    }

    // update reservation table
    paths.insert(i, path);

    // check limitation
    if (overCompTime() || paths.getMakespan() > max_timestep) {
      invalid = true;
      break;
    }
  }

  if (!invalid) {  // success
    solution = pathsToPlan(paths);
    solved = true;
  }
}

// get single agent path
// failed -> return {}
Path HCA::getPrioritizedPath(int id, const Paths& paths)
{
  Node* s = P->getStart(id);
  Node* g = P->getGoal(id);

  Nodes config_s = P->getConfigStart();
  Nodes config_g = P->getConfigGoal();

  CompareAstarNode compare = [&](AstarNode* a, AstarNode* b) {
    if (a->f != b->f) return a->f > b->f;
    // tie-break, avoid goal locations of others
    if (a->v != g && table_goals[a->v->id]) return true;
    if (b->v != g && table_goals[b->v->id]) return false;
    // tie-break, avoid start locations
    if (a->v != s && table_starts[a->v->id]) return true;
    if (b->v != s && table_starts[b->v->id]) return false;
    if (a->g != b->g) return a->g < b->g;
    return false;
  };

  const auto p = Solver::getPrioritizedPath(id, paths, getRemainedTime(),
                                            max_timestep, {}, compare, false);

  // update path table
  updatePathTableWithoutClear(id, p, paths);

  return p;
}

void HCA::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"disable-dist-init", no_argument, 0, 'd'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "d", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'd':
        disable_dist_init = true;
        break;
      default:
        break;
    }
  }
}

void HCA::printHelp()
{
  std::cout << HCA::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals" << std::endl;
}


HCA::HCA(Graph *G,Config starts,Config goals, int max_comp_time)
    :Solver(G,starts,goals,max_comp_time),
    table_starts(G->getNodesSize(),false),
    table_goals(G->getNodesSize(),false){
  
  solver_name=HCA::SOLVER_NAME;

}
