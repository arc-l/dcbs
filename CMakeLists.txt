cmake_minimum_required(VERSION 3.16)
project(mapf_python)

# find_package(Python 3.6 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 3.6 EXACT REQUIRED COMPONENTS Interpreter Development)
find_package(PythonInterp 3.6 EXACT REQUIRED)
find_package(pybind11 REQUIRED)
set(CMAKE_CXX_STANDARD 20 CACHE STRING "C++ version selection")  # or 11, 14, 17, 20

# set(CMAKE_BUILD_TYPE Debug)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0  -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")

ADD_LIBRARY(GRAPHLIB SHARED 
            "./cpp/grid_pathfinding/graph.cpp"
            "./cpp/grid_pathfinding/node.cpp"
            "./cpp/grid_pathfinding/pos.cpp")
            
ADD_LIBRARY(MAPFLIB SHARED
        "./cpp/mapf/paths.cpp"
        "./cpp/mapf/solver.cpp"
        "./cpp/mapf/whca.cpp"
        "./cpp/mapf/plan.cpp"
        "./cpp/mapf/problem.cpp"
        "./cpp/mapf/hca.cpp"
        "./cpp/mapf/lib_cbs.cpp"
        "./cpp/mapf/ecbs.cpp"
        "./cpp/mapf/cbdfs.cpp"
        "./cpp/mapf/ddm.cpp"
        "./cpp/mapf/ddecbs.cpp"
        "./cpp/mapf/greedy.cpp"
        "./cpp/mapf/maxflow.cpp"
        # "./cpp/mapf/ilp.cpp"
)

set(ORTOOLS_LIB "/usr/local/lib/libortools.so")
set(GUROBI_INC /opt/gurobi912/linux64/include/)
# set(GUROBI_LIB "/opt/gurobi912/linux64/lib/libgurobi_c++.a" "/opt/gurobi912/linux64/lib/libgurobi91.so")

# pybind11_add_module(Grids "./cpp/grid_pathfinding/wrapper.cpp" 
#                         "./cpp/grid_pathfinding/graph.cpp"
#                         "./cpp/grid_pathfinding/node.cpp"
#                         "./cpp/grid_pathfinding/pos.cpp")
pybind11_add_module(Grids  "./cpp/grid_pathfinding/wrapper.cpp")
target_link_libraries(Grids PRIVATE GRAPHLIB)

pybind11_add_module(MAPF "./cpp/mapf/wrapper.cpp") 
target_link_libraries(MAPF PRIVATE MAPFLIB GRAPHLIB ${ORTOOLS_LIB})                     
# target_link_libraries(MAPF PRIVATE MAPFLIB GRAPHLIB ${ORTOOLS_LIB} ${GUROBI_LIB})


# add_executable(main "./cpp/mapf/main.cpp")
# target_link_libraries(main PRIVATE MAPFLIB GRAPHLIB)




# pybind11_add_module(ORCA 
#         "./cpp/RVO2/src/KdTree.cpp"
#         "./cpp/RVO2/src/Obstacle.cpp"
#         "./cpp/RVO2/src/RVOSimulator.cpp"
#         "./cpp/RVO2/src/Agent.cpp"
#         "./cpp/mapf/ORCA.cpp")
# target_include_directories(ORCA PRIVATE "./cpp/RVO2/src/")
# target_link_libraries(ORCA PRIVATE GRAPHLIB)
# find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
# include_directories( ${Boost_INCLUDE_DIRS} )



