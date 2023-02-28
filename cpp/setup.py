from setuptools import setup, Extension
import pybind11
from pybind11.setup_helpers import Pybind11Extension


cpp_args=["-std=c++17"]

# functions_module = Extension(
#     name='Grids',
#     sources=['./grid_pathfinding/wrapper.cpp',
#             './grid_pathfinding/pos.cpp',
#             './grid_pathfinding/graph.cpp',
#             './grid_pathfinding/node.cpp'],
#     include_dirs=[pybind11.get_include()],
#     language='c++',
#     extra_compile_args=cpp_args,
# )

functions_module = Extension(
    name='MAPF',
    sources=['./grid_pathfinding/graph.cpp',
            './grid_pathfinding/pos.cpp',
            './grid_pathfinding/node.cpp',
            './mapf/paths.cpp', 
            './mapf/wrapper.cpp',
            './mapf/solver.cpp',
            './mapf/plan.cpp',
            './mapf/problem.cpp',
            './mapf/lib_cbs.cpp',
            './mapf/whca.cpp',
            './mapf/hca.cpp'
            ],
    include_dirs=[pybind11.get_include()],
    language='c++',
    extra_compile_args=cpp_args,
)

setup(ext_modules=[functions_module])
