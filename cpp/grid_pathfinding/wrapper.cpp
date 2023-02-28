#include"pos.hpp"
#include"node.hpp"
#include "graph.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
namespace py = pybind11;




PYBIND11_MODULE(Grids, m ){
    m.doc() = "pybind11 example";
    pybind11::class_<Pos>(m, "Pos")
        .def(pybind11::init<int,int>())
        .def_readwrite("x",&Pos::x)
        .def_readwrite("y",&Pos::y)
        .def("print", &Pos::print)
        .def("println",&Pos::println)
        .def("manhattanDist",&Pos::manhattanDist,"manhattan distance",py::arg("pos"))
        .def("euclideanDist",&Pos::euclideanDist,"Euclidean distance",py::arg("pos"))
        .def(py::self +py::self)
        .def(py::self += py::self)
        .def(py::self - py::self)
    
        // .def(py::self * py::self)
        .def(py::self -= py::self)
        // .def(py::self *= py::self)
        .def(py::self == py::self);

    pybind11::class_<Node>(m, "Node")
        .def(pybind11::init<int,int,int>())
        .def("getDegree", &Node::getDegree)
        .def("println",&Node::println)
        .def_readonly("pos",&Node::pos)
        .def_readonly("id",&Node::id)
        .def_readonly("neighbor",&Node::neighbor)
        .def("print",&Node::print)
        
        // .def("manhattanDist", py::overload_cast<const Node &>(&Node::manhattanDist), "Manhattan distance")
        // .def("manhattanDist", py::overload_cast<const Node * const>(&Node::manhattanDist), "Manhattan distance")
        // .def("euclideanDist", py::overload_cast<const Node &>(&Node::euclideanDist), "Euclidean distance")
        // .def("euclideanDist", py::overload_cast<const Node * const>(&Node::euclideanDist), "Euclidean distance")
        .def(py::self != py::self)
        // .def(py::self - py::self)
        // .def(py::self * py::self)
        .def(py::self == py::self);

    pybind11::class_<Graph>(m,"Graph")
        .def(pybind11::init())
        .def("getNodesSize",&Graph::getNodesSize,"get nodes size")
        .def("getV",&Graph::getV,"get all nodes",py::return_value_policy::reference);

    pybind11::class_<Grid,Graph>(m, "Grid")
        .def(pybind11::init())
        .def(pybind11::init<const std::string &>())
        .def(pybind11::init<int,int,std::vector<std::vector<int>>>())
        .def(pybind11::init<std::vector<std::vector<int>>&>())
        .def(pybind11::init<int,int,double>())
        .def("getNode",py::overload_cast<int>(&Grid::getNode,py::const_),"get node",py::return_value_policy::reference)
        .def("getNode",py::overload_cast<int,int>(&Grid::getNode,py::const_),"get node",py::return_value_policy::reference)
        .def("existNode",py::overload_cast<int>(&Grid::existNode,py::const_),"whether node exist")
        .def("existNode",py::overload_cast<int,int>(&Grid::existNode,py::const_),"whether node exist")
        .def("getWidth",&Grid::getWidth)
        .def("getBinaryMap",&Grid::getBinaryMap,"get binary map")
        .def("generateRandomConfig",&Grid::generateRandomConfig,"random configuration",py::return_value_policy::reference)
        .def("getHeight",&Grid::getHeight);

}