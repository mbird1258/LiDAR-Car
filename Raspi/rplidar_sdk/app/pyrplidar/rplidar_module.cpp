#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "lidar_driver.hpp"

namespace py = pybind11;
using namespace sl;

PYBIND11_MODULE(rplidar, m) {
    py::class_<RPLidar>(m, "RPLidar")
        .def(py::init<>())
        .def("connect", &RPLidar::connect)
        .def("start_scan", &RPLidar::start_scan)
        .def("stop_scan", &RPLidar::stop_scan)
        .def("get_scan", &RPLidar::get_scan);
}