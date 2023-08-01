#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <cstdio>
#include <string>
#include <limits>
#include <stdexcept>

#include "Lidar.h"

namespace py = pybind11;

PYBIND11_MODULE(FastRPLidar, m)
{
    PYBIND11_NUMPY_DTYPE(lidar_sample, angle, distance, quality);
    PYBIND11_NUMPY_DTYPE(point, x, y);

    auto py_lidar = py::class_<PyLidar>(m, "RPLidar");

    py_lidar.def(py::init<std::string, int>(),
                 "Loads Lidar from given USB port at given baud rate",
                 py::arg("port"), py::arg("baud_rate"));

    py_lidar.def("check_health", &PyLidar::checkhealth, "Checks health. True is OK, False is not Ok");

    py_lidar.def("stop_motor", &PyLidar::stopmotor, "Stops the Lidar Motor");

    py_lidar.def("start_motor", &PyLidar::startmotor, "Starts the Lidar Motor");

    py_lidar.def("reset", &PyLidar::reset, "Resets the underlying lidar driver");

    py_lidar.def(
        "get_scanline_xy",
        [](PyLidar &self, bool filter_quality)
        {
            auto data = self.get_scan_as_xy(filter_quality);

            point *points = data.first;

            // Create a Python object that will free the allocated
            // memory when destroyed:
            py::capsule del_when_done(points, [](void *f)
                                      { delete[] (point *)f; });

            return py::array_t<point>(
                {data.second},                 // shape
                {data.second * sizeof(point)}, // C-style contiguous strides for double
                points,                        // the data pointer
                del_when_done                  // numpy array references this parent
            );
        },
        "Returns scan line in the form of x-y pairs with 0-0 as the lidar"
        // py::arg("self"),
        // py::arg("filter")
    );

    py_lidar.def(
        "get_scanline",
        [](PyLidar &self, bool filter_quality)
        {
            auto data = self.get_scan_as_lidar_samples(filter_quality);

            lidar_sample *samples = data.first;

            // Create a Python object that will free the allocated
            // memory when destroyed:
            py::capsule del_when_done(samples, [](void *f)
                                      { delete[] (lidar_sample *)f; });

            return py::array_t<lidar_sample>(
                {data.second},                 // shape
                {data.second * sizeof(point)}, // C-style contiguous strides
                samples,                       // the data pointer
                del_when_done                  // numpy array references this parent
            );
        },
        "Returns scan line in the form of x-y pairs with 0-0 as the lidar"
        // py::arg("self"), py::arg("filter_low_quality"

    );

    py_lidar.def_property_readonly("serial_number", &PyLidar::serial_number, "Device serial number");
    py_lidar.def_property_readonly("firmware_version", &PyLidar::firmware_version, "Device firmware_version");
    py_lidar.def_property_readonly("hardware_version", &PyLidar::hardware_version, "Device hardware_version");

      py_lidar.def("__str__", &PyLidar::to_string);
}