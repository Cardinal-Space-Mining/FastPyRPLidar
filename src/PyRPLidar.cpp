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
    PYBIND11_NUMPY_DTYPE(PyLidar::lidar_sample, angle, distance, quality);
    PYBIND11_NUMPY_DTYPE(PyLidar::point, x, y);

    auto rp_lidar_result_code = py::enum_<PyLidar::RPLidar_Result_Code>(m, "Result_Code", "Lidar result enum")
                                    .value("OK", PyLidar::RPLidar_Result_Code::OK)
                                    .value("FAIL_BIT", PyLidar::RPLidar_Result_Code::FAIL_BIT)
                                    .value("ALREADY_DONE", PyLidar::RPLidar_Result_Code::ALREADY_DONE)
                                    .value("INVALID_DATA", PyLidar::RPLidar_Result_Code::INVALID_DATA)
                                    .value("OPERATION_FAIL", PyLidar::RPLidar_Result_Code::OPERATION_FAIL)
                                    .value("OPERATION_TIMEOUT", PyLidar::RPLidar_Result_Code::OPERATION_TIMEOUT)
                                    .value("OPERATION_STOP", PyLidar::RPLidar_Result_Code::OPERATION_STOP)
                                    .value("OPERATION_NOT_SUPPORT", PyLidar::RPLidar_Result_Code::OPERATION_NOT_SUPPORT)
                                    .value("FORMAT_NOT_SUPPORT", PyLidar::RPLidar_Result_Code::FORMAT_NOT_SUPPORT)
                                    .value("INSUFFICIENT_MEMORY", PyLidar::RPLidar_Result_Code::INSUFFICIENT_MEMORY)
                                    .value("UNKNOWN", PyLidar::RPLidar_Result_Code::UNKNOWN);

    auto rp_lidar_status_code = py::enum_<PyLidar::RPLidar_Status_Code>(m, "Status_Code", "Lidar status enum")
                                    .value("OK", PyLidar::RPLidar_Status_Code::OK)
                                    .value("WARNING", PyLidar::RPLidar_Status_Code::WARNING)
                                    .value("ERROR", PyLidar::RPLidar_Status_Code::ERROR)
                                    .value("UNKNOWN", PyLidar::RPLidar_Status_Code::UNKNOWN);

    auto py_lidar_scan_struct = py::class_<PyLidar::lidar_sample>(m, "Lidar_Scan");
    py_lidar_scan_struct.def_property_readonly(
        "angle", [](PyLidar::lidar_sample &self)
        { return self.angle; },
        "Angle in degrees");
    py_lidar_scan_struct.def_property_readonly(
        "distance", [](PyLidar::lidar_sample &self)
        { return self.distance; },
        "Distance in meters");
    py_lidar_scan_struct.def_property_readonly(
        "quality", [](PyLidar::lidar_sample &self)
        { return self.quality; },
        "Measurement quality (0 ~ 255). Higher is better");

    auto py_lidar_point_struct = py::class_<PyLidar::point>(m, "Point");
    py_lidar_point_struct.def_property_readonly(
        "x", [](PyLidar::point &self)
        { return self.x; },
        "X location with respect to the lidar in meters");
    py_lidar_point_struct.def_property_readonly(
        "y", [](PyLidar::point &self)
        { return self.y; },
        "Y location with respect to the lidar in meters");

    auto py_lidar = py::class_<PyLidar>(m, "RPLidar");

    py_lidar.def(py::init<std::string, int>(),
                 "Loads Lidar over a serial connection from given USB port at given baud rate",
                 py::arg("port"), py::arg("baud_rate"));

    py_lidar.def("start_motor", &PyLidar::start_motor, "Starts the Lidar motor and starts scanning procedures");

    py_lidar.def("stop_motor", &PyLidar::stop_motor, "Stops the Lidar Motor");

    py_lidar.def("reset", &PyLidar::reset, "Resets the underlying lidar driver");

    py_lidar.def(
        "get_scanline_xy",
        [](PyLidar &self, bool filter_quality)
        {
            auto data = self.get_scan_as_xy(filter_quality);

            // Create a Python object that will free the allocated
            // memory when destroyed:
            py::capsule del_when_done(data.first, [](void *f)
                                      { delete[] (PyLidar::point *)f; });

            return py::array_t<PyLidar::point>(
                {data.second},            // shape
                {sizeof(PyLidar::point)}, // C-style contiguous strides for double
                data.first,               // the data pointer
                del_when_done             // numpy array references this parent
            );
        },
        "Returns scan line in the form of x-y pairs with 0-0 as the lidar",
        py::arg("filter"));

    py_lidar.def(
        "get_scanline",
        [](PyLidar &self, bool filter_quality)
        {
            auto data = self.get_scan_as_lidar_samples(filter_quality);

            // Create a Python object that will free the allocated
            // memory when destroyed:
            py::capsule del_when_done(data.first, [](void *f)
                                      { delete[] (PyLidar::lidar_sample *)f; });

            return py::array_t<PyLidar::lidar_sample>(
                {data.second},                   // shape
                {sizeof(PyLidar::lidar_sample)}, // C-style contiguous strides
                data.first,                      // the data pointer
                del_when_done                    // numpy array references this parent
            );
        },
        "Returns scan line in the form of x-y pairs with 0-0 as the lidar", py::arg("filter_low_quality")

    );

    py_lidar.def_property_readonly("serial_number", &PyLidar::serial_number, "Device serial number");
    py_lidar.def_property_readonly("firmware_version", &PyLidar::firmware_version, "Device firmware_version");
    py_lidar.def_property_readonly("hardware_version", &PyLidar::hardware_version, "Device hardware_version");
    py_lidar.def("get_health", &PyLidar::get_health, "Returns the health of the Lidar");

    py_lidar.def("__str__", &PyLidar::to_string);
}