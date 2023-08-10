#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <cstdio>
#include <string>
#include <limits>
#include <stdexcept>

#include "Lidar.h"

namespace py = pybind11;

PYBIND11_MODULE(FastPyRpLidar, m)
{
    /*
    PyLidar::RPLidar_Result_Code is an enum used to convey information about Lidar Health
    */
    auto rp_lidar_result_code = py::enum_<Lidar::RPLidar_Result_Code>(m, "Result_Code", "Result_Code is an enum used to convey information about Lidar Health")
                                    .value("OK", Lidar::RPLidar_Result_Code::OK)
                                    .value("FAIL_BIT", Lidar::RPLidar_Result_Code::FAIL_BIT)
                                    .value("ALREADY_DONE", Lidar::RPLidar_Result_Code::ALREADY_DONE)
                                    .value("INVALID_DATA", Lidar::RPLidar_Result_Code::INVALID_DATA)
                                    .value("OPERATION_FAIL", Lidar::RPLidar_Result_Code::OPERATION_FAIL)
                                    .value("OPERATION_TIMEOUT", Lidar::RPLidar_Result_Code::OPERATION_TIMEOUT)
                                    .value("OPERATION_STOP", Lidar::RPLidar_Result_Code::OPERATION_STOP)
                                    .value("OPERATION_NOT_SUPPORT", Lidar::RPLidar_Result_Code::OPERATION_NOT_SUPPORT)
                                    .value("FORMAT_NOT_SUPPORT", Lidar::RPLidar_Result_Code::FORMAT_NOT_SUPPORT)
                                    .value("INSUFFICIENT_MEMORY", Lidar::RPLidar_Result_Code::INSUFFICIENT_MEMORY)
                                    .value("UNKNOWN", Lidar::RPLidar_Result_Code::UNKNOWN);

    /*
    PyLidar::RPLidar_Status_Code is an enum used to convey information about Lidar Health
    */
    auto rp_lidar_status_code = py::enum_<Lidar::RPLidar_Status_Code>(m, "Status_Code", "Status_Code is an enum used to convey information about Lidar Health")
                                    .value("OK", Lidar::RPLidar_Status_Code::OK)
                                    .value("WARNING", Lidar::RPLidar_Status_Code::WARNING)
                                    .value("ERROR", Lidar::RPLidar_Status_Code::ERROR)
                                    .value("UNKNOWN", Lidar::RPLidar_Status_Code::UNKNOWN);
    /*
    Lidar::lidar_sample is a POD struct that exposes angle, distance and quality in a standare IEEE double in known units
    angle: degrees
    distance: meters
    quality: scale from 0-255
    */
   constexpr const char* LIDAR_SCAN_DOCSTRING =
    R"myDelim(A POD struct that exposes angle, distance and quality in a format with known units.
    angle: degrees
    distance: meters
    quality: scale from 0-255
    )myDelim";
    PYBIND11_NUMPY_DTYPE(Lidar::lidar_sample, angle, distance, quality);
    auto py_lidar_scan_struct = py::class_<Lidar::lidar_sample>(m, "Lidar_Scan", LIDAR_SCAN_DOCSTRING);
    py_lidar_scan_struct.def_property_readonly(
        "angle", [](Lidar::lidar_sample &self)
        { return self.angle; },
        "Angle in degrees");
    py_lidar_scan_struct.def_property_readonly(
        "distance", [](Lidar::lidar_sample &self)
        { return self.distance; },
        "Distance in meters");
    py_lidar_scan_struct.def_property_readonly(
        "quality", [](Lidar::lidar_sample &self)
        { return self.quality; },
        "The quality of the datapoint on a scale of [0,255]");


    /*
    Lidar::point is a POD struct that exposes x, y and quality in a standare IEEE double in known units
    Note, lidar is at the origin. 
    Another way to think about this struct is as a vector with given x-y components streaching from the origin

    x: meters from lidar
    y: meters from lidar
    quality: scale from 0-255
    */
   constexpr const char* POINT_DOCSTRING = 
    R"myDelim(A POD struct that exposes x, y and quality in a standard format with known units given that the lidar is at the origin (0,0). 
        Another way to think about this struct is as a vector with given x-y components streaching from the origin
        x: meters from lidar
        y: meters from lidar
        quality: scale from 0-255
    )myDelim";
    PYBIND11_NUMPY_DTYPE(Lidar::point, x, y, quality);
    auto py_lidar_point_struct = py::class_<Lidar::point>(m, "Point", POINT_DOCSTRING);
    py_lidar_point_struct.def_property_readonly(
        "x", [](Lidar::point &self)
        { return self.x; },
        "X location with respect to the lidar in meters");
    py_lidar_point_struct.def_property_readonly(
        "y", [](Lidar::point &self)
        { return self.y; },
        "Y location with respect to the lidar in meters");
    py_lidar_point_struct.def_property_readonly(
        "quality", [](Lidar::point &self)
        { return self.quality; },
        "The quality of the datapoint on a scale of [0,255]");
    
    /*
    Lidar is a class that encapsulates basic functionality of a RPLidar
    */
    auto py_lidar = py::class_<Lidar>(m, "RPLidar", "A class responsible to connecting to a Slamtek RPLidar over a serial connection.");

    constexpr const char * PY_LIDAR_INIT_DOCSTRING =
    R"myDelim(Loads Lidar over a serial connection from given USB port at given baud rate

    :param port: A OS specific USB port that is connected to a Lidar. Ex: /dev/ttyUSB0 (Linux and OSX), com3 (Windows)
    :type port: str
    :param baud_rate: The baudrate at which to conduct communications. Eg 1000000 (S2 Lidar), 115200 (A2)
    :type baud_rate: 32 bit unsigned int
    :raises OverflowError: If any parameter passed cannot be converted to the propper C++ type resulting in an overflow
    :raises RuntimeError: If establishing communication with the lidar fails

    )myDelim";
    py_lidar.def(py::init<std::string, int>(),
                 "Loads Lidar over a serial connection from given USB port at given baud rate",
                 py::arg("port"), py::arg("baud_rate"), PY_LIDAR_INIT_DOCSTRING);

    constexpr const char* START_MOTOR_DOC_STRING = 
    R"myDelim(Starts the lidar motor spinning

    :raises RuntimeError: If communication with the lidar fails
    )myDelim";
    py_lidar.def("start_motor", &Lidar::start_motor, START_MOTOR_DOC_STRING);

    constexpr const char* STOP_MOTOR_DOC_STRING = 
    R"myDelim(Stops the lidar motor from spinning

    :raises RuntimeError: If communication with the lidar fails
    )myDelim";
    py_lidar.def("stop_motor", &Lidar::stop_motor, STOP_MOTOR_DOC_STRING);

    constexpr const char* RESET_LIDAR_DOC_STRING = 
    R"myDelim(Resets the underlying lidar driver

    :raises RuntimeError: If communication with the lidar fails
    )myDelim";
    py_lidar.def("reset", &Lidar::reset, RESET_LIDAR_DOC_STRING);

    constexpr const char* GET_SCANLINE_X_Y_DOC_STRING = 
    R"myDelim(Returns scan line in the form of x-y pairs with (0-0) as the lidar. Units are in meters. Points are in sequential order so that index 0 corresponds to the first point taken by the lidar, and index 1 corresponds to the second point taken by the lidar.
    :raises RuntimeError: If communication with the lidar fails
    :return: One scan line consisting of a full revolution of the lidar
    :rtype: numpy.ndarray[Point]
    )myDelim";
    py_lidar.def(
        "get_scanline_xy",
        [](Lidar &self)
        {
            auto data = self.get_scan_as_xy();

            // Create a Python object that will free the allocated
            // memory when destroyed:
            py::capsule del_when_done(data.first, [](void *f)
                                      { delete[] (Lidar::point *)f; });

            return py::array_t<Lidar::point>(
                {data.second},            // shape
                {sizeof(Lidar::point)}, // C-style contiguous strides for double
                data.first,               // the data pointer
                del_when_done             // numpy array references this parent
            );
        },
        GET_SCANLINE_X_Y_DOC_STRING);

    constexpr const char* GET_SCANLINE_DOC_STRING = 
    R"myDelim(Returns scan line in the native lidar data format. Angle is in degrees. Distance is in meters. Samples are in sequential order so that index 0 corresponds to the first point taken by the lidar, and index 1 corresponds to the second point taken by the lidar.
    :raises RuntimeError: If communication with the lidar fails
    :return: One scan line consisting of a full revolution of the lidar
    :rtype: numpy.ndarray[Lidar_Scan]
    )myDelim";
    py_lidar.def(
        "get_scanline",
        [](Lidar &self)
        {
            auto data = self.get_scan_as_lidar_samples();

            // Create a Python object that will free the allocated
            // memory when destroyed:
            py::capsule del_when_done(data.first, [](void *f)
                                      { delete[] (Lidar::lidar_sample *)f; });

            return py::array_t<Lidar::lidar_sample>(
                {data.second},                   // shape
                {sizeof(Lidar::lidar_sample)}, // C-style contiguous strides
                data.first,                      // the data pointer
                del_when_done                    // numpy array references this parent
            );
        },
        GET_SCANLINE_DOC_STRING
    );

    py_lidar.def_property_readonly("serial_number", &Lidar::serial_number, "Device serial number");
    py_lidar.def_property_readonly("firmware_version", &Lidar::firmware_version, "Device firmware_version");
    py_lidar.def_property_readonly("hardware_version", &Lidar::hardware_version, "Device hardware_version");
    py_lidar.def_property_readonly("mac_address", &Lidar::mac_addr, "Device mac address");
    
    py_lidar.def("get_health", &Lidar::get_health, "Returns the health of the Lidar");

    py_lidar.def("__str__", &Lidar::to_string);
}
