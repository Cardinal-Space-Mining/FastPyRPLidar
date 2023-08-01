#include <array>     //std::array
#include <cmath>     //M_PI
#include <stdexcept> //std::runtime_error, std::invalid_argument, std::bad_alloc
#include <utility>
#include <vector>
#include <memory>
#include <cstring>

#include "Lidar.h"

using namespace rp::standalone::rplidar;

constexpr double deg2rad = M_PI / 180.0;

using std::size_t;

namespace
{
    const char *to_string(u_result res)
    {
        switch (res)
        {
        case RESULT_OK:
            return "RESULT_OK";
        case RESULT_FAIL_BIT:
            return "RESULT_FAIL_BIT";
        case RESULT_ALREADY_DONE:
            return "RESULT_ALREADY_DONE";
        case RESULT_INVALID_DATA:
            return "RESULT_INVALID_DATA";
        case RESULT_OPERATION_FAIL:
            return "RESULT_OPERATION_FAIL";
        case RESULT_OPERATION_TIMEOUT:
            return "RESULT_OPERATION_TIMEOUT";
        case RESULT_OPERATION_STOP:
            return "RESULT_OPERATION_STOP";

        case RESULT_OPERATION_NOT_SUPPORT:
            return "RESULT_OPERATION_NOT_SUPPORT";

        case RESULT_FORMAT_NOT_SUPPORT:
            return "RESULT_FORMAT_NOT_SUPPORT";
        case RESULT_INSUFFICIENT_MEMORY:
            return "RESULT_INSUFFICIENT_MEMORY";
        case RESULT_OPERATION_ABORTED:
            return "RESULT_OPERATION_ABORTED";
        case RESULT_NOT_FOUND:
            return "RESULT_NOT_FOUND";
        case RESULT_RECONNECTING:
            return "RESULT_RECONNECTING";
        default:
            return "UNKNOWN";
        }
    }

    template <typename T>
    void error_chk(u_result res, const char *message)
    {
        if (!IS_OK(res))
        {
            std::string error_msg(message);
            error_msg.append(" Reason: ");
            error_msg.append(to_string(res));
            throw T(error_msg);
        }
    }
}

// Create the constructor: Here the driver will be created.
PyLidar::PyLidar(std::string my_port, uint32_t baudrate) : drv(RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT)), com_port(my_port)
{
    if (!drv)
    {
        throw std::bad_alloc();
    }

    if (baudrate != 0)
    {
        error_chk<std::runtime_error>(
            drv->connect(com_port.c_str(), baudrate),
            "Could not connect to lidar. Check COM port, baudrate, and the permissions on the port.");

        error_chk<std::runtime_error>(
            drv->getDeviceInfo(this->device_info),
            "Could not determine Lidar health during connection.");
        return;
    }
    else
    {
        std::array<uint32_t, 3> baudrateArray = {{115200, 256000, 1000000}};

        for (const auto &rate : baudrateArray)
        {
            try
            {
                error_chk<std::runtime_error>(
                    drv->connect(com_port.c_str(), baudrate),
                    "Could not connect to lidar. Check COM port, baudrate, and the permissions on the port.");

                rplidar_response_device_info_t devinfo;
                error_chk<std::runtime_error>(
                    drv->getDeviceInfo(devinfo),
                    "Could not determine Lidar health during connection.");
                return;
            }
            catch (std::runtime_error &ignored)
            {
                continue;
            }
        }
        throw std::runtime_error("Could not connect to lidar. Check COM port.");
    }
}

PyLidar::~PyLidar()
{
    if (drv)
        RPlidarDriver::DisposeDriver(drv);
}

// A wrapper code for the checkheatlh status
bool PyLidar::checkhealth() const
{
    rplidar_response_device_health_t healthinfo;

    if (IS_OK(drv->getHealth(healthinfo)))
    { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        return healthinfo.status != RPLIDAR_STATUS_ERROR;
    }
    else
    {
        return false;
    }
}

// for stopping the motor
void PyLidar::stopmotor() const
{
    error_chk<std::runtime_error>(
        drv->stop(),
        "Could not stop lidar.");

    error_chk<std::runtime_error>(
        drv->stopMotor(),
        "Could not stop lidar motor.");
}

void PyLidar::reset() const
{
    error_chk<std::runtime_error>(
        drv->reset(),
        "Could not reset lidar.");
}


void PyLidar::startmotor() const
{
    // Starts the motor.
    error_chk<std::runtime_error>(
        drv->startMotor(),
        "Could not start lidar motor");

    //error_chk<std::runtime_error>(
    //    drv->setMotorSpeed(), "Could not set speed");
    // 
}

/*
 * This function will be used in fetching the scan data
 */

std::pair<lidar_sample *, std::size_t> PyLidar::get_scan_as_lidar_samples(bool filter_quality) const
{
    // Make a buffer for the scanned data
    std::array<rplidar_response_measurement_node_hq_t, 8192> nodes;

    std::size_t count = nodes.size();

    // Grab a scan frame
    error_chk<std::runtime_error>(
        drv->grabScanDataHq(&nodes[0], count),
        "Failed to read lidar.");

    // Sort scan
    error_chk<std::runtime_error>(
        drv->ascendScanData(&nodes[0], count),
        "Could not ascendScanData.");

    // Create output buffer
    lidar_sample *output(new lidar_sample[count]);

    std::size_t idx = 0;
    for (std::size_t pos = 0; pos < count; pos++)
    {
        const int quality = nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        if (filter_quality)
        {
            if (quality > 2)
            {
                lidar_sample ls = {
                    nodes[pos].angle_z_q14 * 90.f / (1 << 14), // Angle
                    nodes[pos].dist_mm_q2 / 4.0f,              // Distance
                    quality                                    // Quality
                };
                output[idx] = ls;
                idx++;
            }
        }
        else
        {
            lidar_sample ls = {
                nodes[pos].angle_z_q14 * 90.f / (1 << 14),                   // Angle
                nodes[pos].dist_mm_q2 / 4.0f,                                // Distance
                nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT // Quality
            };
            output[pos] = ls;
            idx = pos;
        }
    }
    return {output, idx + 1};
}

std::pair<point *, size_t> PyLidar::get_scan_as_xy(bool filter_quality) const
{
    // Make a buffer for the scanned data
    std::array<rplidar_response_measurement_node_hq_t, 8192> nodes;

    std::size_t count = nodes.size();

    // Grab a scan frame
    error_chk<std::runtime_error>(
        drv->grabScanDataHq(&nodes[0], count),
        "Failed to read lidar.");

    // Sort scan
    error_chk<std::runtime_error>(
        drv->ascendScanData(&nodes[0], count),
        "Could not ascendScanData.");

    // Create output buffer
    point *output(new point[count]);

    if (filter_quality)
    {
        size_t idx = 0;
        for (size_t pos = 0; pos < count; pos++)
        {
            const int quality = nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
            if (quality > 2)
            {
                const double angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
                const double distance = nodes[pos].dist_mm_q2 / 4.0f;
                output[idx] = {
                    std::cos(deg2rad * angle) * distance, // X
                    std::sin(deg2rad * angle) * distance  // Y
                };
                idx++;
            }
        }
        return {output, idx + 1};
    }
    else
    {
        for (std::size_t pos = 0; pos < count; pos++)
        {
            const double angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
            const double distance = nodes[pos].dist_mm_q2 / 4.0f;
            output[pos] = {
                std::cos(deg2rad * angle) * distance, // X
                std::sin(deg2rad * angle) * distance  // Y
            };
        }
        return std::make_pair(output, count);
    }
}

// Device Properties
std::string PyLidar::serial_number() const
{
    char arr[128] = {};
    for (int pos = 0; pos < 16; ++pos)
    {
        snprintf(arr + strlen(arr), sizeof(arr) - strlen(arr), "%02X", device_info.serialnum[pos]);
    }

    return arr;
}

// Device Properties
std::string PyLidar::firmware_version() const
{
    char arr[128];
    snprintf(arr, sizeof(arr), "%d.%02d", device_info.firmware_version >> 8, device_info.firmware_version & 0xFF);

    return arr;
}

// Device Properties
std::string PyLidar::hardware_version() const
{
    return std::to_string((int)device_info.hardware_version);
}

std::string PyLidar::to_string() const
{
    char arr[2048];
    snprintf(arr, sizeof(arr), "SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos)
    {
        snprintf(arr + strlen(arr), sizeof(arr) - strlen(arr), "%02X", device_info.serialnum[pos]);
    }

    snprintf(arr + strlen(arr), sizeof(arr) - strlen(arr),
             "\n"
             "Firmware Ver: %d.%02d\n"
             "Hardware Rev: %d",
             device_info.firmware_version >> 8, device_info.firmware_version & 0xFF, (int)device_info.hardware_version);

    return arr;
}