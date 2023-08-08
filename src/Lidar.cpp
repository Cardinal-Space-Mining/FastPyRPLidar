#include <array>     //std::array
#include <cmath>     //M_PI
#include <stdexcept> //std::runtime_error, std::invalid_argument, std::bad_alloc
#include <utility>
#include <vector>
#include <memory>
#include <cstring>
#include <algorithm> //std::find

#include "Lidar.h"

constexpr double deg2rad = M_PI / 180.0;

using std::size_t;

namespace
{
    const char *to_string(sl_result res)
    {
        switch (res)
        {
        case SL_RESULT_OK:
            return "RESULT_OK";
        case SL_RESULT_FAIL_BIT:
            return "RESULT_FAIL_BIT";
        case SL_RESULT_ALREADY_DONE:
            return "RESULT_ALREADY_DONE";
        case SL_RESULT_INVALID_DATA:
            return "RESULT_INVALID_DATA";
        case SL_RESULT_OPERATION_FAIL:
            return "RESULT_OPERATION_FAIL";
        case SL_RESULT_OPERATION_TIMEOUT:
            return "RESULT_OPERATION_TIMEOUT";
        case SL_RESULT_OPERATION_STOP:
            return "RESULT_OPERATION_STOP";

        case SL_RESULT_OPERATION_NOT_SUPPORT:
            return "RESULT_OPERATION_NOT_SUPPORT";

        case SL_RESULT_FORMAT_NOT_SUPPORT:
            return "RESULT_FORMAT_NOT_SUPPORT";
        case SL_RESULT_INSUFFICIENT_MEMORY:
            return "RESULT_INSUFFICIENT_MEMORY";
        default:
            return "UNKNOWN";
        }
    }

    template <typename T>
    void error_chk(sl_result res, const char *message)
    {
        if (!SL_IS_OK(res))
        {
            std::string error_msg(message);
            error_msg.append(" Reason: ");
            error_msg.append(to_string(res));
            throw T(error_msg);
        }
    }
}

// Create the constructor: Here the driver will be created.
PyLidar::PyLidar(std::string my_port, uint32_t baudrate) : channel(*sl::createSerialPortChannel(my_port, baudrate)), drv(*sl::createLidarDriver()), com_port(my_port)
{
    if (!channel)
    {
        throw std::bad_alloc();
    }

    if (!drv)
    {
        throw std::bad_alloc();
    }

    error_chk<std::runtime_error>(
        drv->connect(channel),
        "Could not connect to Lidar");

    error_chk<std::runtime_error>(
        drv->getDeviceInfo(device_info),
        "Could not retrive device data during connection");
}

PyLidar::~PyLidar()
{
    drv->stop();

    drv->disconnect();

    delete drv;
    delete channel;
}

void PyLidar::start_motor()
{
    error_chk<std::runtime_error>(
        drv->setMotorSpeed(),
        "Could not start lidar motor.");

    error_chk<std::runtime_error>(
        drv->startScan(false, true),
        "Could not start scan");
}

void PyLidar::stop_motor()
{
    error_chk<std::runtime_error>(
        drv->stop(),
        "Could not stop lidar.");
}

void PyLidar::reset()
{
    error_chk<std::runtime_error>(
        drv->reset(),
        "Could not reset lidar.");
}

std::pair<PyLidar::lidar_sample *, std::size_t> PyLidar::get_scan_as_lidar_samples(bool filter_quality)
{
    // Make a buffer for the scanned data
    std::array<sl_lidar_response_measurement_node_hq_t, MAX_SCAN_NODES> nodes;

    std::size_t count = nodes.size();

    // Grab a scan frame
    error_chk<std::runtime_error>(
        drv->grabScanDataHq(&nodes[0], count),
        "Failed to read lidar.");

    if (count == 0)
    {
        throw std::runtime_error("No lidar points retrieved");
    }

    // Sort scan
    error_chk<std::runtime_error>(
        drv->ascendScanData(&nodes[0], count),
        "Could not ascendScanData.");

    // Create output buffer
    lidar_sample *output(new lidar_sample[count]);

    std::size_t idx = 0;
    for (std::size_t pos = 0; pos < count; pos++)
    {
        const int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        if (filter_quality)
        {
            if (quality > 2)
            {
                output[idx] = lidar_sample(nodes[pos]);
                idx++;
            }
        }
        else
        {
            output[idx] = lidar_sample(nodes[pos]);
            idx = pos;
        }
    }
    return {output, idx + 1};
}

std::pair<PyLidar::point *, size_t> PyLidar::get_scan_as_xy(bool filter_quality)
{
    // Make a buffer for the scanned data
    std::array<sl_lidar_response_measurement_node_hq_t, MAX_SCAN_NODES> nodes;

    std::size_t count = nodes.size();

    // Grab a scan frame
    error_chk<std::runtime_error>(
        drv->grabScanDataHq(&nodes[0], count),
        "Failed to read lidar.");

    if (count == 0)
    {
        throw std::runtime_error("No lidar points retrieved");
    }

    // Sort scan
    error_chk<std::runtime_error>(
        drv->ascendScanData(&nodes[0], count),
        "Could not ascendScanData.");

    // Create output buffer
    point *output(new point[count]);
    std::size_t output_len = 0;

    if (filter_quality)
    {
        for (size_t pos = 0; pos < count; pos++)
        {
            const int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
            if (quality > 2)
            {
                output[output_len] = point(nodes[pos]);
                output_len++;
            }
        }
    }
    else
    {
        for (std::size_t pos = 0; pos < count; pos++)
        {
            output[pos] = point(nodes[pos]);
        }
        output_len = count;
    }

    output_len = PyLidar::point::deduplicate_array(output, output_len);

    return std::make_pair(output, output_len);

}

// ------------------------ Device Properties ---------------------------------------

// Serial #
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
    char arr[128] = {};
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
    char arr[2048] = {};
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

std::string PyLidar::mac_addr()
{
    char arr[2048] = {};
    error_chk<std::runtime_error>(
        drv->getDeviceMacAddr((sl_u8 *)arr),
        "help");

    return std::string(arr);
}

std::pair<PyLidar::RPLidar_Status_Code, PyLidar::RPLidar_Result_Code> PyLidar::get_health()
{
    sl_lidar_response_device_health_t health;
    error_chk<std::runtime_error>(
        drv->getHealth(health),
        "Could not read health");

    return std::make_pair(
        from_i32(health.error_code),
        from_sl_result(health.status));
}

// -------------------------- Custom Data Types ------------------------------

PyLidar::lidar_sample::lidar_sample(sl_lidar_response_measurement_node_hq_t &node) : angle(node.angle_z_q14 * 90.f / (1 << 14)),
                                                                                     distance(node.dist_mm_q2 / 1000.f / (1 << 2)),
                                                                                     quality(node.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT){};

PyLidar::point::point(sl_lidar_response_measurement_node_hq_t &node) : x(point::generate_x(node)), y(point::generate_y(node)){};

double PyLidar::point::generate_y(sl_lidar_response_measurement_node_hq_t &node)
{
    const double angle_degrees = node.angle_z_q14 * 90.f / (1 << 14);
    const double distance_meters = node.dist_mm_q2 / 1000.f / (1 << 2);
    return std::sin(deg2rad * angle_degrees) * distance_meters;
}

double PyLidar::point::generate_x(sl_lidar_response_measurement_node_hq_t &node)
{
    const double angle_degrees = node.angle_z_q14 * 90.f / (1 << 14);
    const double distance_meters = node.dist_mm_q2 / 1000.f / (1 << 2);

    return std::cos(deg2rad * angle_degrees) * distance_meters;
}

bool PyLidar::point::operator==(const point& other) const{
    return (this->x == other.x) & (this->y==other.y);
}


//We split the array into a deduplicated portion and a unknown portion
//We keep the index to the first item outside our deduplicated portion
//We iterate through the array and if the item is in the deduplicated array, we continue on
//If it is not in the deduplicated array, we add it and increment our bookmark to the next index
//Because the deduplicated array is at most the same size as our current array,
//We will only shrink the array
std::size_t PyLidar::point::deduplicate_array(PyLidar::point* arr, std::size_t length){
    if (length ==0 || length == 1)
        return length;

    std::size_t new_length = 1;

    for(std::size_t i = 0; i< length; i++){
        if (std::find(arr, arr + new_length, arr[i]) == arr + new_length){
            arr[new_length]=arr[i];
            new_length++;
        }
    }

    return new_length;

}