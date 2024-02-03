#include <array>     //std::array
#include <stdexcept> //std::runtime_error, std::invalid_argument, std::bad_alloc
#include <utility>   //std::pair
#include <cmath>     //std::sin, std::cos
#include <vector>    //std::vector
#include <cstdio>    //std::snprintf
#include <algorithm> //std::find
#include <cstring>   //std::strlen

#include "Lidar.h"
namespace {
    constexpr double PI = 3.14159265358979323846;
    constexpr double deg_to_rad = PI / 180.0;
}

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

    constexpr Lidar::RPLidar_Result_Code from_sl_result(sl_result res)
	{
		switch (res)
		{
		case SL_RESULT_OK:
			return Lidar::RPLidar_Result_Code::OK;
		case SL_RESULT_FAIL_BIT:
			return Lidar::RPLidar_Result_Code::FAIL_BIT;
		case SL_RESULT_ALREADY_DONE:
			return Lidar::RPLidar_Result_Code::ALREADY_DONE;
		case SL_RESULT_INVALID_DATA:
			return Lidar::RPLidar_Result_Code::INVALID_DATA;
		case SL_RESULT_OPERATION_FAIL:
			return Lidar::RPLidar_Result_Code::OPERATION_FAIL;
		case SL_RESULT_OPERATION_TIMEOUT:
			return Lidar::RPLidar_Result_Code::OPERATION_TIMEOUT;
		case SL_RESULT_OPERATION_STOP:
			return Lidar::RPLidar_Result_Code::OPERATION_STOP;
		case SL_RESULT_OPERATION_NOT_SUPPORT:
			return Lidar::RPLidar_Result_Code::OPERATION_NOT_SUPPORT;
		case SL_RESULT_FORMAT_NOT_SUPPORT:
			return Lidar::RPLidar_Result_Code::FORMAT_NOT_SUPPORT;
		case SL_RESULT_INSUFFICIENT_MEMORY:
			return Lidar::RPLidar_Result_Code::INSUFFICIENT_MEMORY;
		default:
			return Lidar::RPLidar_Result_Code::UNKNOWN;
		}
	}

	constexpr Lidar::RPLidar_Status_Code from_i32(int32_t code)
	{
		switch (code)
		{
		case SL_LIDAR_STATUS_OK:
			return Lidar::RPLidar_Status_Code::OK;
		case SL_LIDAR_STATUS_WARNING:
			return Lidar::RPLidar_Status_Code::WARNING;
		case SL_LIDAR_STATUS_ERROR:
			return Lidar::RPLidar_Status_Code::ERROR;
		default:
			return Lidar::RPLidar_Status_Code::UNKNOWN;
		}
	}

    template <typename T>
    void error_chk(sl_result res, const char *message)
    {
        if (!SL_IS_OK(res))
        {
            char error_message[2048] = {};

            std::snprintf(error_message + std::strlen(error_message),
                     sizeof(error_message) - std::strlen(error_message),
                     "%s. Reason: %s.",
                     message,
                     to_string(res));

            throw T(std::string(error_message));
        }
    }
}

sl_lidar_response_device_info_t Lidar::init_device_info(){
    if (!m_channel)
    {
        throw std::bad_alloc();
    }

    if (!m_driver)
    {
        throw std::bad_alloc();
    }

    error_chk<std::runtime_error>(
        m_driver->connect(m_channel.get()),
        "Could not connect to Lidar");
    
    sl_lidar_response_device_info_t dev_info;

    error_chk<std::runtime_error>(
        m_driver->getDeviceInfo(dev_info),
        "Could not retrive device data during connection");

    return dev_info;

}

std::string Lidar::init_mac_address()
{
    if (!m_channel)
    {
        throw std::bad_alloc();
    }

    if (!m_driver)
    {
        throw std::bad_alloc();
    }

    char arr[20] = {}; //Max size of Mac Address is 17 chars
    error_chk<std::runtime_error>(
        m_driver->getDeviceMacAddr((sl_u8 *)arr),
        "Failed to retrieve mac address");

    return std::string(arr);
}


sl::IChannel* open_channel(std::string& my_port, uint32_t baudrate){
    sl::Result<sl::IChannel*> channel = sl::createSerialPortChannel(my_port, baudrate);

    error_chk<std::runtime_error>(channel,"Error opening Serial Port Channel!");

    return *channel;

}

sl::ILidarDriver* open_lidar_driver(){
    sl::Result<sl::ILidarDriver*> err_res = sl::createLidarDriver();

    error_chk<std::runtime_error>(err_res,"Error opening Lidar Driver!");

    return *err_res;

}

// Create the constructor: Here the driver will be created.
Lidar::Lidar(std::string my_port, uint32_t baudrate) : m_channel(open_channel(my_port, baudrate)), m_driver(open_lidar_driver()), m_device_info(init_device_info()), m_mac_address(init_mac_address()), m_com_port(my_port)
{
}

Lidar::~Lidar()
{
    if (m_driver)
        m_driver->stop(); //No error checking as it is best effort

    if (m_driver)
        m_driver->disconnect(); //No error checking as it is best effort

}

void Lidar::start_motor()
{
    error_chk<std::runtime_error>(
        m_driver->setMotorSpeed(),
        "Could not start lidar motor.");

    error_chk<std::runtime_error>(
        m_driver->startScan(false, true),
        "Could not start scan");
}

void Lidar::stop_motor()
{
    error_chk<std::runtime_error>(
        m_driver->setMotorSpeed(0),
        "Could not stop lidar.");
}

void Lidar::reset()
{
    error_chk<std::runtime_error>(
        m_driver->reset(),
        "Could not reset lidar.");
}

std::pair<Lidar::lidar_sample *, std::size_t> Lidar::get_scan_as_lidar_samples()
{
    // Make a buffer for the scanned data
    std::array<sl_lidar_response_measurement_node_hq_t, MAX_SCAN_NODES> nodes;

    std::size_t count = nodes.size();

    // Grab a scan frame
    error_chk<std::runtime_error>(
        m_driver->grabScanDataHq(&nodes[0], count),
        "Failed to read lidar.");

    if (count == 0)
    {
        throw std::runtime_error("No lidar points retrieved");
    }

    // Sort scan
    error_chk<std::runtime_error>(
        m_driver->ascendScanData(&nodes[0], count),
        "Could not ascendScanData.");

    // Create output buffer
    lidar_sample *output(new lidar_sample[count]);

    std::size_t idx = 0;
    for (std::size_t pos = 0; pos < count; pos++)
    {
        const int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        output[idx] = lidar_sample(nodes[pos]);
        idx = pos;
    }
    return {output, idx + 1};
}

std::pair<Lidar::point *, size_t> Lidar::get_scan_as_xy()
{
    // Make a buffer for the scanned data
    std::array<sl_lidar_response_measurement_node_hq_t, MAX_SCAN_NODES> nodes;

    std::size_t count = nodes.size();

    // Grab a scan frame
    error_chk<std::runtime_error>(
        m_driver->grabScanDataHq(&nodes[0], count),
        "Failed to read lidar.");

    if (count == 0)
    {
        throw std::runtime_error("No lidar points retrieved");
    }

    // Sort scan
    error_chk<std::runtime_error>(
        m_driver->ascendScanData(&nodes[0], count),
        "Could not ascendScanData.");

    // Create output buffer
    point *output(new point[count]);

    for (std::size_t pos = 0; pos < count; pos++)
    {
        output[pos] = point(nodes[pos]);
    }

    return std::make_pair(output, count);

}

// ------------------------ Device Properties ---------------------------------------

// Serial #
std::string Lidar::serial_number() const
{
    char arr[128] = {};
    for (int pos = 0; pos < 16; ++pos)
    {
        snprintf(arr + strlen(arr), sizeof(arr) - strlen(arr), "%02X", m_device_info.serialnum[pos]);
    }

    return arr;
}

// Device Properties
std::string Lidar::firmware_version() const
{
    char arr[128] = {};
    snprintf(arr, sizeof(arr), "%d.%02d", m_device_info.firmware_version >> 8, m_device_info.firmware_version & 0xFF);

    return arr;
}

// Device Properties
std::string Lidar::hardware_version() const
{
    return std::to_string((int)m_device_info.hardware_version);
}

std::string Lidar::to_string() const
{
    char arr[2048] = {};
    snprintf(arr, sizeof(arr), "SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos)
    {
        snprintf(arr + strlen(arr), sizeof(arr) - strlen(arr), "%02X", m_device_info.serialnum[pos]);
    }

    snprintf(arr + strlen(arr), sizeof(arr) - strlen(arr),
             "\n"
             "Firmware Ver: %d.%02d\n"
             "Hardware Rev: %d",
             m_device_info.firmware_version >> 8, m_device_info.firmware_version & 0xFF, (int)m_device_info.hardware_version);

    return arr;
}

std::string Lidar::mac_addr() const{
    return m_mac_address;
}

std::pair<Lidar::RPLidar_Status_Code, Lidar::RPLidar_Result_Code> Lidar::get_health()
{
    sl_lidar_response_device_health_t health;
    error_chk<std::runtime_error>(
        m_driver->getHealth(health),
        "Could not read health");

    return std::make_pair(
        from_i32(health.error_code),
        from_sl_result(health.status));
}

// -------------------------- Custom Data Types ------------------------------

Lidar::lidar_sample::lidar_sample(sl_lidar_response_measurement_node_hq_t &node) : angle(node.angle_z_q14 * 90.f / (1 << 14)),
                                                                                     distance(node.dist_mm_q2 / 1000.f / (1 << 2)),
                                                                                     quality(node.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT){};

Lidar::point::point(sl_lidar_response_measurement_node_hq_t &node) : x(point::generate_x(node)), 
                                                                       y(point::generate_y(node)), 
                                                                       quality(node.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT){};

double Lidar::point::generate_y(sl_lidar_response_measurement_node_hq_t &node)
{
    const double angle_degrees = node.angle_z_q14 * 90.f / (1 << 14);
    const double distance_meters = node.dist_mm_q2 / 1000.f / (1 << 2);
    return std::sin(deg_to_rad * angle_degrees) * distance_meters;
}

double Lidar::point::generate_x(sl_lidar_response_measurement_node_hq_t &node)
{
    const double angle_degrees = node.angle_z_q14 * 90.f / (1 << 14);
    const double distance_meters = node.dist_mm_q2 / 1000.f / (1 << 2);

    return std::cos(deg_to_rad * angle_degrees) * distance_meters;
}

bool Lidar::point::operator==(const point& other) const{
    return (this->x == other.x) & (this->y==other.y);
}
