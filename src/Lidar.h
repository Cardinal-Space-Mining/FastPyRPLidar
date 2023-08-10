#include <string>  				//std::string
#include <utility> 				//std::pair
#include <cstdint>				//std::uint8_t
#include <memory>               //std::unique_ptr

#include "sl_lidar.h" 			//sl::IChannel
#include "sl_lidar_driver.h"	//sl::ILidarDriver

// The responsability of this class is to interface to the slamtek library and provide easy access to the data of a hardwired lidar
class Lidar
{
public: //Classes and structs
	typedef struct lidar_sample
	{
		double angle;	 // Degrees
		double distance; // Meters
		std::uint8_t quality;	 //[0,255]

		lidar_sample(sl_lidar_response_measurement_node_hq_t &node);
		lidar_sample() = default;
	} lidar_sample;

	typedef struct point
	{
		double x; //X pos in meters
		double y; //Y pos in meters
		std::uint8_t quality;	 //[0,255]

		bool operator==(const point& other) const;

		point(sl_lidar_response_measurement_node_hq_t &node);
		point() = default;

	private:
		static double generate_x(sl_lidar_response_measurement_node_hq_t &node);
		static double generate_y(sl_lidar_response_measurement_node_hq_t &node);
	} point;

	enum class RPLidar_Result_Code : sl_result
	{
		OK = SL_RESULT_OK,
		FAIL_BIT = SL_RESULT_FAIL_BIT,
		ALREADY_DONE = SL_RESULT_ALREADY_DONE,
		INVALID_DATA = SL_RESULT_INVALID_DATA,
		OPERATION_FAIL = SL_RESULT_OPERATION_FAIL,
		OPERATION_TIMEOUT = SL_RESULT_OPERATION_TIMEOUT,
		OPERATION_STOP = SL_RESULT_OPERATION_STOP,
		OPERATION_NOT_SUPPORT = SL_RESULT_OPERATION_NOT_SUPPORT,
		FORMAT_NOT_SUPPORT = SL_RESULT_FORMAT_NOT_SUPPORT,
		INSUFFICIENT_MEMORY = SL_RESULT_INSUFFICIENT_MEMORY,
		UNKNOWN
	};

	enum class RPLidar_Status_Code : int32_t
	{
		OK = (int32_t)SL_LIDAR_STATUS_OK,
		WARNING = (int32_t)SL_LIDAR_STATUS_WARNING,
		ERROR = (int32_t)SL_LIDAR_STATUS_ERROR,
		UNKNOWN
	};


public: //Ctor Dtor
	// Here the driver will be created and the device will be connected
	Lidar(std::string my_port, uint32_t baudrate);
	~Lidar();

private: //Member variable initialization methods
	sl_lidar_response_device_info_t init_device_info();

	std::string init_mac_address();

public: //Methods

	void stop_motor();

	void start_motor();

	void reset();

	std::string serial_number() const;

	std::string firmware_version() const;

	std::string hardware_version() const;

	std::string to_string() const;

	std::string mac_addr() const;

	std::pair<RPLidar_Status_Code, RPLidar_Result_Code> get_health();

	/*
	 * This function will be used in fetching the scan data
	 * The output is a vector of lidar_samples.
	 * */
	std::pair<lidar_sample *, std::size_t> get_scan_as_lidar_samples();

	/*
	 * Returns scan data in the form of x-y pairs
	 * */
	std::pair<point *, std::size_t> get_scan_as_xy();

private: //Class Constants

	// It is defined in the internal section of the RPLidar library
	// But it is not exported in the include headers
	static constexpr size_t MAX_SCAN_NODES = 8192; 

private: //Member Variables

	const std::unique_ptr<sl::IChannel> m_channel;

	const std::unique_ptr<sl::ILidarDriver> m_driver;

	const sl_lidar_response_device_info_t m_device_info;

	//Max size of mac address is 17 chars
	const std::string m_mac_address;

	const std::string m_com_port;
};
