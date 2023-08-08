#include <string>  //std::string
#include <utility> //std::pair

#include "sl_lidar.h" //RPLIDAR sdk
#include "sl_lidar_driver.h"

// Creating a class to manage everything
class PyLidar
{
public:
	typedef struct lidar_sample
	{
		double angle;	 // Degrees
		double distance; // Meters
		int quality;	 //[0,255]

		lidar_sample(sl_lidar_response_measurement_node_hq_t &node);
		lidar_sample() = default;
	} lidar_sample;

	typedef struct point
	{
		double x; //X pos in meters
		double y; //Y pos in meters

		bool operator==(const point& other) const;

		point(sl_lidar_response_measurement_node_hq_t &node);
		point() = default;

		static std::size_t deduplicate_array(point* arr, std::size_t length);

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

	inline constexpr static RPLidar_Result_Code from_sl_result(sl_result res)
	{
		switch (res)
		{
		case SL_RESULT_OK:
			return RPLidar_Result_Code::OK;

		case SL_RESULT_FAIL_BIT:
			return RPLidar_Result_Code::FAIL_BIT;
		case SL_RESULT_ALREADY_DONE:
			return RPLidar_Result_Code::ALREADY_DONE;
		case SL_RESULT_INVALID_DATA:
			return RPLidar_Result_Code::INVALID_DATA;
		case SL_RESULT_OPERATION_FAIL:
			return RPLidar_Result_Code::OPERATION_FAIL;
		case SL_RESULT_OPERATION_TIMEOUT:
			return RPLidar_Result_Code::OPERATION_TIMEOUT;
		case SL_RESULT_OPERATION_STOP:
			return RPLidar_Result_Code::OPERATION_STOP;
		case SL_RESULT_OPERATION_NOT_SUPPORT:
			return RPLidar_Result_Code::OPERATION_NOT_SUPPORT;
		case SL_RESULT_FORMAT_NOT_SUPPORT:
			return RPLidar_Result_Code::FORMAT_NOT_SUPPORT;
		case SL_RESULT_INSUFFICIENT_MEMORY:
			return RPLidar_Result_Code::INSUFFICIENT_MEMORY;
		default:
			return RPLidar_Result_Code::UNKNOWN;
		}
	}

	enum class RPLidar_Status_Code : int32_t
	{
		OK = (int32_t)SL_LIDAR_STATUS_OK,
		WARNING = (int32_t)SL_LIDAR_STATUS_WARNING,
		ERROR = (int32_t)SL_LIDAR_STATUS_ERROR,
		UNKNOWN
	};

	inline constexpr static RPLidar_Status_Code from_i32(int32_t code)
	{
		switch (code)
		{
		case SL_LIDAR_STATUS_OK:
			return RPLidar_Status_Code::OK;

		case SL_LIDAR_STATUS_WARNING:
			return RPLidar_Status_Code::WARNING;
		case SL_LIDAR_STATUS_ERROR:
			return RPLidar_Status_Code::ERROR;
		default:
			return RPLidar_Status_Code::UNKNOWN;
		}
	}

	// Here the driver will be created and the device will be connected
	PyLidar(std::string my_port, uint32_t baudrate);
	~PyLidar();

	// stopping the motor
	void stop_motor();

	void start_motor();

	void reset();

	std::string serial_number() const;

	std::string firmware_version() const;

	std::string hardware_version() const;

	std::string to_string() const;

	std::string mac_addr();

	std::pair<RPLidar_Status_Code, RPLidar_Result_Code> get_health();

	/*
	 * This function will be used in fetching the scan data
	 * The output is a vector of lidar_samples.
	 * */
	std::pair<lidar_sample *, std::size_t> get_scan_as_lidar_samples(bool filter_quality = false);

	/*
	 * Returns scan data in the form of x-y pairs
	 * */
	std::pair<point *, std::size_t> get_scan_as_xy(bool filter_quality = false);

private:
	static constexpr size_t MAX_SCAN_NODES = 8192; // Defined in the internal section of the RPLidar library

	sl::IChannel *const channel;

	sl::ILidarDriver *const drv;

	const std::string com_port;

	sl_lidar_response_device_info_t device_info;
};
