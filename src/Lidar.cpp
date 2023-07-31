#include <array> //std::array
#include <cmath> //M_PI
#include <stdexcept> //std::runtime_error, std::invalid_argument, std::bad_alloc
#include <vector>

#include "Lidar.h"

using namespace rp::standalone::rplidar;

constexpr double deg2rad = M_PI/180.0;

// Create the constructor: Here the driver will be created.
PyLidar::PyLidar(std::string my_port, uint32_t baudrate): drv(RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT)), com_port(my_port)
{
    if (!drv) {
        throw std::bad_alloc();
    }

    if (baudrate != 0){
        if(IS_OK(drv->connect(com_port.c_str(), baudrate)))
        {
            rplidar_response_device_info_t devinfo;
            if (IS_OK(drv->getDeviceInfo(devinfo)))
            {
                return;
            }else
            {
                throw std::runtime_error("Connected to lidar but could not determine its health.");
            };
        }else{
            throw std::invalid_argument("Could not connect to lidar. Check COM port or baudrate.");
        }
    }else{
        std::array<uint32_t,3> baudrateArray = {{115200, 256000, 2000000}};

        for (const auto& rate: baudrateArray){
            if(IS_OK(drv->connect(com_port.c_str(), rate)))
            {
                rplidar_response_device_info_t devinfo;
                if (IS_OK(drv->getDeviceInfo(devinfo)))
                {
                    return;
                }else
                {
                    throw std::runtime_error("Connected to lidar but could not determine its health.");
                };
            }
        }
        throw std::invalid_argument("Could not connect to lidar. Check COM port.");
    }

}

PyLidar::~PyLidar()
{
    if(drv)
        RPlidarDriver::DisposeDriver(drv);
}


// A wrapper code for the checkheatlh status
bool PyLidar::checkhealth() const
{
    rplidar_response_device_health_t healthinfo;

    if (IS_OK(drv->getHealth(healthinfo))) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        return healthinfo.status != RPLIDAR_STATUS_ERROR;

    } else {
        return false;
    }
}


// for stopping the motor
void PyLidar::stopmotor() const
{
    if(!IS_OK(drv->stop())) 
        throw std::runtime_error("Could not stop lidar");
    if(!IS_OK(drv->stopMotor())) 
        throw std::runtime_error("Could not stop lidar");
}

void PyLidar::reset() const{
    if(!IS_OK(drv->reset())) 
        throw std::runtime_error("Could not reset lidar");
}


// This function starts the motor and also begins scan
// It requires the scanmode to use: 3 scan modes are supported: 1,2,3
// Default mode is 2
void PyLidar::startmotor(int my_scanmode) const
{
    // Starts the motor.
     if(!IS_OK(drv->startMotor()))
         throw std::runtime_error("Could not start lidar motor");
    
    // For setting scanmodes
	std::vector<RplidarScanMode> myscanModes;

    //Fetch scan modes
    if(!IS_OK(drv->getAllSupportedScanModes(myscanModes)))
         throw std::runtime_error("Could not start lidar motor");

    // start scan...
    //drv->startScan(0,1);
    if(!IS_OK(drv->startScanExpress(false, myscanModes[my_scanmode].id)))
         throw std::runtime_error("Could not scan");
    
}


/*
 * This function will be used in fetching the scan data
 */

std::pair<std::unique_ptr<lidar_sample[]>, size_t> PyLidar::get_scan_as_lidar_samples(bool filter_quality) const
{ 
    // Make a buffer for the scanned data
	std::array<rplidar_response_measurement_node_hq_t, 8192>  nodes;

    size_t count = nodes.size();

    // Grab a scan frame
    if (IS_OK(drv->grabScanDataHq(&nodes[0], count))) {
        // sort the scan data
        if(!IS_OK(drv->ascendScanData(&nodes[0], count)))
            throw std::runtime_error("Could not ascendScanData");

        //Create output buffer
        std::unique_ptr<lidar_sample[]> output(new lidar_sample[count]);
        
        if (filter_quality){
            size_t idx = 0;
            for (size_t pos = 0; pos < count ; pos++) {
                const int quality = nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                if (quality > 2){
                    lidar_sample ls = {
                        nodes[pos].angle_z_q14 * 90.f / (1 << 14), //Angle
                        nodes[pos].dist_mm_q2/4.0f,                //Distance
                        quality                                    //Quality
                    };
                    output[idx] = ls;
                    idx++;
                }
            }
            return std::make_pair(output, idx+1);

        }else{
            for (size_t pos = 0; pos < count ; pos++) {
                lidar_sample ls = {
                        nodes[pos].angle_z_q14 * 90.f / (1 << 14), //Angle
                        nodes[pos].dist_mm_q2/4.0f,                //Distance
                        nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT                                    //Quality
                    };
                output[pos] = ls;

            }
            return std::make_pair(output, count);
        }

    }else{
        throw std::runtime_error("Failed to read Lidar");
    }

}

	
std::pair<std::unique_ptr<point[]>, size_t> PyLidar::get_scan_as_xy(bool filter_quality) const{
  // Make a buffer for the scanned data
	std::array<rplidar_response_measurement_node_hq_t, 8192>  nodes;

    size_t count = nodes.size();

    // Grab a scan frame
    if (IS_OK(drv->grabScanDataHq(&nodes[0], count))) {
        // sort the scan data
        if(!IS_OK(drv->ascendScanData(&nodes[0], count)))
            throw std::runtime_error("Could not ascendScanData");

        //Create output buffer
        std::unique_ptr<point[]> output(new point[count]);
        
        if (filter_quality){
            size_t idx = 0;
            for (size_t pos = 0; pos < count ; pos++) {
                const int quality = nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                if (quality > 2){
                    const double angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
                    const double distance = nodes[pos].dist_mm_q2/4.0f;
                    output[idx] = {
                        std::cos(deg2rad*angle) * distance, //X
                        std::sin(deg2rad*angle) * distance  //Y
                    };
                    idx++;
                }
            }
            return std::make_pair(output, idx + 1);

        }else{
            for (size_t pos = 0; pos < count ; pos++) {
                const double angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
                const double distance = nodes[pos].dist_mm_q2/4.0f;
                output[pos] = {
                    std::cos(deg2rad*angle) * distance, //X
                    std::sin(deg2rad*angle) * distance  //Y
                };
            }
            return std::make_pair(output, count);
        }

    }else{
        throw std::runtime_error("Failed to read Lidar");
    }

}