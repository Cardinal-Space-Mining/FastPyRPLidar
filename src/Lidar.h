#include <string>  //std::string
#include <utility> //std::pair
#include <memory>  //std::unique_ptr

#include "rplidar.h" //RPLIDAR sdk



typedef struct
{
    double angle;
    double distance;
    int quality;
}lidar_sample;

typedef struct point
{
    double x;
    double y;
}point;



// Creating a class to manage everything
class PyLidar {	
	public:
	    // Here the driver will be created and the device will be connected
		PyLidar(std::string my_port, uint32_t baudrate = 0);
		~PyLidar();

		/* This function is for checking the device health. 
		 * It returns true if the health is OK and false 
		 * if otherwise
		 */
		bool checkhealth(void) const;

		// stopping the motor
		void stopmotor(void) const;

		// starts the motor
		void startmotor(int my_scanmode = 2) const ;

        void reset() const;

		/*
		 * This function will be used in fetching the scan data
		 * The output is a vector of lidar_samples.
		 * */
		std::pair<std::unique_ptr<lidar_sample[]>, size_t> get_scan_as_lidar_samples(bool filter_quality=false) const;

		/*
		 * Returns scan data in the form of x-y pairs
		 * */
		std::pair<std::unique_ptr<point[]>, size_t> get_scan_as_xy(bool filter_quality=false) const;

    private:
		rp::standalone::rplidar::RPlidarDriver* const drv;

		const std::string com_port;
};
