# FastPyRPLidar
This library is a compatibility layer between the Slamtek C++ SDK and Python.
It exposes:
* class `Lidar`:
   * methods:
      * start_motor
      * stop_motor
      * reset
      * get_scanline_xy
      * get_scanline
      * get_health
   * properties:
      * serial_number
      * firmware_version
      * hardware_version
      * mac address
* enum `Result_Code`
   * OK
   * FAIL_BIT
   * ALREADY_DONE
   * INVALID_DATA
   * OPERATION_FAIL
   * OPERATION_TIMEOUT
   * OPERATION_STOP
   * OPERATION_NOT_SUPPORT
   * FORMAT_NOT_SUPPORT
   * INSUFFICIENT_MEMORY
   * UNKNOWN
* enum `Status_Code`
   * OK
   * WARNING
   * ERROR
   * UNKNOWN
* class `Lidar_Scan`
   * properties:
      * angle (degrees)
      * distance (meters)
      * quality (range[0,255])
* class `Point`
   * properties:
      * x (meters from lidar)
      * y (meters from lidar)
      * quality (range[0,255])
# Requirements
* C++ compiler (GCC) reccomended
* Building Documentation requires Sphinx

# Installation
* Manual:
  1. Download this repository
  2. Navigate to the top level of the repository
  3. Execute  `pip install ./`
* Pip and git installed:
   1. `pip install git+https://github.com/Cardinal-Space-Mining/FastPyRPLidar.git`
* Only pip installed
    1. `pip install https://github.com/Cardinal-Space-Mining/FastPyRPLidar/tarball/master`

# Documentation
1. Download this repository
2. Navigate to the docs folder
3. Execute `make html`
4. Documentation files are in .\docs\_build

# Troubleshooting
1. Lidar refuses to connect (Linux):
    * Try `sudo chmod a+rw /dev/ttyUSB0` (or whatever USB device you are using). Your OS may be blocking access to the USB device.
2. Lidar connection times out during constructor with the message `Could not retrieve device data during connection` or `Could not connect to Lidar`
    * Try `sudo chmod a+rw /dev/ttyUSB0` (or whatever USB device you are using). Your OS may be blocking access to the USB device.
    * Validate that the Lidar is getting at least 4A of USB current. 2A @ 5V from barrel jack and 2A @5V from USB in.
