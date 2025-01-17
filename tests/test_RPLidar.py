import pickle
import unittest
import numpy
import time

from FastPyRpLidar import RPLidar


class TestRPLidar(unittest.TestCase):
    def test_construction(self):
        l = RPLidar("/dev/ttyUSB0",1000000)
        print(l.serial_number)
        print(l.firmware_version)
        print(l.hardware_version)
        print(l)

    def test_scan(self):
        l = RPLidar("/dev/ttyUSB0",1000000)
        l.start_motor()
        l.get_scanline_xy()



if __name__ == '__main__':
    unittest.main()
    