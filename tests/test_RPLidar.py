import pickle
import unittest
import numpy
import time

from FastRPLidar import RPLidar


class TestRPLidar(unittest.TestCase):
    def test_construction(self):
        l = RPLidar("/dev/ttyUSB0",1000000)
        l.start_motor(0)
        l.get_scanline_xy(False)
        time.sleep(2)



if __name__ == '__main__':
    unittest.main()
    