from __future__ import annotations
import FastPyRpLidar
import typing
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "Lidar_Scan",
    "Point",
    "RPLidar",
    "Result_Code",
    "Status_Code"
]


class Lidar_Scan():
    @property
    def angle(self) -> float:
        """
        Angle in degrees

        :type: float
        """
    @property
    def distance(self) -> float:
        """
        Distance in meters

        :type: float
        """
    @property
    def quality(self) -> int:
        """
        Measurement quality (0 ~ 255). Higher is better

        :type: int
        """
    pass
class Point():
    @property
    def x(self) -> float:
        """
        X location with respect to the lidar in meters

        :type: float
        """
    @property
    def y(self) -> float:
        """
        Y location with respect to the lidar in meters

        :type: float
        """
    pass
class RPLidar():
    def __init__(self, port: str, baud_rate: int) -> None: 
        """
        Loads Lidar from given USB port at given baud rate
        """
    def __str__(self) -> str: ...
    def get_health(self) -> typing.Tuple[Status_Code, Result_Code]: 
        """
        Returns the health of the Lidar
        """
    def get_scanline(self, filter_low_quality: bool) -> numpy.ndarray[Lidar_Scan]: 
        """
        Returns scan line in the form of x-y pairs with 0-0 as the lidar
        """
    def get_scanline_xy(self, filter: bool) -> numpy.ndarray[Point]: 
        """
        Returns scan line in the form of x-y pairs with 0-0 as the lidar
        """
    def reset(self) -> None: 
        """
        Resets the underlying lidar driver
        """
    def start_motor(self) -> None: 
        """
        Starts the Lidar motor and starts scanning procedures
        """
    def stop_motor(self) -> None: 
        """
        Stops the Lidar Motor
        """
    @property
    def firmware_version(self) -> str:
        """
        Device firmware_version

        :type: str
        """
    @property
    def hardware_version(self) -> str:
        """
        Device hardware_version

        :type: str
        """
    @property
    def serial_number(self) -> str:
        """
        Device serial number

        :type: str
        """
    pass
class Result_Code():
    """
    Lidar result enum

    Members:

      OK

      FAIL_BIT

      ALREADY_DONE

      INVALID_DATA

      OPERATION_FAIL

      OPERATION_TIMEOUT

      OPERATION_STOP

      OPERATION_NOT_SUPPORT

      FORMAT_NOT_SUPPORT

      INSUFFICIENT_MEMORY

      UNKNOWN
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    ALREADY_DONE: FastPyRpLidar.Result_Code # value = <Result_Code.ALREADY_DONE: 32>
    FAIL_BIT: FastPyRpLidar.Result_Code # value = <Result_Code.FAIL_BIT: 2147483648>
    FORMAT_NOT_SUPPORT: FastPyRpLidar.Result_Code # value = <Result_Code.FORMAT_NOT_SUPPORT: 2147516421>
    INSUFFICIENT_MEMORY: FastPyRpLidar.Result_Code # value = <Result_Code.INSUFFICIENT_MEMORY: 2147516422>
    INVALID_DATA: FastPyRpLidar.Result_Code # value = <Result_Code.INVALID_DATA: 2147516416>
    OK: FastPyRpLidar.Result_Code # value = <Result_Code.OK: 0>
    OPERATION_FAIL: FastPyRpLidar.Result_Code # value = <Result_Code.OPERATION_FAIL: 2147516417>
    OPERATION_NOT_SUPPORT: FastPyRpLidar.Result_Code # value = <Result_Code.OPERATION_NOT_SUPPORT: 2147516420>
    OPERATION_STOP: FastPyRpLidar.Result_Code # value = <Result_Code.OPERATION_STOP: 2147516419>
    OPERATION_TIMEOUT: FastPyRpLidar.Result_Code # value = <Result_Code.OPERATION_TIMEOUT: 2147516418>
    UNKNOWN: FastPyRpLidar.Result_Code # value = <Result_Code.UNKNOWN: 2147516423>
    __members__: dict # value = {'OK': <Result_Code.OK: 0>, 'FAIL_BIT': <Result_Code.FAIL_BIT: 2147483648>, 'ALREADY_DONE': <Result_Code.ALREADY_DONE: 32>, 'INVALID_DATA': <Result_Code.INVALID_DATA: 2147516416>, 'OPERATION_FAIL': <Result_Code.OPERATION_FAIL: 2147516417>, 'OPERATION_TIMEOUT': <Result_Code.OPERATION_TIMEOUT: 2147516418>, 'OPERATION_STOP': <Result_Code.OPERATION_STOP: 2147516419>, 'OPERATION_NOT_SUPPORT': <Result_Code.OPERATION_NOT_SUPPORT: 2147516420>, 'FORMAT_NOT_SUPPORT': <Result_Code.FORMAT_NOT_SUPPORT: 2147516421>, 'INSUFFICIENT_MEMORY': <Result_Code.INSUFFICIENT_MEMORY: 2147516422>, 'UNKNOWN': <Result_Code.UNKNOWN: 2147516423>}
    pass
class Status_Code():
    """
    Lidar status enum

    Members:

      OK

      WARNING

      ERROR

      UNKNOWN
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    ERROR: FastPyRpLidar.Status_Code # value = <Status_Code.ERROR: 2>
    OK: FastPyRpLidar.Status_Code # value = <Status_Code.OK: 0>
    UNKNOWN: FastPyRpLidar.Status_Code # value = <Status_Code.UNKNOWN: 3>
    WARNING: FastPyRpLidar.Status_Code # value = <Status_Code.WARNING: 1>
    __members__: dict # value = {'OK': <Status_Code.OK: 0>, 'WARNING': <Status_Code.WARNING: 1>, 'ERROR': <Status_Code.ERROR: 2>, 'UNKNOWN': <Status_Code.UNKNOWN: 3>}
    pass
