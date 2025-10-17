from enum import (
    IntEnum,
    auto
)

class SensorType(IntEnum):
    VOXL_IMU0 = auto()
    VOXL_IMU1 = auto()
    VOXL_TRACKING_CAMERA = auto() # tracking camera image data
    
    PX4_IMU0 = auto()
    PX4_IMU1 = auto()
    PX4_GPS = auto()
    PX4_ACTUATOR_MOTORS = auto() # PX4 actuator motors [-1, 1]
