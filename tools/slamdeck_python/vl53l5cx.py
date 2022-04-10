from dataclasses import dataclass, fields
from numpy import dtype, ndarray, uint16, uint32, uint8
from enum import IntEnum
import numpy as np

from slamdeck_python.utils import Observable, Subscriber
from time import time


class VL53L5CX_Status(IntEnum):
    OK			 = 1
    TIMEOUT_ERROR = 2
    NOT_ENABLED   = 2
    INIT_FAIL     = 3
    MCU_ERROR			 = 66
    STATUS_INVALID_PARAM = 127
    STATUS_ERROR         = 255

class VL53L5CX_PowerMode(IntEnum):
    POWER_MODE_SLEEP  = 0
    POWER_MODE_WAKEUP = 1

class VL53L5CX_Resolution(IntEnum):
    RESOLUTION_4X4 = 16
    RESOLUTION_8X8 = 64

class VL53L5CX_TargetOrder(IntEnum):
    TARGET_ORDER_CLOSEST   = 1
    TARGET_ORDER_STRONGEST = 2

class VL53L5CX_RangingMode(IntEnum):
    RANGING_MODE_CONTINUOUS = 1
    RANGING_MODE_AUTONOMOUS = 3


@dataclass
class VL53L5CX:
    """
        This class represents a single VL53L5CX sensor.
    """
    id:                   uint8
    data:                 np.ndarray           = np.ones(128, dtype=uint16)
    i2c_address:          uint8                = 0x00
    integration_time_ms:  uint32               = 0
    sharpener_percent:    uint8                = 0
    ranging_frequency_hz: uint8                = 15
    status:               VL53L5CX_Status      = VL53L5CX_Status.OK
    resolution:           VL53L5CX_Resolution  = VL53L5CX_Resolution.RESOLUTION_8X8
    power_mode:           VL53L5CX_PowerMode   = VL53L5CX_PowerMode.POWER_MODE_WAKEUP
    target_order:         VL53L5CX_TargetOrder = VL53L5CX_TargetOrder.TARGET_ORDER_CLOSEST
    ranging_mode:         VL53L5CX_RangingMode = VL53L5CX_RangingMode.RANGING_MODE_AUTONOMOUS
