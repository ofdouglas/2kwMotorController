# cmd_enums.py - Enumerations used for Python interface to the
# motor controller
from enum import Enum

# IDs of command strings
class CommandNames(Enum):
    FAULT = 0
    REF = 1
    STOP = 2
    FREEWHEEL = 3
    SET_STATE = 4
    GET_STATE = 5
    RET_STATE = 6
    READ_SENSOR = 7
    SENSOR_DATA = 8
    SET_CONFIG = 9
    GET_CONFIG = 10
    RET_CONFIG = 11
    NODE_ONLINE = 12
    RESET = 13

# Indexes of configuration registers
class ConfigRegisters(Enum):
    CURRENT_KP = 0
    CURRENT_KI = 1
    CURRENT_KD = 2
    VELOCITY_KI = 3
    VELOCITY_KP = 4
    VELOCITY_KD = 5
    POSITION_KP = 6
    POSITION_KI = 7
    POSITION_KD = 8
    CURRENT_DEADBAND = 9
    VELOCITY_DEADBAND = 10
    POSITION_DEADBAND = 11
    MAX_AVG_CURRENT = 12
    MAX_PEAK_CURRENT = 13
    MOTOR_SHUTDOWN_TEMP = 14
    HBRIDGE_FAN_TEMP = 15
    HBRIDGE_SHUTDOWN_TEMP = 16
    MAX_ACCEL_RATE = 17
    DRIVE_MODE = 18
    NODE_ID = 19
    CAN_BAUD_RATE = 20
    UART_BAUD_RATE = 21
    SENSOR_LOG_ENABLES = 22

    
# Indexes of system state registers
class StateRegisters(Enum):
    SYSTEM_STATE = 0
    FAULT_FLAGS = 1
    CONTROL_MODE = 2
    CONTROL_TARGET = 3
    DRIVE_ENABLED = 4

    
# Values of ConfigRegisters.DRIVE_MODE
class DriveModes(Enum):
    SIGN_MAG = 0
    ASYNC_SIGN_MAG = 1
    LOCK_ANTIPHASE = 2

# Values of StateRegisters.SYSTEM_STATE    
class SystemStates(Enum):
    CONFIG = 0
    RUNNING = 1
    FAULTED = 2

# Values of StateRegisters.FAULT_FLAGS
class Faults(Enum):
    OVERCURRENT  = 0x01
    MOTOR_TEMP   = 0x02
    BRIDGE_TEMP  = 0x04
    CONTROL_BATT = 0x08
    CAN_BUS      = 0x10

# Values of StateRegisters.CONTROL_MODE
class ControlModes(Enum):
    OPENLOOP = 0
    CURRENT = 1
    VELOCITY = 2
    POSITION = 3

class Sensors(Enum):
    CURRENT = 0
    VELOCITY = 1
    POSITION = 2
    BUS_VOLTAGE = 3
    BATTERY_VOLTAGE = 4
    MOTOR_TEMP = 5
    HBRIDGE_TEMP = 6
    
