from enum import Enum

class Register:
    def __init__(self, index, name, typechar, enums=None):
        self.index = index
        self.name = name
        self.typechar = typechar
        self.enums = enums

    def __str__(self):
        l = ['Register', str(self.index), '(' + self.name + ')',
             'type:', self.typechar]
        return ' '.join(l)


################################################################################
# Configuration Registers
################################################################################
        
# Values of ConfigRegisters.DRIVE_MODE
class DriveModes(Enum):
    SIGN_MAG = 0
    ASYNC_SIGN_MAG = 1
    LOCK_ANTIPHASE = 2

reg_current_kp =            Register(0, 'current_kp', 'f')
reg_current_ki =            Register(1, 'current_ki', 'f')
reg_current_kd =            Register(2, 'current_kd', 'f')
reg_velocity_kp =           Register(3, 'velocity_kp', 'f')
reg_velocity_ki =           Register(4, 'velocity_ki', 'f')
reg_velocity_kd =           Register(5, 'velocity_kd', 'f')
reg_position_kp =           Register(6, 'position_kp', 'f')
reg_position_ki =           Register(7, 'position_ki', 'f')
reg_position_kd =           Register(8, 'position_kd', 'f')
reg_current_deadband =      Register(9, 'current_deadband', 'f')
reg_velocity_deadband =     Register(10, 'velocity_deadband', 'f')
reg_position_deadband =     Register(11, 'position_deadband', 'f')
reg_max_avg_current =       Register(12, 'max_avg_current', 'f')
reg_max_peak_current =      Register(13, 'max_peak_current', 'f')
reg_motor_shutdown_temp =   Register(14, 'motor_shutdown_temp', 'f')
reg_hbridge_fan_temp =      Register(15, 'hbridge_fan_temp', 'f')
reg_hbridge_shutdown_temp = Register(16, 'hbridge_shutdown_temp', 'f')
reg_max_accel_rate =        Register(17, 'max_accel_rate', 'f')
reg_drive_mode =            Register(18, 'drive_mode', 'i', enums=DriveModes)
reg_node_id =               Register(19, 'node_id', 'i'),
reg_can_baud_rate =         Register(20, 'can_baud_rate', 'i')
reg_uart_baud_rate =        Register(21, 'uart_baud_rate', 'i')
reg_sensor_log_enables =    Register(22, 'sensor_log_enables', 'i')

config_reg_map = {
    0                       : reg_current_kp,
    'current_kp'            : reg_current_kp,
    1                       : reg_current_ki,
    'current_ki'            : reg_current_ki,
    2                       : reg_current_kd,
    'current_kd'            : reg_current_kd,
    3                       : reg_velocity_kp,
    'velocity_kp'           : reg_velocity_kp,
    4                       : reg_velocity_ki,
    'velocity_ki'           : reg_velocity_ki,
    5                       : reg_velocity_kd,
    'velocity_kd'           : reg_velocity_kd,
    6                       : reg_position_kp,
    'position_kp'           : reg_position_kp,
    7                       : reg_position_ki,
    'position_ki'           : reg_position_ki,
    8                       : reg_position_kd,
    'position_kd'           : reg_position_kd,
    9                       : reg_current_deadband,
    'current_deadband'      : reg_current_deadband,
    10                      : reg_velocity_deadband,
    'velocity_deadband'     : reg_velocity_deadband,
    11                      : reg_position_deadband,
    'position_deadband'     : reg_position_deadband,
    12                      : reg_max_avg_current,
    'max_avg_current'       : reg_max_avg_current,
    13                      : reg_max_peak_current,
    'max_peak_current'      : reg_max_peak_current,
    14                      : reg_motor_shutdown_temp,
    'motor_shutdown_temp'   : reg_motor_shutdown_temp,
    15                      : reg_hbridge_fan_temp,
    'hbridge_fan_temp'      : reg_hbridge_fan_temp,
    16                      : reg_hbridge_shutdown_temp,
    'hbridge_shutdown_temp' : reg_hbridge_shutdown_temp,
    17                      : reg_max_accel_rate,
    'max_accel_rate'        : reg_max_accel_rate,
    18                      : reg_drive_mode,
    'drive_mode'            : reg_drive_mode,
    19                      : reg_node_id,
    'node_id'               : reg_node_id,
    20                      : reg_can_baud_rate,
    'can_baud_rate'         : reg_can_baud_rate,
    21                      : reg_uart_baud_rate,
    'uart_baud_rate'        : reg_uart_baud_rate,
    22                      : reg_sensor_log_enables,
    'sensor_log_enables'    : reg_sensor_log_enables
}


################################################################################
# System Registers
################################################################################


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

    
reg_system_state =   Register(0, 'system_state', 'i', enums=SystemStates)
reg_fault_flags =    Register(1, 'fault_flags', 'i', enums=Faults)
reg_control_mode =   Register(2, 'control_mode', 'i', enums=ControlModes)
reg_control_target = Register(3, 'control_target', 'f')
reg_drive_enabled =  Register(4, 'drive_enabled', 'i')

state_reg_map = {
    0                 : reg_system_state,
    'system_state'    : reg_system_state,
    1                 : reg_fault_flags,
    'fault_flags'     : reg_fault_flags,
    2                 : reg_control_mode,
    'control_mode'    : reg_control_mode,
    3                 : reg_control_target,
    'control_target'  : reg_control_target,
    4                 : reg_drive_enabled,
    'drive_enabled'   : reg_drive_enabled
}


################################################################################
# Sensors - treated as 'virtual registers'
################################################################################

# Enumeration of different sensors (each treated as a register)
class Sensors(Enum):
    CURRENT = 0
    VELOCITY = 1
    POSITION = 2
    BUS_VOLTAGE = 3
    BATTERY_VOLTAGE = 4
    MOTOR_TEMP = 5
    HBRIDGE_TEMP = 6

sensor_current =         Register(0, 'current', 'f', enums=Sensors)
sensor_velocity =        Register(1, 'velocity', 'f', enums=Sensors)
sensor_position =        Register(2, 'position', 'f')
sensor_bus_voltage =     Register(3, 'bus_voltage', 'f')
sensor_battery_voltage = Register(4, 'battery_voltage', 'f')
sensor_motor_temp =      Register(5, 'motor_temp', 'f')
sensor_hbridge_temp =    Register(6, 'hbridge_temp', 'f')

sensor_reg_map = {
    0                 : sensor_current,
    'current'         : sensor_current,
    1                 : sensor_velocity,
    'velocity'        : sensor_velocity,
    2                 : sensor_position,
    'position'        : sensor_position,
    3                 : sensor_bus_voltage,
    'bus_voltage'     : sensor_bus_voltage,
    4                 : sensor_battery_voltage,
    'battery_voltage' : sensor_battery_voltage,
    5                 : sensor_motor_temp,
    'motor_temp'      : sensor_motor_temp,
    6                 : sensor_hbridge_temp,
    'hbridge_temp'    : sensor_hbridge_temp
}


    
################################################################################
# Old enums - for developer reference only
################################################################################

    
# Id's of command strings
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

    


