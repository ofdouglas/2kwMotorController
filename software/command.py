#!/usr/bin/python3
#
# Library that defines CAN communication with motor controller
#

import struct
from enum import Enum
from CANard.canard import can
from registers import *
 
################################################################################
# Command class and command table
################################################################################

class Command:
    def __init__(self, index, name, argtypes, regmap=None):
        self.index = index        # integer representation of command
        self.name = name          # string representation
        self.argtypes = argtypes  # list of argument types as chars
        self.regmap = regmap      # (optional) dictionary of register objects
        
cmd_fault =       Command(0, 'fault', '')
cmd_ref =         Command(1, 'ref', 'f')
cmd_stop =        Command(2, 'stop', '')
cmd_freewheel =   Command(3, 'freewheel', '')
cmd_set_state =   Command(4, 'set_state', 'br', regmap=state_reg_map)
cmd_get_state =   Command(5, 'get_state', 'b', regmap=state_reg_map)
cmd_ret_state =   Command(6, 'ret_state', 'br', regmap=state_reg_map)
cmd_read_sensor = Command(7, 'read_sensor', 'b', regmap=sensor_reg_map)
cmd_sensor_data = Command(8, 'sensor_data', 'br', regmap=sensor_reg_map)
cmd_set_config =  Command(9, 'set_config', 'br', regmap=config_reg_map)
cmd_get_config =  Command(10, 'get_config', 'b', regmap=config_reg_map)
cmd_ret_config =  Command(11, 'ret_config', 'br', regmap=config_reg_map)
cmd_node_online = Command(12, 'node_online', '')
cmd_reset =       Command(13, 'reset', '')

cmd_table = {
    0             : cmd_fault,
    'fault'       : cmd_fault,
    1             : cmd_ref,
    'ref'         : cmd_ref,
    2             : cmd_stop,
    'stop'        : cmd_stop,
    3             : cmd_freewheel,
    'freewheel'   : cmd_freewheel,
    4             : cmd_set_state,
    'set_state'   : cmd_set_state,
    5             : cmd_get_state,
    'get_state'   : cmd_get_state,
    6             : cmd_ret_state,
    'ret_state'   : cmd_ret_state,
    7             : cmd_read_sensor,
    'read_sensor' : cmd_read_sensor,
    8             : cmd_sensor_data,
    'sensor_data' : cmd_sensor_data,
    9             : cmd_set_config,
    'set_config'  : cmd_set_config,
    10            : cmd_get_config,
    'get_config'  : cmd_get_config,
    11            : cmd_ret_config,
    'ret_config'  : cmd_ret_config,
    12            : cmd_node_online,
    'node_online' : cmd_node_online,
    13            : cmd_reset,
    'reset'       : cmd_reset
}


################################################################################
# Message class - specific instances of a command
################################################################################
#
# self.args  : list of arguments in native format (e.g. [uint8_t, uint32_t] is 2 items)
#
#
#
class Message:
    def __init__(self, string=None, frame=None):
        if type(string) is str:
           self.init_from_string(string)
        elif type(frame) is can.Frame:
            self.init_from_frame(frame)
        else:
            raise TypeError('Bad arguments to Message constructor')

        
    def __str__(self):
        l = [self.cmd.name]
        args_iter = iter(self.args)
        register = None
        
        for c in self.cmd.argtypes:
            arg = next(args_iter)
            if c == 'f' or c == 'i':
                l.append(str(arg))
            elif c == 'b':
                register = self.cmd.regmap[arg]
                l.append(register.name)
            elif c == 'r':
                if register.typechar == 'i':
                    l.append(register.enums(arg).name)
                elif register.typechar == 'f':
                    l.append(str(arg))
                else:
                    raise ValueError('Unexpected type character')
        return ' '.join(l)

    
    def to_frame(self):
        """ Generate a CAN frame from a Message object
        """
        struct_str = '<' + ''.join(self.argtypes)
        data_bytes = struct.pack(struct_str, *self.args)
        f = can.Frame(id = self.cmd.index << 6, dlc = len(data_bytes),
                      data = list(data_bytes))
        return f

    
    def init_from_frame(self, frame):
        """ Initialize a Message object from a CAN frame.
        """
        self.cmd = cmd_table[frame.id >> 6]
        self.args = []
        self.argtypes = list(self.cmd.argtypes)

        if ''.join(self.argtypes) == 'br':
            register = self.cmd.regmap[frame.data[0]]
            self.argtypes[1] = register.typechar

        struct_str = '<' + ''.join(self.argtypes)
        raw_bytes = bytearray(frame.data[:frame.dlc])
        self.args = list(struct.unpack(struct_str, raw_bytes))

        
    def __parse_reg_arg(self, s, register):
        """ Parse a string into a register argument (helper for init_from_string)
        """
        assert register != None, "Register argument without register index"
        if register.typechar == 'f':
            self.args.append(float(s))
            c = 'f'
        elif register.typechar == 'i':
            c = 'i'
            try:
                self.args.append(register.enums[s.upper()].value)
            except:
                try:
                    self.args.append(int(s))
                except Exception as ex:
                    raise ex
        return c

    
    def init_from_string(self, string):
        """ Initialize a Message object from a text command line
        """
        strings_iter = iter(string.strip().lower().split())
        register = None
        self.cmd = cmd_table[next(strings_iter)]
        self.args = []
        self.argtypes = []

        for c in self.cmd.argtypes:
            s = next(strings_iter)
            if c == 'f':                # arg type == float
                self.args.append(float(s))
            elif c == 'i':              # arg type == integer
                self.args.append(int(s))
            elif c == 'b':              # arg type == register index
                if s in self.cmd.regmap:
                    register = self.cmd.regmap[s]
                    self.args.append(register.index)
                elif int(s) in self.cmd.regmap:
                    register = self.cmd.regmap[int(s)]
                    self.args.append(register.index)
                else:
                    raise ValueError('Parse register index failed')
            elif c == 'r':             # arg type is defined by register map
                c = self.__parse_reg_arg(s, register)
                
            self.argtypes.append(c)



#print(Message(string='ref 15'))   
#print(Message(string='set_state system_state running'))
#print(Message(string='set_state system_state 1'))
#print(Message(string='set_state 0 1'))


#f = can.Frame(id=6<<6, dlc=5, data=[1, 2, 0, 0, 0,])
#m = Message(frame=f)
#print(m)
#f2 = m.to_frame()
#print(f2)
#print(f)
