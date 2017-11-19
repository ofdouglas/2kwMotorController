#!/usr/bin/python3

from CANard.canard import can
from CANard.canard.hw import socketcan
import struct
import sys
import threading
import cmd_enums
    
class Command:
    def __init__(self, id, usable, args_fmt,
                 args_enums_list=None):
        self.id = id
        self.usable = usable
        self.args_fmt = args_fmt
        self.args_enums_list = args_enums_list

    # TODO: this will accept any enum from args_enums_list
    # for any position... meaning a broken statement like
    # set_state CONFIG system_state is accepted, even though
    # the arguments are switched! The resulting command will
    # be incorrect any may have unfortunate results...
    def parse_int(self, string):
        retval = None
        try:
            retval = int(string)
        except:
            for e in self.args_enums_list:
                if string in e.__members__:
                    # x.value is a tuple... wtf?
                    x = e[string]
                    retval = x.value[0]
                    break;
            else:
                raise ValueError('Unable to parse int arg')
        return retval
                                  
    def parse_args(self, args_stringings):
        strings = iter(args_stringings)
        args_bin = []
        
        for c in self.args_fmt:
            s = next(strings).upper()
            if c == 'f':
                args_bin.append(float(s))
            elif c == 'b' or c == 'i':
                args_bin.append(self.parse_int(s))
            else:
                assert 0, 'Fatal error'
                
        return struct.pack('<' + self.args_fmt, *args_bin)


state_enums = [ cmd_enums.StateRegisters,
                cmd_enums.SystemStates,
                cmd_enums.Faults,
                cmd_enums.ControlModes ]

config_enums = [ cmd_enums.ConfigRegisters,
                 cmd_enums.DriveModes ]
    
# Command table
# Dict values: (cmd enum, user-sendable, args format)
# The args format is the same used by Python's stringuct module,
# except that '%' will indicate an arg whose type is defined
# by the previous arg.
cmd_table = {
    0  : Command(0, False, ''),
    1  : Command(1, True, 'f'),
    2  : Command(2, True, ''),
    3  : Command(3, True, ''),
    4  : Command(4, True, 'bi', state_enums),
    5  : Command(5, True, 'b', state_enums),
    6  : Command(6, False, 'bi'),
    7  : Command(7, True, 'b', cmd_enums.Sensors),
    8  : Command(8, False, 'bi'),
    8  : Command(9, True, 'bf', config_enums),
    10 : Command(10, True, 'b', config_enums),
    11 : Command(11, False, 'bf'),
    12 : Command(12, True, 'bi'),
    13 : Command(13, True, 'bi')
    }
    

def get_cmd_line():
    try:
        kb_input = input("> ").split()
    except KeyboardInterrupt:
        print("\n")
        sys.exit()
    return kb_input

def lookup_cmd(string):
    index = cmd_enums.CommandNames[string.upper()].value
    cmd = cmd_table[index]
    if cmd.usable != True:
        raise LookupError('Invalid command')
    return cmd
    
def send_packet(dev, cmd, data):
    frame = can.Frame(id=cmd.id << 6,
                      dlc=len(data),
                      data = list(data))
    print(frame)
    dev.send(frame)


def send_cmd(dev, cmd_line):
    try:
        cmd = lookup_cmd(cmd_line[0])
    except Exception as ex:
        print('lookup_cmd failed: ' + str(ex))
        return
    
    try:
        data = cmd.parse_args(cmd_line[1:])
    except Exception as ex:
        print('parse args failed: ' + str(ex))
        return
    
    send_packet(dev, cmd, data)
    

def receiver(dev):
    while True:
        frame = dev.recv()
        cmd_index = frame.id >> 6
        cmd_string = cmd_enums.CommandNames(cmd_index)
        cmd = cmd_table[cmd_index]
        
        if cmd_string == cmd_enums.CommandNames['SENSOR_DATA']:
            assert frame.dlc == 5, "Malformed sensor data packet"
            data = bytearray(frame.data[:5])
            (sensor_index, value) = struct.unpack('<bf', data)
            sensor_name = cmd_enums.Sensors(sensor_index).name
            print(sensor_name.lower(), " = {:.3f}".format(value))
        else:
            print(cmd_string)

    
def main():
    dev = socketcan.SocketCanDev(sys.argv[1])
    dev.start()

    t = threading.Thread(target=receiver, args=(dev,))
    t.start()
    
    while True:
        cmd_line = get_cmd_line()
        send_cmd(dev, cmd_line)


if __name__ == "__main__":
    main()
