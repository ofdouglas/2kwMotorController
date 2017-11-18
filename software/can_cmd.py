#!/usr/bin/python3

from CANard.canard import can
from CANard.canard.hw import socketcan
import struct
import sys

# Command table
# Dict values: (cmd enum, user-sendable, args format)
#  args format is the same used by Python's struct module
#
cmd_table = {
    'fault'       : (0, False, ''),
    'ref'         : (1, True, 'f'),
    'stop'        : (2, True, ''),
    'freewheel'   : (3, True, ''),
    'set_state'   : (4, True, 'bi'),
    'get_state'   : (5, True, 'b'),
    'ret_state'   : (6, False, 'bi'),
    'read_sensor' : (7, True, 'b'),
    'sensor_data' : (8, False, 'bi'),
    'set_config'  : (9, True, 'bi'),
    'get_config'  : (10, True, 'b'),
    'ret_config'  : (11, False, 'bi'),
    'node_online' : (12, True, 'bi'),
    'reset'       : (13, True, 'bi')
    }
    
dev = socketcan.SocketCanDev(sys.argv[1])
dev.start()
frame = can.Frame(id=0)

while True:
    try:
        kb_input = input("> ").split()
    except KeyboardInterrupt:
        print("\n")
        sys.exit()
    try:
        cmd = cmd_table[kb_input[0]]
        assert cmd[1] == True
    except:
        print("Invalid command")
        continue

    try:
        raw_args = iter(kb_input[1:])
        fmt_args = []
        for c in cmd[2]:
            if c == '':
                pass
            elif c == 'b' or c == 'i':
                fmt_args.append(int(next(raw_args)))
            elif c == 'f':
                fmt_args.append(float(next(raw_args)))
            else:
                assert 0, "Fatal error"
        data = struct.pack('<' + cmd[2], *fmt_args)
    except Exception as e:
        print(e)
        continue

    frame.id = cmd[0] << 6
    frame.dlc = len(data)
    frame.data = list(data)
    print(frame)
    dev.send(frame)
