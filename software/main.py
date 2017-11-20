#!/usr/bin/python3
#
# Client program to communicate with the motor controller
#

from CANard.canard import can
from CANard.canard.hw import socketcan

import struct
import sys
import threading

from command import *

def send_packet(dev, cmd, data):
    frame = can.Frame(id=cmd.id << 6,
                      dlc=len(data),
                      data = list(data))
    print(frame)
    dev.send(frame)

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

def get_cmd_line():
    try:
        kb_input = input("> ")
    except KeyboardInterrupt:
        print("\n")
        sys.exit()
    return kb_input
    
def main():
#    dev = socketcan.SocketCanDev(sys.argv[1])
#    dev.start()
#    t = threading.Thread(target=receiver, args=(dev,))
#    t.start()
    
    while True:
        cmd_line = get_cmd_line()
        m = Message(cmd_line)
        print(m)


if __name__ == "__main__":
    main()
