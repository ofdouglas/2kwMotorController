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

exitflag = False

def get_cmd_line():
    global exitflag
    try:
        kb_input = input("> ")
    except KeyboardInterrupt:
        print("\n")
        exitflag = True
        sys.exit()
    return kb_input


def freceiver(dev):
    f = open('temp.csv', 'w')
    global exitflag
    i = 0
    while True:
        if exitflag == True:
            sys.exit()
        frame = dev.recv()
        m = Message(frame=frame)
        print(m, file=f)
        if i == 99:
            print(m)
            i = 0
        else:
            i = i + 1

def receiver(dev):
    global exitflag
    while True:
        if exitflag == True:
            sys.exit()
        frame = dev.recv()
        m = Message(frame=frame)
        print(m)

    
def main():
    dev = socketcan.SocketCanDev(sys.argv[1])
    dev.start()
    t = threading.Thread(target=freceiver, args=(dev,))
    t.start()
    
    while True:

        # m = Message(string='set_config sensor_log_enables 0')
        # dev.send(m.to_frame())
        
        cmd_line = get_cmd_line()
        m = Message(cmd_line)
        dev.send(m.to_frame())
#        print(m)


if __name__ == "__main__":
    main()
