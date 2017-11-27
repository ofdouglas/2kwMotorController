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


# Short names for faster entry of common commands in interactive mode
pseudo_cmd_table = {
    'config'       : 'set_state system_state config',
    'run'          : 'set_state system_state running',
    'clear'        : 'set_state fault_flags 0',
    'current'      : 'set_state control_mode current',
    'velocity'     : 'set_state control_mode velocity',
    'openloop'     : 'set_state control_mode openloop',
    }


# Read a line from the keyboard and return it, unless an escape char was found
def get_kb_input():
    global exitflag
    try:
        kb_input = input("> ")
    except (KeyboardInterrupt, EOFError) as ex:
        print("\n")
        exitflag = True
        sys.exit()
    return kb_input


# Attempt to create a Message object from user keyboard input
def message_from_kb_cmd():
    cmd_line = get_kb_input()
    m = None
    try:
        first_word = cmd_line.strip().lower().split()[0]
        if first_word in pseudo_cmd_table:
            m = Message(pseudo_cmd_table[first_word])
        else:
            m = Message(cmd_line)
    except Exception:
        print("Command not recognized")
    return m






# Receiver thread: logs 100 Hz sensor data to a CSV file. Also prints low-pass
# filtered sensor data to a TXT file at 1 Hz, for viewing with 'tail -f display.txt'
# Current sensor data is always sent first, so reception of current data is used
# to trigger a write of all the most recently received data.
#
# TODO: need to see FAULT messages any time they are sent!
def freceiver(dev):
    data_file = open('data.csv', 'w')
    display_file = open('display.txt', 'w')
    global exitflag

    sensor_data = [0,0,0,0,0,0,0]
    avg_data    = [0,0,0,0,0,0,0]
    count = 0
    
    while True:
        if exitflag == True:
            sys.exit()
        frame = dev.recv()
        m = Message(frame=frame)
        
        if m.cmd.name == 'fault':
            print(m)

        elif m.cmd.name == 'sensor_data':
            # Update sensor data arrays
            sensor_index = m.args[0]
            sensor_datum = m.args[1]
            avg_data[sensor_index] = avg_data[sensor_index] * 0.95 + sensor_datum * 0.05
            sensor_data[sensor_index] = sensor_datum

            # Print new data
            if m.args[0] == Sensors.CURRENT.value:
                count += 1
                l = ['{:.3f}'.format(x) for x in sensor_data]
                print(','.join(l), file=data_file)
                
                if count == 100:
                    avg_current_str = avg_data[Sensors.CURRENT.value]
                    avg_motor_temp = avg_data[Sensors.MOTOR_TEMP.value]
                    avg_hbridge_temp = avg_data[Sensors.HBRIDGE_TEMP.value]
                    avg_bus_voltage = avg_data[Sensors.BUS_VOLTAGE.value]
                    avg_battery_voltage = avg_data[Sensors.BATTERY_VOLTAGE.value]
                    display_file.write('\033[2J\r')
                    display_file.write("Current         = {:03.2f} A\n".format(avg_current))
                    display_file.write("Motor Temp      = {:03.2f} C\n".format(avg_motor_temp))
                    display_file.write("H-Bridge Temp   = {:03.2f} C\n".format(avg_hbridge_temp))
                    display_file.write("Battery Voltage = {:03.2f} V\n".format(avg_battery_voltage))
                    display_file.write("Bus Voltage     = {:03.2f} V\n".format(avg_bus_voltage))
                    display_file.flush()
                    count = 0
                



def receiver(dev):
    global exitflag
    while True:
        if exitflag == True:
            sys.exit()
        frame = dev.recv()
        m = Message(frame=frame)
        print(m)

        
def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  " + sys.argv[0] + " dryrun")
        print("  " + sys.argv[0] + " can[0..3]")
        
    elif sys.argv[1] == 'dryrun':
        f = can.Frame(id=0<<6, dlc=4, data=[3])
        m = Message(frame=f)
        print(m)
        
        while True:
            m = message_from_kb_cmd()
            if m:
                print(m)
            
    else:
        dev = socketcan.SocketCanDev(sys.argv[1])
        dev.start()
        t = threading.Thread(target=freceiver, args=(dev,))
        t.start()
        while True:
            m = message_from_kb_cmd()
            if m != None:
                dev.send(m.to_frame())

if __name__ == "__main__":
    main()
