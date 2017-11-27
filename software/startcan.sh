#!/bin/bash

# s4 == 125kbaud
# s8 == 1Mbaud
sudo slcand -o -c -s8 /dev/serial/by-id/*CANtact*-if00 can0
sudo ip link set up can0
#sudo ifconfig can0 up



# Helpful commands for testing CAN:
###################################
# Send a frame to 0x999 with payload 0xdeadbeef:
# > cansend can0 999#DEADBEEF
#
# Show all traffic received by can0:
# > candump can0
#
# Calculate bus loading percentage on can0:
# > canbusload can0 500000
#
# Display top-style view of can traffic:
# > cansniffer can0
#
# Generate fixed-data CAN messages:
# > cangen can0 -D 11223344DEADBEEF -L 8    
