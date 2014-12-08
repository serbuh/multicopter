#!/usr/bin/env python
import mavlink_apm
import re, sys, os, socket, select, time


HOST = ''
mavproxy_port = 12345
# Create a server socket for MAVProxy
mavproxy_sock = socket.socket (socket.AF_INET,socket.SOCK_DGRAM)
mavproxy_sock.setblocking(0)
mavproxy_sock.bind((HOST, mavproxy_port))

mav_obj = mavlink_apm.MAVLink (mavproxy_sock)

# Call to receive data over UDP socket. 1024 is the buffer size
data_from_mavproxy,address_of_mavproxy = mavproxy_sock.recvfrom (1024)
decoded_message = mav_obj.decode(data_from_mavproxy)

print("Got a message with id %u, fields: %s, component: %d, System ID: %d" %
(decoded_message.get_msgId(), decoded_message.get_fieldnames(),
decoded_message.get_srcComponent(), decoded_message.get_srcSystem()))
# Prints the entire decode message
print (decoded_message)