#!/usr/bin/env python
import time
import serial
from message import *
from plutodrone.srv import *
from plutodrone.msg import *
from std_msgs.msg import Int16
import rospy

def read_msg(serialdev):
	import select
	rospy.init_node('drone_server')
	

	msglen = 0
	msg = None
	s = 0
	while True:
		try:
			c = ord(serialdev.read(1))
		except select.error: #normal syscal interrupt
			continue
		if s == 0:
			s = (c == MSGSTART)
		elif s == 1:
			if c in MSGMAP:
				s = 2
				msg = [c] + [0]*(MSGMAP[c]-1)
				msglen = 1
			else: #we are not in sync, reset
				s = 0
		else:
			msg[msglen] = c
			msglen += 1
			if msglen >= MSGMAP[msg[0]]:
				if checksum(msg):
					return msg
				s = 0

ser = serial.Serial(
               port='/dev/ttyUSB0',
               baudrate = 115200,
               parity=serial.PARITY_NONE,
               stopbits=serial.STOPBITS_ONE,
               bytesize=serial.EIGHTBITS,
               timeout=1
      )

while 1:
	command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
	cmd = PlutoMsg()
	msg=read_msg(ser)
	print msg
	channel_values = [0, 0, 0, 0, 0, 0]
	for i, channel in enumerate(channels):
		channel_values[i] = channel.read(msg)
	# print channel_values
	cmd.rcRoll =channel_values[0]
	cmd.rcPitch = channel_values[1]
	cmd.rcYaw =channel_values[3]
	cmd.rcThrottle =channel_values[2]
	cmd.rcAUX1 =1500
	cmd.rcAUX2 =channel_values[4]
	cmd.rcAUX3 =1500
	cmd.rcAUX4 =channel_values[5]
	command_pub.publish(cmd)

