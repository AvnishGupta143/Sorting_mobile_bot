#!/usr/bin/env python
import serial 
import serial.tools.list_ports as port
import time
import sys
import struct
import rospy
from std_msgs.msg import Int16MultiArray

class Barcode_scanner(object):

	def __init__(self):
		super(Barcode_scanner, self).__init__()

		self.ser = serial.Serial()
		
		#Finding the port at which we can connect
		#self.comport = []
		portlist = list(port.comports())
		for p in portlist:
			if "/dev/ttyUSB" in str(p):
				self.comport = str(p).split(" ")				
		
		self.ser.port = self.comport[0]
		self.ser.baudrate = 115200
		self.ser.bytesize = serial.EIGHTBITS
		#self.ser.stopbits = serial.STOPBITS_ONE
		self.ser.parity = serial.PARITY_EVEN
		self.ser.timeout = None       # block read 
		self.ser.xonxoff = False      # disable software flow control
		self.ser.rtscts = False       # disable hardware (RTS/CTS) flow control
		self.ser.dsrdtr = False       # disable hardware (DSR/DTR) flow control
		self.ser.timeout = 0.01
		self.ser.writeTimeout = None  # timeout for write
		self.publish_data_to_rostopic = True

		self.data_pub = rospy.Publisher('/barcode_scanner_data', Int16MultiArray, queue_size = 10)

	def start(self):
		try:
			self.ser.open()
			if(self.ser.isOpen()): 
				print("Communication started successfully at " + self.ser.portstr + "\n")
		except serial.SerialException:
			print("Failed to connect at any Port !!!!....Please check the connection and try again")
			exit()

	def initiate(self):
		initiate_byte1 = bytearray([0xEC])
		initiate_byte2 = bytearray([0x13])
		try:
			write_repsonse = 0
			write_repsonse += self.ser.write(initiate_byte1)
			write_repsonse += self.ser.write(initiate_byte2)
			print(write_repsonse)
			print("Scanner initiated successfully")
		except serial.SerialTimeoutException:
			print(" Initiaiting Failed: Check Connection")

	def reset_buffer(self):
		self.ser.reset_input_buffer()
		self.ser.reset_output_buffer()
		time.sleep(0.3)

	def write(self):
		try:
			num_sent = 0
			write_byte = bytearray([0xC8])
			num_sent += self.ser.write(write_byte)
			write_byte = bytearray([0x37])
			num_sent += self.ser.write(write_byte)
			#print("Sent: ", num_sent)
		except serial.SerialTimeoutException:
			print(" Write Failed")

	def read(self):
		bytes_received = self.ser.readline()
		bytes_received_list = []
		if len(bytes_received) == 21:
			for b in bytes_received:
				byte_str = ''.join(b)
				num, = struct.unpack('>B', byte_str)
				bytes_received_list.append(num)

		#print("Received: ", len(bytes_received_list)," Bytes")
		#print(bytes_received_list)
			aligned = self.check_alignment(bytes_received_list[0])
			if aligned:
				self.process_data(bytes_received_list)

 	def check_alignment(self,first_byte):
 		if first_byte & 2 :
 			return False
 		else:
 			return True

	def process_data(self,bytes_received):
		X_value  = int(bytes_received[5])-int(bytes_received[4])
		Y_value  = int(bytes_received[7])-int(bytes_received[6])
		Ang_value= int(bytes_received[10])*128 + int(bytes_received[11])
		Tag_id   = int(bytes_received[17]) - int(bytes_received[16])
		Marker   = 1
            
		#print("X_value: %d | Y_value: %d | Ang_value: %d | Tag_id: %d | Marker: %d" %(X_value,Y_value,Ang_value,Tag_id,Marker))

		if self.publish_data_to_rostopic == True:
			self.publish_data(X_value,Y_value,Ang_value,Tag_id,Marker)

	def publish_data(self,X_value,Y_value,Ang_value,Tag_id,Marker):
		msg = Int16MultiArray()
		msg.data = [X_value,Y_value,Ang_value,Tag_id,Marker]
		self.data_pub.publish(msg) 

if __name__ == '__main__':
	#while port is opened
	rospy.init_node('Barcode_scanner', anonymous=True)
	Port = Barcode_scanner()
	#raw_input()
	Port.start()
	Port.initiate()
	while(1):
		Port.write()
		Port.read()
	Port.ser.close()
