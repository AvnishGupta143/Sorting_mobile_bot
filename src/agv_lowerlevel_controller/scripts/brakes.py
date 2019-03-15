#!/usr/bin/env python
import RPi.GPIO as gpio
import rospy
import geometry_msgs.msg

brakepin = 24
k = 1

def callback(msg):



    global k

    if msg.linear.x == 0.0 and msg.angular.z==0.0:
        gpio.output(brakepin, gpio.HIGH)
	if k == 1:	
		print("brakes on")
		k = 0
    else:
        gpio.output(brakepin, gpio.LOW)
	if k==0:
		print("brakes off")
		k = 1
if __name__ == '__main__':
    gpio.setmode(gpio.BCM)
    gpio.setup(brakepin, gpio.OUT)
    gpio.setwarnings(False)
    rospy.init_node("brakes", anonymous = True)
    en_brake = rospy.Subscriber('/cmd_vel', geometry_msgs.msg.Twist, callback, queue_size = 5)
    rospy.spin()





