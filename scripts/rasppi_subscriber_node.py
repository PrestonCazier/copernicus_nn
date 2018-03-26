#!/usr/bin/env python
import rospy
import pttsx3
import os
from std_msgs.msg import String


def callback(data):
	my_str = data.data
	engine = pttsx3.init()
	engine.say(my_str)
	engine.runAndWait()
        
		
def responder():
    rospy.init_node('responder', anonymous=True)
    rospy.Subscriber("response", String, callback)
    rospy.spin()


if __name__ == 
'__main__':
	listener()
