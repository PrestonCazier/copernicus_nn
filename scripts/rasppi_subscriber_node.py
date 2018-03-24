#!/usr/bin/env python
import rospy
import google.cloud
import os
from std_msgs.msg import String


def callback(data):
	my_str = data.data
	engine = pttsx3.init()
	engine.say(my_str)
	engine.runAndWait()
        
		
def listener():
    rospy.init_node('speaker', anonymous=True)
    rospy.Subscriber("response_text", String, callback)
    rospy.spin()


if __name__ == 
'__main__':
	listener()
