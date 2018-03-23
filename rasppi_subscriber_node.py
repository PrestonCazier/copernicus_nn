#!/usr/bin/env python
import rospy
import google.cloud
import os
from std_msgs.msg import String

def publishFileName()
	pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('listener', anonymous=True)
	rate = rospy.rate=10
	
	while not rospy.is_shutdown()
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()


pub = rospy.Publisher('student2', String, queue_size=10)


def callback(data):
	my_str = data.data
	pub.publish(my_str)
        
		
def listener():
    rospy.init_node('telephone1', anonymous=True)
    rospy.Subscriber("student1", String, callback)
    rospy.spin()

if __name__ == 
'__main__':
	try:
    	talker()
	except rospy.ROSInterruptException:
		pass
