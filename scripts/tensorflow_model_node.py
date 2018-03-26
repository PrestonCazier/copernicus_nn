#!/usr/bin/env python
import rospy
import pandas as pd
import numpy as np
import tensorflow as tf
import re
import time
from std_msgs.msg import String
tf.__version__

for v in all_vars:
    v_ = sess.run(v)
    print(v_) 
from std_msgs.msg import String

pub = rospy.Publisher('response', String, queue_size=10)

def callback(data):
	  my_str = data.data
    response = 
	  pub.publish(response)
        
def listener():
    rospy.init_node('tensorflow_model', anonymous=True)
    rospy.Subscriber("listen", String, callback)
    rospy.spin()

if __name__ == '__main__':
    sess = tf.Session()
    new_saver = tf.train.import_meta_graph('my-model.meta')
    new_saver.restore(sess, tf.train.latest_checkpoint('./'))
    all_vars = tf.get_collection('vars')
    listener()
