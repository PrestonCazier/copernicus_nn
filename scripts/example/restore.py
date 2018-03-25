#!/usr/bin/env python
import pandas as pd
import numpy as np
import tensorflow as tf
import re
import time
tf.__version__

sess = tf.Session()
new_saver = tf.train.import_meta_graph('my-model.meta')
new_saver.restore(sess, tf.train.latest_checkpoint('./'))
all_vars = tf.get_collection('vars')
for v in all_vars:
    v_ = sess.run(v)
    print(v_) 
