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


def question_to_seq(question, vocab_to_int):
    '''Prepare the question for the model'''
    question = clean_text(question)
    return [vocab_to_int.get(word, vocab_to_int['<UNK>']) for word in question.split()]


def callback(data):
    my_str = data.data
    # Prepare the question
    input_question = question_to_seq(input_question, questions_vocab_to_int)
    # Pad the questions until it equals the max_line_length
    input_question = input_question + [questions_vocab_to_int["<PAD>"]] * (max_line_length - len(input_question))
    # Add empty questions so the the input_data is the correct shape
    batch_shell = np.zeros((batch_size, max_line_length))
    # Set the first question to be out input question
    batch_shell[0] = input_question    
    # Run the model with the input question
    answer_logits = sess.run(inference_logits, {input_data: batch_shell, 
                                                keep_prob: 1.0})[0]
    # Remove the padding from the Question and Answer
    pad_q = questions_vocab_to_int["<PAD>"]
    pad_a = answers_vocab_to_int["<PAD>"]
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
