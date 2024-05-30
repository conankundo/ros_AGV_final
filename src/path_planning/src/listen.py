#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)

def listen():
    # Khởi tạo node 'listener'
    rospy.init_node('listen', anonymous=True)
    
    # Tạo subscriber, nhận thông điệp String từ topic 'chatter'
    rospy.Subscriber('chatter', String, callback)
    
    # Spin để giữ cho node không kết thúc
    rospy.spin()

if __name__ == '__main__':
    listen()