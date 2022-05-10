#!/usr/bin/env python3

# license removed for brevity
import math

import rospy

from std_msgs.msg import Float64

def forward_joint_pub ():
    pub = rospy.Publisher('right_forward2_position_controller/command',Float64,queue_size = 10)
    rospy.init_node('right_forward_joint_pub', anonymous=True)
    rate = rospy.Rate(10)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = 500    
        'math.sin((rospy.Time.now() - start_time).to_sec() + 1)'
        rospy.loginfo('Value: right_forward2_position_controller/command {}'.format(msg.data))
        pub.publish(msg)
        rate.sleep()

def middle_joint_pub ():
    pub = rospy.Publisher('right_middle2_position_controller/command',Float64,queue_size = 10)
    rospy.init_node('right_middle_joint_pub_2', anonymous=True)
    rate = rospy.Rate(10)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = 2520000
        'math.sin((rospy.Time.now() - start_time).to_sec() + 1)'
        rospy.loginfo('Value: right_middle1_position_controller/command {}'.format(msg.data))
        pub.publish(msg)
        rate.sleep()

def backward_joint_pub_1 ():
    pub = rospy.Publisher('right_backward1_position_controller/command',Float64,queue_size = 10)
    rospy.init_node('right_backward_joint_pub', anonymous=True)
    rate = rospy.Rate(10)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = math.sin((rospy.Time.now() - start_time).to_sec() + 1)
        rospy.loginfo('Value: right_backward1_position_controller/command {}'.format(msg.data))
        pub.publish(msg)
        rate.sleep()
def backward_joint_pub_2 ():
    pub = rospy.Publisher('right_backward2_position_controller/command',Float64,queue_size = 10)
    rospy.init_node('right_backward_joint_pub', anonymous=True)
    rate = rospy.Rate(10)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = math.sin((rospy.Time.now() - start_time).to_sec() + 1)
        rospy.loginfo('Value: right_backward2_position_controller/command {}'.format(msg.data))
        pub.publish(msg)
        rate.sleep()
def backward_joint_pub_3 ():
    pub = rospy.Publisher('right_backward3_position_controller/command',Float64,queue_size = 10)
    rospy.init_node('right_backward_joint_pub', anonymous=True)
    rate = rospy.Rate(10)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = math.sin((rospy.Time.now() - start_time).to_sec() + 1)
        rospy.loginfo('Value: right_backward3_position_controller/command {}'.format(msg.data))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        forward_joint_pub()
        middle_joint_pub()
        backward_joint_pub_1 ()
        backward_joint_pub_2 ()
        backward_joint_pub_3 ()
    except rospy.ROSInterruptException:
        pass
        
