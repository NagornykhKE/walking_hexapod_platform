#!/usr/bin/env python3

# license removed for brevity
import math

import rospy

from std_msgs.msg import Float64

def joints ():
    for i in range (4):
        pub_r_f_%d = rospy.Publisher('right_forward%d_position_controller/command',Float64,queue_size = 10) , % i
        #pub_r_f_2 = rospy.Publisher('right_forward2_position_controller/command',Float64,queue_size = 10)
        #pub_r_f_3 = rospy.Publisher('right_forward3_position_controller/command',Float64,queue_size = 10)
        #pub_r_f_4 = rospy.Publisher('right_forward4_position_controller/command',Float64,queue_size = 10)
    rospy.init_node('right_forward_joint_pub', anonymous=True)
    rate = rospy.Rate(10)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = [0.00,-0.56, 2, 0.11]
        rospy.loginfo('Value: right_forward1_position_controller/command {}'.format(msg.data(lst(0)))
        pub_r_f_1.publish(msg)
        rospy.loginfo('Value: right_forward2_position_controller/command {}'.format(msg.data(lst(1)))
        pub_r_f_2.publish(msg)
        rospy.loginfo('Value: right_forward3_position_controller/command {}'.format(msg.data(lst(2)))
        pub_r_f_3.publish(msg)
        rospy.loginfo('Value: right_forward4_position_controller/command {}'.format(msg.data(lst(3)))
        pub_r_f_4.publish(msg)
        rate.sleep()



if __name__ == '__main__':
    try:
       joints ()
    except rospy.ROSInterruptException:
        pass
        
