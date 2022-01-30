#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkState

def main():
    rospy.init_node('rotor_rotate')
    state_right = LinkState()
    state_left = LinkState()
    state_front = LinkState()
    state_back = LinkState()
    pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
    rate = rospy.Rate(1)
    rotor_vel = 0.1
    state_right.link_name = 'rotor_right_1'
    state_right.twist.angular.z = rotor_vel
    state_left.link_name = 'rotor_left_1'
    state_left.twist.angular.z = -rotor_vel
    state_front.link_name = 'rotor_front_1'
    state_front.twist.angular.z = rotor_vel
    state_back.link_name = 'rotor_back_1'
    state_back.twist.angular.z = -rotor_vel
    while not rospy.is_shutdown():
        pub.publish(state_right)
        pub.publish(state_left)
        pub.publish(state_front)
        pub.publish(state_back)
        rate.sleep()

if __name__=='__main__':
    try: main()
    except rospy.ROSInterruptException: pass