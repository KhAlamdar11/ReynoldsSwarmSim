#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import random

#NOTE: DEVELOPMENT IN PROGRESS

def state_publisher():
    rospy.init_node('state_publisher')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['joint1', 'joint2', 'joint3']
        joint_state.position = [random.random(), random.random(), random.random()]
        joint_state.velocity = []
        joint_state.effort = []

        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        state_publisher()
    except rospy.ROSInterruptException:
        pass
