#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class TwistFilter:
    def __init__(self):
        # Set up publishers/subscribers
        self.cmd_in = rospy.Subscriber('cmd_in', Twist, self.callback)

        rospy.loginfo('Filter ready!')

    def callback(self, data):
        rospy.loginfo(data)

def main():
    while not rospy.is_shutdown():
        try:
            rospy.init_node('twist_filter')
            filter = TwistFilter()
            rospy.spin()
        except rospy.ROSInterruptException as e:
            rospy.loginfo(e)

    rospy.loginfo('Shutting down...')

if __name__ == '__main__':
    main()