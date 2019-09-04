#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

from filter_types import FilterType, MAFilter

class TwistFilter:
    def __init__(self):
        # Load filter params from parameter server
        try:
            self.set_filter_type()
        except Exception as e:
            rospy.loginfo(e)
            return

        # Set up publishers/subscribers
        self.sub_cmd_in = rospy.Subscriber('cmd_in', Twist, self.callback)

        rospy.loginfo('Filter ready!')

    def set_filter_type(self):
        # Get filter type and sample number
        filter_type = rospy.get_param('filter_type')
        samples = rospy.get_param('samples')

        # Build desired filter
        if filter_type == FilterType.MAF.value:
            self.filter = MAFilter(samples)
        else:
            raise Exception('ERROR - Uknown filter type: ' + str(filter_type))

    def callback(self, data):
        rospy.loginfo(data)

def main():
    while not rospy.is_shutdown():
        try:
            rospy.init_node('twist_filter')
            f = TwistFilter()
            rospy.spin()
        except rospy.ROSInterruptException as e:
            rospy.loginfo(e)

    rospy.loginfo('Shutting down...')

if __name__ == '__main__':
    main()