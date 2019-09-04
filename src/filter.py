#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

from filter_types import FilterType, MAFilter

class TwistFilterComponent:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None

class TwistFilterObj:
    def __init__(self):
        self.linear = TwistFilterComponent()
        self.angular = TwistFilterComponent()

class TwistFilter:
    def __init__(self):
        # Load filter params from parameter server
        try:
            self.filters = TwistFilterObj()
            self.set_filter_type()
        except Exception as e:
            rospy.loginfo(e)
            return

        # Set up publishers/subscribers
        self.sub_cmd_in = rospy.Subscriber('cmd_in', Twist, self.filter_twist)

        rospy.loginfo('Filter ready!')

    def _create_filters(self, f):
        '''
        @brief Populates self.filters with desired filter type for linear
               and angular x/y/z values
        
        @param f - filter object (from filter_types.py)
        '''

        self.filters.linear.x = f
        self.filters.linear.y = f
        self.filters.linear.z = f
        self.filters.angular.x = f
        self.filters.angular.y = f
        self.filters.angular.z = f

    def set_filter_type(self):
        # Get filter type and sample number
        filter_type = rospy.get_param('filter_type')
        samples = rospy.get_param('samples')

        # Build desired filters
        if filter_type == FilterType.MAF.value:
            f = MAFilter(samples)
            self._create_filters(f)
        else:
            raise Exception('ERROR - Uknown filter type: ' + str(filter_type))

    def filter_twist(self, data):
        rospy.loginfo(data)
        # Get filtered response
        self.filter.filter_signal(data)

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