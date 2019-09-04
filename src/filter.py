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
        self.pub_cmd_out = rospy.Publisher('cmd_out', Twist, queue_size=10)

        rospy.loginfo('Filters ready!')

    def set_filter_type(self):
        # Get filter type and sample number
        filter_type = rospy.get_param('filter_type')
        samples = rospy.get_param('samples')

        # Build desired filters
        if filter_type == FilterType.MAF.value:
            self.filters.linear.x = MAFilter(samples)
            self.filters.linear.y = MAFilter(samples)
            self.filters.linear.z = MAFilter(samples)
            self.filters.angular.x = MAFilter(samples)
            self.filters.angular.y = MAFilter(samples)
            self.filters.angular.z = MAFilter(samples)
        else:
            raise Exception('ERROR - Uknown filter type: ' + str(filter_type))

    def filter_twist(self, data):
        filtered_twist = Twist()

        # Get filtered response
        filtered_twist.linear.x = self.filters.linear.x.filter_signal(data.linear.x)
        filtered_twist.linear.y = self.filters.linear.y.filter_signal(data.linear.y)
        filtered_twist.linear.z = self.filters.linear.z.filter_signal(data.linear.z)
        filtered_twist.angular.x = self.filters.angular.x.filter_signal(data.angular.x)
        filtered_twist.angular.y = self.filters.angular.y.filter_signal(data.angular.y)
        filtered_twist.angular.z = self.filters.angular.z.filter_signal(data.angular.z)

        # Publish output twist
        cmd_out = filtered_twist
        self.pub_cmd_out.publish(cmd_out)
        

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