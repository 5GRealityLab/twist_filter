#!/usr/bin/env python
import rospy
from component_filters import IIRTwistFilterObject
from filter_twist import TwistFilter

def main():
    while not rospy.is_shutdown():
        rospy.init_node('iir_filter')

        # Get list of filters to activate, defaults to planar motion x/theta
        active_filters = rospy.get_param('~active_filters', {
            'linear': {
                'x': True,
                'y': False,
                'z': False
            },
            'angular': {
                'x': False,
                'y': False,
                'z': True
            }
        })
        
        # Create component filters
        component_filters = IIRTwistFilterObject(active_filters)

        # Initialize the twist filter
        twist_filter = TwistFilter(component_filters)

        rospy.spin()

if __name__ == '__main__':
    main()