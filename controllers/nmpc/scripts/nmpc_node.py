#!/usr/bin/env python3

import rospy
import yaml

from nmpc.nmpc_controller import NMPC

def main():
    rospy.init_node('nmpc_controller_node')

    rate = rospy.Rate(12) 

    params = rospy.get_param("controller/NMPC")
    control = NMPC(**params)
    
    while not rospy.is_shutdown():
        control.run()
        rate.sleep()

if __name__ == '__main__':
    main()