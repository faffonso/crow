#!/usr/bin/env python3

from __future__ import print_function

from wp_gen.srv import RTInference, RTInferenceResponse, RTInferenceRequest
from wp_gen.msg import CropLine

import rospy

def callback(req):
    # left_line = CropLine(-8.14, 798.21)
    # right_line = CropLine(-8.14, 1471.06)

    # left_line = CropLine(-2.25, 380.0)
    # right_line = CropLine(-2.25, 586.86)

    left_line = CropLine(11.43, -561.97)
    right_line = CropLine(11.43, -1548.71)

    # left_line = CropLine(-57.29, 4348.31)
    # right_line = CropLine(-57.29, 8932.20)
    image = []

    return RTInferenceResponse(left_line, right_line, image)

def add_two_ints_server():
    rospy.init_node('RTInference')
    s = rospy.Service('RTInference', RTInference, callback)
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()