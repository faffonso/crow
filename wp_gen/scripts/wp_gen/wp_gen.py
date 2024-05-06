#!/usr/bin/env python3

from __future__ import print_function

from wp_gen.msg import CropLine

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
import colorful as cf
import math
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Wp_gen():
    def __init__(self, row_height, row_width, D, img_height, img_width):
        """ 
        Waypoint generator

        Generates waypoints using reference corn lines predicted using
        a neural network applied to point clouds (LiDAR).
        The lines are in the following format: y = mx + b

        Attributes:
            row_height: Real height of the image in meters
            row_width: Real width of the image in meters 
            img_height: Image height resolution in pixels
            img_width: Image width resolution in pixels
            odom_sub: ROS subscriber for odometry messages
            goal_pub: ROS publisher for goal messages
            D: Euclidean distance of robot to waypoints
        """

        self.row_height = row_height
        self.row_width = row_width

        self.img_height = img_height
        self.img_width = img_width

        self.old_m1 = 0
        self.old_m2 = 0

        odom_topic = rospy.get_param("wp_gen/odom/topic")
        frame_id = rospy.get_param("wp_gen/odom/frame_id")


        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.line_sub = rospy.Subscriber('/terrasentia/crop_lines', CropLine, self.line_callback)
        self.goal_pub = rospy.Publisher('/terrasentia/goal', PoseStamped, queue_size=10)

        self.odom_msg = Odometry()
        self.line_msg = CropLine()
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = frame_id
        
        self.D = D
        self.run_en = False

        rospy.loginfo(cf.green(f"Waypoint Generator created!"))


    def run(self, show=False, verbose=False):
        if (self.run_en == False):
            rospy.loginfo(cf.orange(f"Without new crop lines"))
            return

    
        rospy.loginfo(cf.green(f"New crop lines"))


        m1 = self.line_msg.m1
        c1 = self.line_msg.b1

        m2 = self.line_msg.m2
        c2 = self.line_msg.b2

        rospy.loginfo(cf.yellow(f'Calculating waypoint (Line1 m={m1}, b={c1}; Line2 m={m2}, b={c2})'))

        x, y = self.get_target(m1, m2, c1, c2)

        rospy.loginfo(cf.orange(f'Y={y}, X={x}'))

        q = (
            self.odom_msg.pose.pose.orientation.x,
            self.odom_msg.pose.pose.orientation.y,
            self.odom_msg.pose.pose.orientation.z,
            self.odom_msg.pose.pose.orientation.w)
        _, _, heading_global = euler_from_quaternion(q)

        heading = np.arctan2(x, y)

        self.goal_msg.pose.position.x = self.odom_msg.pose.pose.position.x + x * np.sin(heading_global) + y * np.cos(heading_global)
        self.goal_msg.pose.position.y = self.odom_msg.pose.pose.position.y - (x * np.cos(heading_global) - y * np.sin(heading_global))

        q = quaternion_from_euler(0.0, 0.0, - heading + heading_global)

        self.goal_msg.pose.orientation.x = q[0]
        self.goal_msg.pose.orientation.y = q[1]
        self.goal_msg.pose.orientation.z = q[2]
        self.goal_msg.pose.orientation.w = q[3]

        self.goal_pub.publish(self.goal_msg)    

        self.run_en = False

        if verbose == True:
            print(f'New waypoint x={x} y={y}, heading={heading + heading_global} -- heading{heading} + global_heading{heading_global}')    


    def get_target(self, m1, m2, c1, c2):
        """ 
        Get Target

        Generates a point with robot pose as reference.

        Args:
            m1: Angular coefficient of the first corn line
            m2: Angular coefficient of the second corn line
            c1: Linear coefficient of the first corn line
            c2: Linear coefficient of the second corn line
        Returns:
            x: x-coordinate of the generated point
            y: y-coordinate of the generated point
        """

        # Convert coefficient args to a reference line to follow up
        m, c = self._convert_origin(m1, m2, c1, c2)

        # Get x and y resolution [m/px]^2
        x_regu = (self.img_width / self.row_width)
        y_regu = (self.img_height / self.row_height)
        
        # Solve equation using Euclidean distance and line equation
        x1, x2 = self._solve_quadratic(
                    1 + m**2 * (x_regu**2 / y_regu**2),
                    2 * m * c * (x_regu / y_regu**2),
                    c**2 / y_regu**2 - self.D**2)
        
        # Get corresponding y and return the possible value
        y1 = x1 * m * x_regu + c
        y2 = x2 * m * x_regu + c

        # print(f'Sol 1 x={x1} and y={y1} using {x1 * x_regu}')
        # print(f'Sol 2 x={x2} and y={y2} using {x2 * x_regu}')

        # x = np.linspace(-self.img_width/2, self.img_width/2, 100)
        # y = m * x + c

        # # Plot lines
        # plt.plot(x1 * x_regu, y1, 'ro') 
        # plt.plot(x2 * x_regu, y2, 'ro') 
        # plt.plot(x, y, label='m')
        
        # # Add labels and legend
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.legend()

        # plt.xlim(-self.img_width/2, self.img_width/2) 
        # plt.ylim(0, self.img_height)  
    
        
        # # Show plot
        # plt.grid(True)
        # plt.show()

        if (y1 >= 0 and y1 <= self.img_height):
            return x1, y1 / y_regu
        elif (y2 >= 0 and y2 <= self.img_height):
            return x2, y2 / y_regu
        else:
            rospy.logerr("Waypoint doesn't match with crop lines")
            return None


    def line_callback(self, msg):
        self.line_msg = msg
        self.run_en = True

    def odom_callback(self, msg):
        self.odom_msg = msg

    def _convert_origin(self, m1, m2, c1, c2):
        print(f'Receive {m1}, {m2}, {c1}, {c2}')

        m = -(m1 + m2) / 2
        c = -(c1 + c2) / 2 

        aux = -c / self.img_height
        c += aux*self.img_width

        x = np.linspace(-self.img_width/2, self.img_width/2, 100)
        y = m * x + c
        y1 = - m1 * x - c1 + aux*self.img_width
        y2 = - m2 * x - c2 + aux*self.img_width

        # # Plot lines
        # plt.plot(x, y1, label='m1')
        # plt.plot(x, y2, label='m2')
        # plt.plot(x, y, label='m')
        
        # # Add labels and legend
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.legend()

        # plt.xlim(-self.img_width/2, self.img_width/2) 
        # plt.ylim(0, self.img_height)  
    
        
        # # Show plot
        # plt.grid(True)
        # plt.show()

        return m, c

    
    def _solve_quadratic(self, a, b, c):
        discriminant = (b**2) - (4*a*c)

        sol1 = (-b - math.sqrt(discriminant)) / (2 * a)
        sol2 = (-b + math.sqrt(discriminant)) / (2 * a)

        return sol1, sol2

