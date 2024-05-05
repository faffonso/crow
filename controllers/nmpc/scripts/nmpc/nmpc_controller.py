#!/usr/bin/env python3

import casadi as ca
import numpy as np
import os

import rospy
import time

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ilqr.msg import ControllerTime

class NMPC:
    def __init__(self, dt=0.1, N=10, Q_x=1.0, Q_y=1.0, Q_theta=1.0, R_v=1.0, R_omega=1.0, v_max=1.0, omega_max=1.0):
        self.dt = dt
        self.N  = N

        self.Q = np.diag([Q_x, Q_y, Q_theta])
        self.R = np.diag([R_v, R_omega])

        # Publishers and Subscribers
        self.odom_topic = rospy.get_param("odom/topic")
        self.odom_frame_id = rospy.get_param("odom/frame_id")

        self.odom_subscriber = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        self.goal_subscriber = rospy.Subscriber("/terrasentia/goal", PoseStamped, self.goal_callback)

        self.path_publisher     = rospy.Publisher("/terrasentia/path", Path, queue_size=1)
        self.cmd_vel_publisher  = rospy.Publisher("/terrasentia/cmd_vel", TwistStamped, queue_size=10)
        self.time_cost_publisher  = rospy.Publisher("/terrasentia/ilqr_time", ControllerTime, queue_size=10)

        self.odom    = Odometry()
        self.goal    = PoseStamped()
        self.cmd_vel = TwistStamped()
        self.nmpc_time = ControllerTime()

        # Optimization struct
        os.environ['IPOPT_NUM_THREADS'] = '5'
        opti = ca.Opti()

        # State space
        opt_states = opti.variable(N+1, 3)
        self.opt_states = opt_states

        x       = opt_states[:, 0]
        y       = opt_states[:, 1]
        theta   = opt_states[:, 2]

        opt_controls = opti.variable(N, 2)
        self.opt_controls = opt_controls

        v       = opt_controls[:, 0]
        omega   = opt_controls[:, 1]

        # Parameters
        self.opt_x0 = opti.parameter(3)
        self.opt_xref = opti.parameter(3)

        # Init condition
        x0 = self.opt_x0.T
        opti.subject_to(opt_states[0, :] == x0)
    
        # Subject to dynamic system
        for k in range(N):
            xs = opt_states[k, :]
            us = opt_controls[k, :]

            x_next = xs + self.f(xs, us).T * dt
            opti.subject_to(opt_states[k+1, :] == x_next)

        # Cost functions
        obj = 0
        xref = self.opt_xref

        for k in range(N):
            xs = opt_states[k, :].T
            us = opt_controls[k, :].T

            obj += self.l((xs - xref), us)

        opti.minimize(obj)

        # Boundrary and control conditions
        self.v_max       = v_max
        self.omega_max   = omega_max

        opti.subject_to(opti.bounded(0, v, v_max))
        opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

        # Solver settings
        opts_setting = {
            'ipopt.max_iter':50, 
            'ipopt.print_level':0, 
            'print_time':0, 
            'ipopt.acceptable_tol':1e-10, 
            'ipopt.acceptable_obj_change_tol':1e-8}

        opti.solver('ipopt', opts_setting)

        self.opti = opti
        print(f'Optimization problem {self.opti}')

    def run(self):
        start_time = time.time()

        x0 = self.state_from_pose(self.odom.pose.pose)
        xref = self.state_from_pose(self.goal.pose)

        u0 = np.zeros((self.N,2))
        xs = np.tile(x0, (self.N+1, 1))

        x, u = self.fit(xs, xref, x0, u0)
        print(u)

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.odom_frame_id
        for xi in x:
            pose = self.pose_from_state(xi)
            path_msg.poses.append(pose)

        self.cmd_vel.twist.linear.x = u[0][0]
        self.cmd_vel.twist.angular.z = u[0][1]

        self.path_publisher.publish(path_msg)
        # self.cmd_vel_publisher.publish(self.cmd_vel)

        end_time = time.time()
        elapsed_time = end_time - start_time
        self.nmpc_time = elapsed_time
        self.time_cost_publisher.publish(self.nmpc_time)
        print(f'NMPC Run | State {x0} | Reference {xref} | Action control {u[0]} | Time {elapsed_time:.3f} seconds')
        
    def state_from_pose(self, pose):
        position      = pose.position
        orientation_q = pose.orientation
        orientation = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        x_0 = position.x
        y_0 = position.y
        _, _, theta_0 = euler_from_quaternion(orientation)

        return np.array([x_0, y_0, theta_0])

    def pose_from_state(self, x):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.odom_frame_id
   
        pose_msg.pose.position.x = x[0]
        pose_msg.pose.position.y = x[1] 

        orientation = quaternion_from_euler(0.0, 0.0, x[2])
        pose_msg.pose.orientation.x = orientation[0]
        pose_msg.pose.orientation.y = orientation[1]
        pose_msg.pose.orientation.z = orientation[2]
        pose_msg.pose.orientation.w = orientation[3]

        return pose_msg

    def odom_callback(self, msg):
        self.odom = msg

    def goal_callback(self, msg):
        self.goal = msg

    def fit(self, xs, xref, x0, u0):
        self.opti.set_value(self.opt_xref, xref)
        self.opti.set_value(self.opt_x0, x0)

        self.opti.set_initial(self.opt_controls, u0)
        self.opti.set_initial(self.opt_states, xs)

        sol = self.opti.solve()

        u = sol.value(self.opt_controls)
        x = sol.value(self.opt_states)
        return x, u
    
    def f(self, x, u):
        x1 = u[0]*ca.cos(x[2])
        x2 = u[0]*ca.sin(x[2])
        x3 = u[1]

        return ca.vertcat(x1, x2, x3)
    
    def l(self, x, u = 0):
        running_cost = x.T  @ self.Q @ x + u.T @ self.R @ u
        return running_cost 