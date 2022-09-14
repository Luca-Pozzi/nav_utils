#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math
import copy
import numpy as np

import rospy
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry


class MoveRobot(object):
    def __init__(self, hz = 10.0):
        
        self._pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)
        self._hz = hz #[Hz]
        self.rate = rospy.Rate(self._hz)     # hz should be retrieved from the parameter server.
                                        # A too low value would cause the robot to move in fits and starts.
        # The following attributes will determine the robot movement speed. Max velocity and acceleration are retrieved from the parameter server, i.e. the robot will move as fast as possible.
        self._angular_vel = rospy.get_param('/mobile_base_controller/angular/z/max_velocity', 
                                            2.0
                                            ) #[rad/s]
        self._linear_vel  = rospy.get_param('/mobile_base_controller/linear/x/max_velocity', 
                                            1.0
                                            ) #[m/s]        
        self._angular_acc = rospy.get_param('/mobile_base_controller/angular/z/max_acceleration', 
                                            2.0
                                            ) #[rad/s^2]
        self._linear_acc  = rospy.get_param('/mobile_base_controller/linear/x/max_acceleration', 
                                            1.0
                                            ) #[m/s] 

        #self._odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self._update_odom)
        # self.event_sub = ...

    def ask_translation(self):  # Placeholder for the event receival callback
        self.translation_target = float(raw_input('Enter the linear movement [m]: '))
        self.forward = self.translation_target >= 0


    def ask_rotation(self):  # Placeholder for the event receival callback
        self.rotation_target = float(raw_input('Enter the angle [deg]: ')) * math.pi / 180.0
        self.counterclockwise = self.rotation_target >= 0

    def base_translation(self, translation = True):
        vel_profile = self._trapezoidal_vel_profile(v_max = self._linear_vel, 
                                                    a_max = self._linear_acc, 
                                                    s_tot = abs(self.translation_target) 
                                                    )
        if not self.forward:
            vel_profile *= -1
        print(vel_profile)
        for v in vel_profile:
            self._publish(linear = v)
            self.rate.sleep()   # keep the loop at the desired frequency


    def base_rotation(self):
        vel_profile = self._trapezoidal_vel_profile(v_max = self._angular_vel, 
                                                    a_max = self._angular_acc, 
                                                    s_tot = abs(self.rotation_target)
                                                    )
        if not self.counterclockwise:
            vel_profile *= -1
        for v in vel_profile:
            self._publish(angular = v)
            self.rate.sleep()   # keep the loop at the desired frequency

    def _trapezoidal_vel_profile(self, v_max, a_max, s_tot):
        """ Generate a trapezoidal velocity profile to control the base movement in feedforward.
        The initial velocity is assumed to be 0.

        Args:
            v_max (float): The maximum velocity. Measurement unit depends on input.
            a_max (float): The maximum acceleration. Measurement unit depends on input.
            s_tot (float): The overall displacement. Measurement unit depends on input.
        """
        # The duration of the acceleration and deceleration phase (and also the time instant at which the peak velocity is reached, assuming 0 at start).
        t_acc = min([math.sqrt(s_tot / a_max),  # if triangular
                     v_max / a_max              # if trapezoidal
                    ])
        print('Acceleration lasts:\t\t' + str(t_acc))
        # The duration of the constant speed phase. 
        t_vmax = max([0.0,                          # if triangular (i.e. no constant speed phase)
                      s_tot / v_max - v_max / a_max # if trapezoidal
                      ])
        print('Max velocity is kept for:\t' + str(t_vmax))
        v_peak = t_acc * a_max  # maximum velocity reached during the movement
        print('The maximum velocity is:\t' + str(v_peak))
        # Velocity values to send to the base.
        v_acc = np.linspace(0.0,
                            v_peak,
                            num = int(self._hz * t_acc),
                            endpoint = False
                            )
        print(v_acc)
        v_const = [v_max] * (int(self._hz * t_vmax) - 1)
        print(v_const)
        v_dec = np.linspace(v_peak,
                            0.0, 
                            num = int(self._hz * t_acc),
                            endpoint = True
                            )
        print(v_dec)
        return np.concatenate((v_acc, v_const, v_dec))


    def _publish(self, linear = 0.0, angular = 0.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._pub_cmd.publish(twist)

    '''
    def _update_odom(self, odom_msg):
        """ Callback of the odometry subscriber. Updates the attributes with the current position and orientation of the robot.

        Args:
            odom_msg (nav_msgs/Odometry): the robot odometry, as estimated by its onboard sensors.
        """
        self.pos = odom_msg.pose.pose.position
        self.ori = odom_msg.pose.pose.orientation
    '''

    '''
    def _quat_to_rotation_around_z(self, quat):
        # ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        qz = sin(rot/2)
    '''

if __name__ == '__main__':
    rospy.init_node('cmd_vel_from_input')
    mr = MoveRobot()
    try:
        while not rospy.is_shutdown():
            mr.ask_rotation()
            mr.base_rotation()
            #mr.ask_translation()
            #mr.base_translation()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down.')
