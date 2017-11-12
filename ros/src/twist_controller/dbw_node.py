#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
        self.pid = PID(-0.9, -0.001, -0.07, decel_limit, accel_limit)
        self.lowpass_filter = LowPassFilter(1.0, 2.0) # the less the value, the more smooth
        self.yaw_lowpass = LowPassFilter(1.0, 1.0)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0, max_lat_accel, max_steer_angle)
        self.brake_factor = vehicle_mass * wheel_radius
        self.brake_deadband = brake_deadband

        # TODO: Subscribe to all the topics you need to
        # init the vars
        self.current_velocity = None
        self.twist_cmd = None
        self.dbw_enabled = False # For debugging, use true
        self.last_timestamp = rospy.get_time()
        # subscribe
        rospy.Subscriber("/current_velocity", TwistStamped, self.current_velocity_cb)
        rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber("/dbw_enabled", Bool, self.dbw_enabled_cb)


        rospy.loginfo("### Controller initialized Yeah")

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)

            # init the vars
            ref_v = 0.
            cur_v = 0.
            err_v = 0.
            ref_yaw = 0.
            dt = 0.01
            thro_cmd = 0.
            brake_cmd = 0.
            steer_cmd = 0.

            if self.twist_cmd and self.current_velocity:
                ref_v = self.twist_cmd.twist.linear.x
                ref_yaw = self.twist_cmd.twist.angular.z
                cur_v = self.current_velocity.twist.linear.x
                err_v = cur_v - ref_v
                dt = rospy.get_time() - self.last_timestamp
                self.last_timestamp = rospy.get_time()
                rospy.loginfo("CurV: %f, RefV: %f"%(cur_v, ref_v))
                rospy.loginfo("Change the errors: "+str(err_v))
            if self.dbw_enabled:
                acc = self.pid.step(err_v, dt) # get the acc
                if acc >= 0:
                    thro_cmd = self.lowpass_filter.filt(acc)
                    brake_cmd = 0.
                else:
                    thro_cmd = 0.
                    brake_cmd = self.brake_factor * (-acc) + self.brake_deadband

                steer_cmd = self.yaw_controller.get_steering(ref_v, ref_yaw, cur_v)
                steer_cmd = self.yaw_lowpass.filt(steer_cmd)
                self.publish(thro_cmd, brake_cmd, steer_cmd)
                rospy.loginfo("Publish:"+str(thro_cmd)+"  "+str(steer_cmd))
                rospy.loginfo("Time used: %.3f"%dt)
            else:
                self.pid.reset()
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg
        rospy.loginfo("Get twist cmd:"+str(self.twist_cmd.twist))

    def current_velocity_cb(self, msg):
        self.current_velocity = msg

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg


if __name__ == '__main__':
    DBWNode()