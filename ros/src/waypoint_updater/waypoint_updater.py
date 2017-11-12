#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import numpy as np
import tf

from rotation import rotate
import copy

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
TL_DECAY_RATE = 0.125 # The deceleration in waypoints when the traffic light is red


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.loginfo("### 3X Speed")

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber("/traffic_waypoint", Int32, self.traffic_cb)
        rospy.Subscriber("/obstacle_waypoint", Lane, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None # delete?
        self.closest_wpt_id = 0
        self.closest_wpt = None
        self.base_wpts = None
        self.num_base_wpts = 0
        self.traffic_wpt_id = -1

        rospy.loginfo("### Waypoint updator inited")

        rate = rospy.Rate(8)

        while not rospy.is_shutdown():
            if self.current_pose and self.base_wpts:
                self.update()
            rate.sleep()

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg

    def update(self):
        closest_dist = float("inf")
        self.closest_wpt = self.base_wpts.waypoints[0]
        d1 = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(len(self.base_wpts.waypoints)):
            wpt = self.base_wpts.waypoints[i]
            dist = d1(self.current_pose.pose.position, wpt.pose.pose.position)
            if dist < closest_dist:
                closest_dist = dist
                self.closest_wpt_id = i
                self.closest_wpt = wpt

        # know whether the closest one is on the front
        self.closest_wpt_id = self.get_next_waypoint(self.current_pose.pose, self.closest_wpt_id)
        self.closest_wpt_id %= self.num_base_wpts
        self.closest_wpt = self.base_wpts.waypoints[self.closest_wpt_id]

        new_wpts = Lane()
        #raw_new_wpts = []
        end_wpt_id = (self.closest_wpt_id+LOOKAHEAD_WPS)%self.num_base_wpts
        if end_wpt_id < self.closest_wpt_id:
            new_wpts.waypoints = self.base_wpts.waypoints[self.closest_wpt_id:] + self.base_wpts.waypoints[:end_wpt_id]
            #new_wpts.waypoints = copy.deepcopy(self.base_wpts.waypoints[self.closest_wpt_id:] + self.base_wpts.waypoints[:end_wpt_id])
        else:
            new_wpts.waypoints = self.base_wpts.waypoints[self.closest_wpt_id:end_wpt_id]
            #new_wpts.waypoints = copy.deepcopy(self.base_wpts.waypoints[self.closest_wpt_id:end_wpt_id])
        # copy the wpts
        #for wpt in raw_new_wpts:
        #    new_wpt = copy.deepcopy(wpt)
        #    new_wpts.waypoints.append(new_wpt)

        # debug the copy
        #ref_v = self.closest_wpt.twist.twist.linear.x
        #ref_yaw = self.closest_wpt.twist.twist.angular.z
        #rospy.loginfo("Nearest waypoint: %d, RefV: %f, RefYaw: %f"%(self.closest_wpt_id, ref_v, ref_yaw))

        # debug
        #self.final_waypoints_pub.publish(new_wpts)
        #return
        

        for i in range(len(new_wpts.waypoints)):
            base_wpt = new_wpts.waypoints[i]
            wpt = Waypoint()
            wpt.pose.pose.position = base_wpt.pose.pose.position
            wpt.pose.pose.orientation = base_wpt.pose.pose.orientation
            wpt.twist.twist.linear.x = base_wpt.twist.twist.linear.x
            wpt.twist.twist.angular.z = base_wpt.twist.twist.angular.z
            new_wpts.waypoints[i] = wpt

        # debug
        #new_wpts.waypoints = raw_new_wpts
        #debug
        #self.final_waypoints_pub.publish(new_wpts)
        #return


        if self.traffic_wpt_id and self.traffic_wpt_id != -1: # have traffic light
            if self.traffic_wpt_id > self.closest_wpt_id:
                next_decay_wpt_id = self.traffic_wpt_id - self.closest_wpt_id
            else:
                next_decay_wpt_id = (self.num_base_wpts - self.closest_wpt_id) + self.traffic_wpt_id
            num_decay_wpts = 0
            decay_ongoing = True

            # stop in next 20 waypoints
            for i in range(20):
                stop_wpt_id = next_decay_wpt_id + i
                if stop_wpt_id < len(new_wpts.waypoints):
                    self.set_waypoint_velocity(new_wpts.waypoints, stop_wpt_id, 0)

            # and decay in previous waypoints
            while decay_ongoing and next_decay_wpt_id >= 0 and next_decay_wpt_id < len(new_wpts.waypoints):
                new_velocity = num_decay_wpts * TL_DECAY_RATE
                if self.get_waypoint_velocity(new_wpts.waypoints[next_decay_wpt_id]) < new_velocity:
                    decay_ongoing = False # decay finished
                else:
                    self.set_waypoint_velocity(new_wpts.waypoints, next_decay_wpt_id, new_velocity)
                    next_decay_wpt_id -= 1
                    num_decay_wpts += 1


        self.final_waypoints_pub.publish(new_wpts)
        rospy.loginfo("waypoints published " + str(self.closest_wpt_id) + " " + str(end_wpt_id))
        # now_time = rospy.get_time()
        # delta_t = now_time - self.last_timestamp
        # self.last_timestamp = now_time
        # rospy.loginfo("Time used: %.3f"%delta_t)

    def get_next_waypoint(self, pose, index):
	"""Identifies the first waypoint that is currently ahead of the car
        Args:
            index(int): index of the closest waypoint in self.waypoints

        Returns:
            int: index of the first waypoint currently ahead of the car

	"""
        # At first I try to use rotation matrix, then I found that
        # use tf.euler_from_quaternion is better
        # btw, this tf is not tensorflow
        next_index = index 
        p1 = pose.position
        p2 = self.base_wpts.waypoints[index].pose.pose.position
        heading = math.atan2( (p2.y-p1.y),(p2.x-p1.x) );
        quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        angle = abs(yaw-heading)

        if angle > math.pi/4:
            next_index += 1
        
        return next_index

        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_wpts = waypoints
        self.num_base_wpts = len(self.base_wpts.waypoints)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_wpt_id = msg.data


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')