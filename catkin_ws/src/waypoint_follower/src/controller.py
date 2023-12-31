#!/usr/bin/env python
import numpy as np
import os
import pandas as pd
import math

import roslib
import rospy
import rospkg

from std_msgs.msg import Int16
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

import tf
from tf.transformations import euler_from_quaternion

import pdb

# Calculate distance
def calc_dist(tx, ty, ix, iy):
    return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

# Normalize angle [-pi, +pi]
def normalize_angle(angle):
    if angle > math.pi:
        norm_angle = angle - 2*math.pi
    elif angle < -math.pi:
        norm_angle = angle + 2*math.pi
    else:
        norm_angle = angle
    return norm_angle

# Global2Local
def global2local(ego_x, ego_y, ego_yaw, x_list, y_list):
    """
    TODO
    Transform from global to local coordinate w.r.t. ego vehicle's current pose (x, y, yaw).
        - x_list, y_list               : global coordinate trajectory.
        - ego_x, ego_y, ego_yaw        : ego vehicle's current pose.
        - output_x_list, output_y_list : transformed local coordinate trajectory.
    """
    new_x = x_list - ego_x
    new_y = y_list - ego_y

    output_x_list = math.cos(ego_yaw) * new_x + math.sin(ego_yaw) * new_y
    output_y_list = -math.sin(ego_yaw) * new_x + math.cos(ego_yaw) * new_y

    return output_x_list, output_y_list

# Find nearest point
def find_nearest_point(ego_x, ego_y, x_list, y_list):
    """
    TODO
    Find the nearest distance(near_dist) and its index(near_ind) w.r.t. current position (ego_x,y) and given trajectory (x_list,y_list).
        - Use 'calc_dist' function to calculate distance.
        - Use np.argmin for finding the index whose value is minimum.
    """

    dist_mat = np.zeros(x_list.shape)

    for i in range(len(x_list)):
        dist_mat[i] = calc_dist(x_list[i],y_list[i],ego_x,ego_y)
    
    near_ind = np.argmin(dist_mat)
    near_dist = dist_mat[near_ind]

    return near_dist, near_ind

# Calculate Error
def calc_error(ego_x, ego_y, ego_yaw, x_list, y_list, wpt_look_ahead=0):
    """
    TODO
    Local coordinate approach
    (!It doesn't matter if you implement the global coordinates approach.)
    (!Just return the cross-track and heading errors.)
    1. Transform from global to local coordinate trajectory.
    2. Find the nearest waypoint.
    3. Set lookahead waypoint.
        - (hint) use the index of the nearest waypoint (near_ind) from 'find_nearest_point'.
        - (hint) consider that the next index of the terminal waypoint is 0, which is not terminal_index + 1.
    4. Calculate errors
        - error_yaw (yaw error)
            : (hint) use math.atan2 for calculating angle btw lookahead and next waypoints.
            : (hint) consider that the next index of the terminal waypoint is 0, which is not terminal_index + 1.
        - error_y (crosstrack error)
            : (hint) y value of the lookahead point in local coordinate waypoint trajectory.
    """
    local_x_list, local_y_list = global2local(ego_x, ego_y, ego_yaw, x_list, y_list)

    # 2. Find the nearest waypoint
    _, near_ind = find_nearest_point(ego_x, ego_y, x_list, y_list)

    # 3. Set lookahead waypoint (index of waypoint trajectory)
    lookahead_wpt_ind = (near_ind + wpt_look_ahead) % len(x_list)
    
    # 4. Calculate errors
    next_wpt = (near_ind + wpt_look_ahead + 1) % len(x_list)
    delta_y = local_y_list[next_wpt] - local_y_list[lookahead_wpt_ind]
    delta_x = local_x_list[next_wpt] - local_x_list[lookahead_wpt_ind]
    error_yaw = math.atan2(delta_y,delta_x)
    error_yaw = normalize_angle(error_yaw) # Normalize angle to [-pi, +pi]
    error_y   = local_y_list[lookahead_wpt_ind]
    
    return error_y, error_yaw


class WaypointFollower():
    def __init__(self):
        # ROS init
        rospy.init_node('wpt_follwer')
        self.rate = rospy.Rate(100.0)

        # Params
        self.target_speed = 10/3.6
        self.MAX_STEER    = np.deg2rad(17.75)

        # vehicle state
        self.ego_x   = 0
        self.ego_y   = 0
        self.ego_yaw = 0
        self.ego_vx  = 0
        self.error_v = self.target_speed 
        self.v_i = 0
        self.prior_steer =0

        self.wpt_look_ahead = 0   # [index]

        # Pub/Sub
        self.pub_command = rospy.Publisher('control', AckermannDriveStamped, queue_size=5)
        self.sub_odom    = rospy.Subscriber('simulation/bodyOdom', Odometry, self.callback_odom)

    def callback_odom(self, msg):
        """
        Subscribe Odometry message
        ref: http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
        """
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_vx = msg.twist.twist.linear.x
        # get euler from quaternion
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        _, _, self.ego_yaw = euler_from_quaternion(q_list)

    # Controller
    def steer_control(self, error_y, error_yaw):
        """
        TODO
        Implement a steering controller (PID controller or Pure pursuit or Stanley method).
        You can use not only error_y, error_yaw, but also other input arguments for this controller if you want.
        """
        steer = 0.15 * error_yaw + math.atan2(1.7 * error_y, self.ego_vx)
        steer = 0.9*steer + 0.1*self.prior_steer # contatin prior steer
        self.prior_steer = steer
        steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)
        return steer

    def speed_control(self, error_v):
        """
        TODO
        Implement a speed controller (PID controller).
        You can use not only error_v, but also other input arguments for this controller if you want.
        """
        
        v_p = error_v
        self.v_i += error_v
        v_d = error_v - self.error_v
        self.error_v = error_v

        throttle = 0.52 * v_p + 0 * self.v_i + 0.005 + v_d

        return throttle

    def publish_command(self, steer, accel):
        """
        Publish command as AckermannDriveStamped
        ref: http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html
        """
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steer / np.deg2rad(17.75)
        msg.drive.acceleration = accel
        self.pub_command.publish(msg)

def main():
    # Load Waypoint
    rospack = rospkg.RosPack()
    WPT_CSV_PATH = rospack.get_path('waypoint_follower') + "/wpt_data/wpt_data_dense.csv"
    csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
    wpts_x = csv_data.values[:,0]
    wpts_y = csv_data.values[:,1]
    
    # List to evaluate performance
    error_y_data = []
    error_v_data = []

    print("loaded wpt :", wpts_x.shape, wpts_y.shape)

    # Define controller
    wpt_control = WaypointFollower()


    while not rospy.is_shutdown():
        # Get current state
        
        ego_x = wpt_control.ego_x
        ego_y = wpt_control.ego_y
        ego_yaw = wpt_control.ego_yaw
        ego_vx = wpt_control.ego_vx

        # Lateral error calculation (cross-track error, yaw error)
        error_y, error_yaw = calc_error(ego_x, ego_y, ego_yaw, wpts_x, wpts_y, wpt_look_ahead=wpt_control.wpt_look_ahead)

        # Longitudinal error calculation (speed error)
        error_v = wpt_control.target_speed - ego_vx

        # Log errors
        error_y_log, _ = calc_error(ego_x, ego_y, ego_yaw, wpts_x, wpts_y, wpt_look_ahead=40)
        error_y_data.append(abs(error_y_log))
        error_v_data.append(abs(error_v))

        # Control
        steer_cmd = wpt_control.steer_control(error_y, error_yaw)
        throttle_cmd = wpt_control.speed_control(error_v)

        # Publish command
        wpt_control.publish_command(steer_cmd, throttle_cmd)

        rospy.loginfo("Commands: (steer=%.3f, accel=%.3f). Errors: (CrossTrack=%.3f, Yaw=%.3f, Speed=%.3f). Ave. Errors ((CrossTrack=%.3f, Speed=%.3f)) ." %(steer_cmd, throttle_cmd, error_y, error_yaw, error_v, np.mean(error_y_data), np.mean(error_v_data)))
        wpt_control.rate.sleep()


if __name__ == '__main__':
    main()

