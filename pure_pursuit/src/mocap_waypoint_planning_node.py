#!/usr/bin/env python
import rospy
import roscpp
import numpy as np
import math
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from geometry_msgs.msg import PoseArray, Point, Pose
import tf
import sys
import time
import threading

class MocapWaypointPlanningNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # start patrolling
        self.start = True
        # waypoint position
        self.waypoint_index = 0  
        self.X = [3.9, 3.9,    4, 4.31, 4.53, 4.65, 4.75, 5.05, 5.32, 5.41]
        self.Y = [2.44, 2.18, 1.95, 1.79, 1.75, 1.81, 1.91, 2.02, 1.89, 1.72]

        # vehicle point pair
        self.vehicle_yaw_pre = 0
        self.vehicle_front_pose = Pose()
        #!!!!!!!!self.vehicle_back_point = Point()
        # the previous vehicle point of last moment
        #self.pre_vehicle_point = Point()

        self.kd = 0.02
        self.kp = 0.05

        self.switch = True

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        self.pub_car_twist = rospy.Publisher("/cmd_vel",Twist,queue_size=1) ########from lane_controller
        # Subscription
        self.sub_vehicle_pose_pair = rospy.Subscriber("/lily/get_model_pose/vehicle_pose_pair", PoseArray, self.cbPoseArray, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.onShutdown)

        # timer
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def cbPoseArray(self, point_array_msg):
        if not(self.start):
            return
        # assign vehicle point pair 
        #!!!!!!self.vehicle_front_point = point_array_msg.poses[0].position
        self.vehicle_front_pose.position = point_array_msg.poses[0].position
        self.vehicle_front_pose.orientation = point_array_msg.poses[0].orientation
        #print("**************************")
        #print point_array_msg.poses
        #print("**************************")
        #!!!!!!self.vehicle_back_point = point_array_msg.poses[1].position
        
        # set target waypoint position
        target_point = self.set_target_point(self.waypoint_index)

        # calculate yaw angle from vehicle to target waypoint
        target_yaw = self.get_yaw_two_point(self.vehicle_front_pose, target_point)#target_yaw = self.get_yaw_two_point(self.vehicle_back_point, target_point)

        # calculate yaw angle from vehicle to previous vehicle
        vehicle_yaw = self.get_yaw_front_point(self.vehicle_front_pose)#vehicle_yaw = self.get_yaw_two_point(self.vehicle_back_point, self.vehicle_front_point)

        dist = self.get_dist_two_point(self.vehicle_front_pose, target_point)#dist = self.get_dist_two_point(self.vehicle_back_point, target_point)

        #self.set_pre_vehicle_point(point_msg)
        print "yaw from vehicle to waypoint: ", target_yaw
        print "yaw from previous vehicle to vehicle: ", vehicle_yaw
        print "distance between vehicle and waypoint", dist

        ess = vehicle_yaw - target_yaw
        diff = vehicle_yaw - self.vehicle_yaw_pre
        u = self.kp * ess + self.kd * diff
        print 'omega pd: ', -u
        if( u < -7):
           u = -7
        if( u > 7):
            u = 7
        print 'mega pd com: ', -u

   #     if(u*u < 0.00005):
    #        self.publish_car_cmd(0.2, 0 , 0.1)
     #       self.publish_car_cmd(1, -u , 0.1)

        self.publish_car_cmd(0.2, -u , 0.1)
        if(dist <= 0.2):
            if(self.waypoint_index < 9): #10 points
                print "goal!!!"
                #self.publish_car_cmd(0, -u, 0.1)
                self.waypoint_index +=1
            else:
                self.publish_car_cmd(0, 0, 2)
                self.start = False

    def set_target_point(self, order):
        # set a target_point
        print "the ",(order+1)," point"
        target_point = Point()
        target_point.x = self.X[order]
        target_point.y = self.Y[order]
        target_point.z = 0
        return target_point

    def get_yaw_two_point(self, source_pose, target_point):
        # calculate arctan(in rad) of two point
        #print("+++++++++++++++++++++++++++++++")
        #print source_pose
        #print("+++++++++++++++++++++++++++++++")
        dx = target_point.x - source_pose.position.x
        dy = target_point.y - source_pose.position.y
        yaw = math.atan(dy/dx) * 180/math.pi
        #print 'original yaw', yaw
        #print 'dx', dx
        #print 'dy', dy
        # rad compensation
        if self.switch == False:
            if( dx > 0 and dy > 0):
                yaw = yaw
            elif( dx < 0):
                yaw = yaw + 180
            elif( dx > 0 and dy < 0):
                yaw = yaw + 360
        else:
            if( dx > 0):
                yaw = yaw
            elif( dx < 0 and dy > 0):
                yaw = yaw + 180
            elif( dx < 0 and dy < 0):
                yaw = yaw - 180
            elif( dx == 0 and dy == 0):
                yaw = 0
            elif( dx == 0 and dy > 0):
                yaw = 90
            elif( dx == 0 and dy < 0):
                yaw = -90 
        #print("+++++++++++++++++++++++++++++++")
        #print yaw
        #print("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ")
        return yaw
    def get_yaw_front_point(self, source_pose):
        # calculate arctan(in rad) of two point
        quaternion = (source_pose.orientation.x, source_pose.orientation.y, source_pose.orientation.z, source_pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        yaw = yaw * 180 /math.pi

        #print yaw
        #print("+++++++++++++++++++++++++++++++")
        return yaw

    def get_dist_two_point(self, source_pose, target_point):
        dx = target_point.x - source_pose.position.x
        dy = target_point.y - source_pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)
        return dist 
    def set_pre_vehicle_point(self, point):
        self.pre_vehicle_point.x = point.x
        self.pre_vehicle_point.y = point.y
        self.pre_vehicle_point.z = point.z

    def publish_car_cmd(self, v, omega, duration):
        # publish car command
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        self.pub_car_cmd.publish(car_cmd_msg)
        
        car_twist_msg = Twist() #########from lane_controller_node
        car_twist_msg.linear.x = car_cmd_msg.v*1.5 #########from lane_controller_node
        car_twist_msg.angular.z = car_cmd_msg.omega*0.5 #########from lane_controller_node
        self.pub_car_twist.publish(car_twist_msg) #########from lane_controller_node
        
        rospy.sleep(duration)
        
        # stop 1s
        car_cmd_msg.v = 0
        car_cmd_msg.omega = 0
        self.pub_car_cmd.publish(car_cmd_msg)

        car_twist_msg = Twist() #########from lane_controller_node
        car_twist_msg.linear.x = car_cmd_msg.v*1.5 #########from lane_controller_node
        car_twist_msg.angular.z = car_cmd_msg.omega*0.5 #########from lane_controller_node
        self.pub_car_twist.publish(car_twist_msg) #########from lane_controller_node
        
        rospy.sleep(0)      

    def onShutdown(self):
        # Send stop command
        self.publish_car_cmd(0,0,2)
        rospy.loginfo("[%s] Shutdown" %self.node_name)

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

if __name__ == '__main__':
    rospy.init_node('mocap_waypoint_planning_node',anonymous=False)
    print("==========before mocap_waypoint_planning_node==============")
    mocap_waypoint_planning_node = MocapWaypointPlanningNode()
    print("==========after mocap_waypoint_planning_node==============")
    rospy.on_shutdown(mocap_waypoint_planning_node.onShutdown)

    rospy.spin()