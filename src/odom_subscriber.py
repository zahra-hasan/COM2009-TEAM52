#!/usr/bin/env python3
# A simple ROS subscriber node in Python


import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class Subscriber():

    def callback(self, odom_data):

        position_x=odom_data.pose.pose.position.x
        position_y=odom_data.pose.pose.position.y

        orientation_x= odom_data.pose.pose.position.x
        orientation_y=odom_data.pose.pose.position.y
        orientation_z=odom_data.pose.pose.position.z
        orientation_w=odom_data.pose.pose.position.z

        (roll,pitch,yaw)=euler_from_quaternion([orientation_x,orientation_y,orientation_z,orientation_w,'sxyz'])

        if self.counter >10:
            self.counter=0
            print("x = {:.2f} [m], y = {:.2f} [m], yaw = {:.1f} [degrees].".format(position_x,position_y,yaw))
        else:
                self.counter +=1

    def __init__(self):

        rospy.init_node('odom_subscriber_node', anonymous=True)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.loginfo(f"odom subscriber is active... ")
        self.counter=0

        

    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()
