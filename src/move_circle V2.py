#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math



class Circle():
    
    def __init__(self):

        rospy.init_node('move_circle', anonymous=True)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
       
        self.rate = rospy.Rate(10) # hz
            
        self.vel_cmd = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 
        
        rospy.loginfo(f"The 'move_circle' node is active...")

    def shutdownhook(self):
        self.vel_cmd.linear.x=0.0
        self.vel_cmd.angular.z=0.0
        print("Stopping the robot")
        self.pub.publish(self.vel_cmd)
        self.ctrl_c = True

    def main_loop(self):
        startTime= rospy.get_rostime()
        
        path_rad=0.5
        lin_vel=0.12
        duration=rospy.get_rostime()-startTime 
        while duration.secs<2*math.pi*abs(path_rad)/abs(lin_vel):
             
            duration=rospy.get_rostime()-startTime        

            self.vel_cmd.linear.x=lin_vel
            self.vel_cmd.angular.z=lin_vel/path_rad
            

            self.pub.publish(self.vel_cmd)
        

        while duration.secs>2*math.pi*abs(path_rad)/abs(lin_vel) and duration.secs<=2*2*math.pi*abs(path_rad)/abs(lin_vel):

            path_rad=-0.5
             
            duration=rospy.get_rostime()-startTime        

            self.vel_cmd.linear.x=lin_vel
            self.vel_cmd.angular.z=lin_vel/path_rad
            

            self.pub.publish(self.vel_cmd)
            self.rate.sleep()
        


if __name__ == '__main__':
    vel_ctrl = Circle()
    try:
        vel_ctrl.main_loop()
    except rospy.ROSInterruptException:
        pass
