#!/usr/bin/env python3

import rospy
from fleet_turtle.msg import Setpoint
from geometry_msgs.msg import Twist#, Vector3
from nav_msgs.msg import Odometry
# implement odometry msgs
import os
import math
#import numpy as np


class navigatorNode():
    def __init__(self):

        self.setpointX=0
        self.setpointY=0
        self.setpointReached=True

        self.currentPositionX=0
        self.currentPositionY=0
        self.currentOrientation=0
        self.velocityX=0
        self.velocityY=0
        self.lever=0.3

        self.robotStuck=False
        self.previousPositionX=0
        self.previousPositionY=0
        self.lastMotilePositionX=0
        self.lastMotilePositionY=0
        self.setpointBackup=(0,0)

        self.setpointTopic = rospy.get_param('~setpoint_topic', '/Setpoint')
        self.odometryTopic= rospy.get_param('~odometry_topic','/odom')

        # setpoint publisher
        self.setpointCommand = rospy.Publisher(self.setpointTopic,Setpoint,\
                                                queue_size=10)
                                                
        # odometry subscriber
        self.odometryListener=rospy.Subscriber(self.odometryTopic, Odometry,\
                                                self.odometryCallback, queue_size=5)
        
        #timer
        # self.collisionTimer=rospy.Timer(rospy.Duration(0.5),self.collisionTimerCallback)
        # self.navigationTimer=rospy.Timer(rospy.Duration(0.01),self.navigaztionCallback)


    def setpointCallback(self,data):
        self.setpointX=round(data.x_setpoint,4)
        self.setpointY=round(data.y_setpoint,4)
        self.setpointReached=False
        print("received a new goal position: x="+str(self.setpointX)+
                                            " ,y="+str(self.setpointY))

    
    def odometryCallback(self,data):
        orientation = data.pose.pose.orientation
        position = data.pose.pose.position
        siny_cosp = 2*(orientation.w*orientation.z+orientation.x*orientation.y)
        cosy_cosp = 1-2*(orientation.y**2 + orientation.z**2)
        self.currentOrientation = math.atan2(siny_cosp,cosy_cosp)
        self.currentPositionX = position.x + self.lever*math.cos(self.currentOrientation)
        self.currentPositionY = position.y + self.lever*math.sin(self.currentOrientation)


    '''
    def mapCallback(self,data):
    '''


    def regulatorTimerCallback(self,data):

        if not self.setpointReached:

            positionErrorX=self.setpointX-self.currentPositionX
            positionErrorY=self.setpointY-self.currentPositionY

            if self.controlMode=='PROPORTIONAL':

                self.velocityX=self.KX*positionErrorX
                self.velocityY=self.KY*positionErrorY

                self.control1=self.KControl1*(math.cos(self.currentOrientation)*self.velocityX+\
                    math.sin(self.currentOrientation)*self.velocityY)
                self.control2=self.KControl2*(-self.velocityX*(math.sin(self.currentOrientation/self.lever))+\
                    self.velocityY*(math.cos(self.currentOrientation/self.lever)))
            
                # minimum speed
                if abs(self.control1)<self.MIN_CONTROL_X:
                    self.control1=math.copysign(self.MIN_CONTROL_X,self.control1)
            
                self.velocityMessage.linear.x=self.control1
                self.velocityMessage.angular.z=self.control2
                self.velocityCommand.publish(self.velocityMessage)
            
                if abs(positionErrorX)<self.ERROR_THRESHOLD and\
                        abs(positionErrorY)<self.ERROR_THRESHOLD:
                    self.setpointReached=True
                    print('Setpoint Reached')
            
            else:
                self.goalDistance=math.sqrt(positionErrorX**2+positionErrorY**2)
                self.goalOrientation=math.atan2(positionErrorY,positionErrorX)
                # print("error "+str(self.positionErrorX)+str(self.positionErrorY))

                # rotation
                self.velocityMessage.angular.z=self.VEL_ANGULAR_MAX
                self.velocityCommand.publish(self.velocityMessage)
                rospy.sleep(self.goalOrientation/self.VEL_ANGULAR_MAX)
                # linear motion
                self.velocityMessage.angular.z=0
                self.velocityMessage.linear.x=self.VEL_LINEAR_MAX
                self.velocityCommand.publish(self.velocityMessage)
                rospy.sleep(self.goalDistance/self.VEL_LINEAR_MAX)
                # stop
                self.velocityMessage.linear.x=0
                self.velocityCommand.publish(self.velocityMessage)
                self.setpointReached=True
                print('Setpoint Reached')

            if self.robotStuck:
                self.setpointReached=False
                (self.setpointX,self.setpointY)=self.setpointBackup


    def collisionTimerCallback(self,event=None):
        if not self.setpointReached:
            if abs(self.currentPositionX-self.previousPositionX)<0.001 \
            and abs(self.currentPositionY-self.previousPositionY)<0.001:
            
                print("I'm stuck, going back")
                self.robotStuck=True
                self.setpointBackup=(self.setpointX,self.setpointY)
                self.setpointReached=False
                self.setpointX=self.lastMotilePositionX
                self.setpointY=self.lastMotilePositionY

            else:
                self.lastMotilePositionX=self.currentPositionX
                self.lastMotilePositionY=self.currentPositionY

            self.previousPositionX=self.currentPositionX
            self.previousPositionY=self.currentPositionY



if __name__ == '__main__':
    initialMessage = rospy.get_param('~message', '------starting turtlebot navigator------')
    nodeName=rospy.get_param('~node_name', 'turtlebot_navigator')

    print(initialMessage)
    rospy.init_node(nodeName,anonymous=True)

    turtlenavigator=navigatorNode()
    
    try:
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("turtle motion aborted!")