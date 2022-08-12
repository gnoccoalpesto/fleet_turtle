#!/usr/bin/env python3
from time import sleep
import rospy
import actionlib
import math
# from actionlib.action_server import ActionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Pose2D
# import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from fleet_turtle.msg import Setpoint,SetpointMulti
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan,GetPlanRequest,GetPlanResponse

class navigationNode():
    def __init__(self):

        self.next_pose_goal=Pose2D()
        self.current_world_pose=Pose2D()
        self.previous_world_pose=Pose2D()
        self.setpoint_reached=True
        
        # cinematic parameters
        # self.MAX_LINEAR_VELOCITY=0.22
        # self.MAX_ROTATION_VELOCITY=2.84#[rad/s] of the robot {==162.72 deg/s}

        # self.MOTION_MONITOR_PERIOD=0.8
        # self.collision_counter=0
        # self.COLLISION_DISTANCE=0.5*self.MAX_LINEAR_VELOCITY/self.MOTION_MONITOR_PERIOD
        # self.COLLISION_ANGLE=12 #unused
        # self.MAX_COLLISION_COUNT=18
        # self.robot_is_stuck=False

        # self.proximity_counter=0
        # self.PROXIMITY_DISTANCE=0.6
        # self.MAX_PROXIMITY_COUNT=17
        
        #TODO: get namespaces from the aether (or use as ns robot_names, hardcoded smwhere else)
        ROBOT_NAMES=['robot1','robot2','robot3','robot4','robot5']
        self.move_base_client=[actionlib.SimpleActionClient('/'+robot+'/move_base',MoveBaseAction) for robot in ROBOT_NAMES]
        [[client.wait_for_server(), print("move base client for robot{} online".format(robot_number+1))]  \
            for robot_number,client in enumerate(self.move_base_client)]
            
        #TODO: gets plan data, lenght, ...
        # self.PlanClient=rospy.ServiceProxy('/move_base/make_plan',GetPlan)

        self.VELOCITY_TOPIC= rospy.get_param('~velocity_topic', '/cmd_vel')
        self.ODOMETRY_TOPIC= rospy.get_param('~odometry_topic','/odom')
        self.SETPOINT_TOPIC = rospy.get_param('~setpoint_topic', '/Setpoint')

        
        self.setpoint_sub = rospy.Subscriber(self.SETPOINT_TOPIC,SetpointMulti,self.readSetpoint,queue_size=5)
        print("started setpoint listener for SetpointMulti(string robot_name,Pose2D pose) msg type ")
        self.odometry_sub=rospy.Subscriber(self.ODOMETRY_TOPIC, Odometry,self.readOdometry, queue_size=5)

        # self.velocityCommand=rospy.Publisher(self.VELOCITY_TOPIC,Twist,queue_size=5)
        

    def readOdometry(self,odom_msg):
    # updates current position in WORLD frame
    ###################
    # REGULATOR VERSION
    #     orientation = data.pose.pose.orientation
    #     position = data.pose.pose.position
    #     siny_cosp = 2*(orientation.w*orientation.z+orientation.x*orientation.y)
    #     cosy_cosp = 1-2*(orientation.y**2 + orientation.z**2)
    #     theta = math.atan2(siny_cosp,cosy_cosp)
    #     x = position.x + self.LEVER*math.cos(theta)
    #     y = position.y + self.LEVER*math.sin(theta)
    ####################
        x=round(odom_msg.pose.pose.position.x,4)
        y=round(odom_msg.pose.pose.position.y,4)
        theta=round(euler_from_quaternion((odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,\
                odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w))[2],4)
        self.current_world_pose=Pose2D(x,y,theta)


    def readSetpoint(self,setpoint_msg):
        robot_name=setpoint_msg.robot_name
        pose=setpoint_msg.pose
        self.next_pose_goal=pose
        self.requestNewGoal(robot_name,pose)


    def updateReferencePose(self):
    # updates reference position in WORLD frame; initialize point for collision detection
        self.previous_world_pose=self.referenceWpoint=self.current_world_pose


# SECONDARY BEHAVIOUR ##########################################################
            
    def motionMonitor(self,event=None):#,straight_recovery=False):
    # monitor correct motion
        self.distance_from_goal=self.pointDistance(self.current_world_pose,self.next_pose_goal)
        # checks if robot is taking too long to reach the goal from a close position
        proximity_result=self.proximityCheck()
        if proximity_result:
            print('Robot is close enough to the target')
            #TODO: must know which client
            self.move_base_client.cancel_goal()
        # checks if robot is blocked in a given position for too long
        collision_result=self.collisionCheck()
        if collision_result:
            print('robot is stuck near point {}, {}, with orientation {}'\
              .format(self.previous_world_pose.x,self.previous_world_pose.y,self.previous_world_pose.theta))
            # if not self.robot_is_stuck:
            #     self.robot_is_stuck=True
            self.monitor.shutdown()
            # self.resetMonitor()
            self.move_base_client.cancel_goal()
                # self.randomMotion()
                # self.requestNewGoal(self.referenceWpoint)
                # print('going back to starting point')
        # if not (collision_result or proximity_result):
        #     if not self.robot_is_stuck:
        #         self.robot_is_stuck=True
                # self.monitor.shutdown()
                # self.resetMonitor()
                # self.move_base_client.cancel_goal()
                # self.randomMotion()
                # self.requestNewGoal(self.referenceWpoint)
                # print('going back to starting point')
        

    def resetMonitor(self):
    # resets motion monitor's counters and state
        self.collision_counter=0
        self.proximity_counter=0
        self.robot_is_stuck=False
        self.robot_close_to_target=False

    # TODO improve this by using cmd_vel signal and position measuring
    def collisionCheck(self):
    # checks if the robot stays close to the same point for too long, hence counts
        if self.pointDistance(self.current_world_pose,self.previous_world_pose)<self.COLLISION_DISTANCE:
        # or abs(self.current_world_pose.theta-self.previous_world_pose.theta)<self.COLLISION_ANGLE:
            self.collision_counter+=1
        else: 
            # if not, start over
            self.robot_is_stuck=False
            self.collision_counter=0
            self.previous_world_pose=self.current_world_pose
        # if robot close to target, wait much longer
        if self.collision_counter>self.MAX_COLLISION_COUNT and not self.robot_close_to_target\
        or self.collision_counter>self.MAX_COLLISION_COUNT*2 and self.robot_close_to_target:
            self.collision_counter=0
            self.robot_is_stuck=True
            return True
        else: return False

    def proximityCheck(self):
    # checks if robot is close, yet not at, the target
        if self.distance_from_goal<self.PROXIMITY_DISTANCE:
            self.robot_close_to_target=True
            # if it's close enough, counts
            if self.distance_from_goal<self.PROXIMITY_DISTANCE*0.42:
                self.proximity_counter+=1
            # otherwise starts over
            else: self.proximity_counter=0
        else: self.robot_close_to_target=False
        if self.robot_close_to_target and self.proximity_counter>self.MAX_PROXIMITY_COUNT:
            return True
        else: return False


    # MOTION ##############################################################
    
    def requestNewGoal(self,robot,pose,check_plan=False,tolerance=0.2,duration=0,print_message=False):
    
        self.updateReferencePose()

        #remove obstacles from costmap
        # import std_srvs.srv
        # rospy.ServiceProxy('move_base/clear_costmaps', std_srvs.srv.Empty)

        goal=MoveBaseGoal()
        goal.target_pose.header.frame_id='map'
        goal.target_pose.header.stamp=rospy.Time.now()
        goal.target_pose.pose.position.x=self.next_pose_goal.x
        goal.target_pose.pose.position.y=self.next_pose_goal.y
        goal.target_pose.pose.position.z=0
        (qx,qy,qz,qw)=quaternion_from_euler(*(0,0,self.next_pose_goal.theta))
        goal.target_pose.pose.orientation.x=qx
        goal.target_pose.pose.orientation.y=qy
        goal.target_pose.pose.orientation.z=qz
        goal.target_pose.pose.orientation.w=qw

        if robot=="robot1": client_number=0
        elif robot=="robot2": client_number=1
        elif robot=="robot3": client_number=2
        elif robot=="robot4": client_number=3
        elif robot=="robot5": client_number=4
        else:
            print("requested robot unavailable, aborting"); return False
        self.move_base_client[client_number].send_goal(goal)

        # self.move_base_client.send_goal(goal,self.done_cb,self.active_cb,self.feedback_cb)
        
        # activates motion monitor
        # self.resetMonitor()
        # self.monitor=rospy.Timer(rospy.Duration(self.MOTION_MONITOR_PERIOD),self.motionMonitor)#straight_recovery))
        request_result=self.move_base_client[client_number].wait_for_result() if duration==0\
                else self.move_base_client[client_number].wait_for_result(timeout=rospy.Duration(duration))
        # self.monitor.shutdown()
        return request_result

    # UTILITIES ###################################################################
    @staticmethod
    def pointDistance(point_a,point_b):
        return math.sqrt((point_a.x-point_b.x)**2+(point_a.y-point_b.y)**2)


if __name__ == '__main__':
    # sleep(100)
    print('==#== MOVE BASE ACTIONS BROADCASTER NODE ==#==')
    rospy.init_node('move_base_client',anonymous=True)

    fleet_navigator=navigationNode()

    try:
        rospy.spin()
    except KeyboardInterrupt or rospy.ROSInternalException:
        rospy.loginfo("turtle motion aborted!")