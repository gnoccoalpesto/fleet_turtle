#!/usr/bin/env python3
import rospy
import os
import time
# import actionlib
from math import pi as PI
import numpy as np
# from actionlib.action_server import ActionServer
from actionlib_msgs.msg import GoalID
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D,PoseWithCovarianceStamped,TransformStamped,PoseStamped
import tf2_ros
from tf.transformations import euler_from_quaternion#,quaternion_from_euler
from fleet_turtle.msg import SetpointMulti
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
# from nav_msgs.srv import GetPlan,GetPlanRequest,GetPlanResponse

"""
====GAME RULES====

TURTLE OF INTEREST (TOI) will score a point every time it passes an imaginary line
    (try_line) at a certaint x coord value

TOI has also a health statistics, which can be lowered by the attacking team
to randomize things, each new episode, a different goal is generated, inside a given area

EPISODE CONCLUDES when:
- TOI surpasses try_line
- TOI healt reaches 0 -- unused
- game time expires

"""

class Referee:
    def __init__(self,toi_name,try_line_x,goal_x_range,goal_y_range,game_duration) -> None:

        self.ROBOTS=['robot1','robot2','robot3','robot4','robot5','robot6']
        self.ROBOT_NUMBERS={'robot1':0,
                            'robot2':1,
                            'robot3':2,
                            'robot4':3,
                            'robot5':4,
                            'robot6':5}
                            
        self.TOI=toi_name
        self.DEFENDERS=['robot2','robot3']
        self.ATTACKERS=['robot4','robot5','robot6']

        self.ROBOT_INITIAL_POSES = [[-2.,0.,0.],
                                    [-1.5,-1.,0.],[-1.5,1.,0.],
                                    [2.,-1.,PI],[2.,0.,PI],[2.,1.,PI]]
        self.robot_poses=[Pose2D()]*len(self.ROBOTS)

        #episode termination conditions
        # self.TOI_HEALTH_BAR=100
        self.GAME_DURATION=game_duration

        #coordinate to surpass to consider the game won
        self.TRY_LINE=try_line_x

        #rectangle in which TOI goal point is generated (randomly)
        self.GOAL_X_RANGE=goal_x_range
        self.GOAL_Y_RANGE=goal_y_range

        # self.SETPOINT_TOPIC = '/Setpoint'
        # self.toi_goal=SetpointMulti(robot_name=self.TOI)
        # self.setpoint_pub = rospy.Publisher(self.SETPOINT_TOPIC,SetpointMulti,queue_size=1)
        self.goal_pubs=[rospy.Publisher('/'+robot+'/move_base_simple/goal',PoseStamped,queue_size=1) for robot in self.ROBOTS]
        self.goal_cancel_pubs=[rospy.Publisher('/'+robot+'/move_base/cancel',GoalID,queue_size=1) for robot in self.ROBOTS]

        #some time to let the pub connect
        rospy.sleep(5)

        self.odom_sub=[]
        for robot in self.ROBOTS:
            odometry_topic="/"+robot+"/odom"
            if robot==self.TOI:
                behaviour=self.checkToiTry
            elif robot in self.DEFENDERS:
                behaviour=self.defendToi
            else:
                behaviour=self.attackToi
            #subs will spawn task with correct behaviour every time they receive /robotJ/odom
            self.odom_sub.append(rospy.Subscriber(odometry_topic, Odometry,behaviour, queue_size=5))

        self.TERMINATION_TIMER_DURATION=rospy.Duration(nsecs=1E7)
        self.RESET_TIMEOUT=35
        #in case some robot cannot reach its initial position during preparation phase,
        #   following game will be counted as null/less important
        self.last_game_valid=True

        self.game_counter=0

        self.setToiGoal()


    def setToiGoal(self):
        self.game_counter+=1
        print('\nSTART: generating random goal. GAME '+str(self.game_counter)+
                    " , duration: "+str(self.GAME_DURATION))
        goal=PoseStamped()
        goal_x=np.random.uniform(*self.GOAL_X_RANGE)
        goal_y=np.random.uniform(*self.GOAL_Y_RANGE)
        goal_theta=np.random.uniform(-PI/4,PI/4)
        goal.header.frame_id='map'
        goal.pose.position.x=goal_x
        goal.pose.position.y=goal_y
        goal.pose.orientation.w=1
        # self.toi_goal.pose=Pose2D(goal_x,goal_y,goal_theta)
        self.START_TIME=time.time()
        self.GAME_OVER=False
        self.termination_timer=rospy.Timer(self.TERMINATION_TIMER_DURATION,self.checkTerminalConditions)
        # self.setpoint_pub.publish(self.toi_goal)
        self.goal_pubs[self.ROBOT_NUMBERS[self.TOI]].publish(goal)


    def checkToiTry(self,odom_msg):
        self.updateRobotPose(self.TOI,odom_msg.pose.pose)
        rospy.sleep(.1)


    def defendToi(self,odom_msg):
        robot=self.nameFromOdometry(odom_msg.child_frame_id)
        self.updateRobotPose(robot,odom_msg.pose.pose)
        if not self.GAME_OVER:
            random_attacker=np.random.randint(0,len(self.ATTACKERS))
            defense_pose=PoseStamped()
            defense_pose.header.frame_id='map'
            defense_pose.pose.position.x=self.robot_poses[self.ROBOT_NUMBERS[self.ATTACKERS[random_attacker]]].x
            defense_pose.pose.position.y=self.robot_poses[self.ROBOT_NUMBERS[self.ATTACKERS[random_attacker]]].y
            defense_pose.pose.orientation.w=1
            self.goal_pubs[self.ROBOT_NUMBERS[robot]].publish(defense_pose)
            # defense_pose=Pose2D(self.robot_poses[self.ROBOT_NUMBERS[self.ATTACKERS[0]]].x,
            #                     self.robot_poses[self.ROBOT_NUMBERS[self.ATTACKERS[0]]].y,
            #                     self.robot_poses[self.ROBOT_NUMBERS[self.ATTACKERS[0]]].theta
            # )
            # self.setpoint_pub.publish(SetpointMulti(robot,defense_pose))
        rospy.sleep(.5)


    def attackToi(self,odom_msg):
        robot=self.nameFromOdometry(odom_msg.child_frame_id)
        self.updateRobotPose(robot,odom_msg.pose.pose)
        if not self.GAME_OVER:
            attack_pose=PoseStamped()
            attack_pose.header.frame_id='map'
            attack_pose.pose.position.x=self.robot_poses[self.ROBOT_NUMBERS[self.TOI]].x
            attack_pose.pose.position.y=self.robot_poses[self.ROBOT_NUMBERS[self.TOI]].y
            attack_pose.pose.orientation.w=1
            self.goal_pubs[self.ROBOT_NUMBERS[robot]].publish(attack_pose)
            # attack_pose=Pose2D(self.robot_poses[self.ROBOT_NUMBERS[self.TOI]].x,
            #                     self.robot_poses[self.ROBOT_NUMBERS[self.TOI]].y,
            #                     self.robot_poses[self.ROBOT_NUMBERS[self.TOI]].theta
            # )
            # self.setpoint_pub.publish(SetpointMulti(robot,attack_pose))
        rospy.sleep(.5)


    def checkTerminalConditions(self,_):
        #TOI PASSED TRY LINE
        if self.robot_poses[self.ROBOT_NUMBERS[self.TOI]].x>= self.TRY_LINE:
            print('GAME OVER: TOI won',end="")
            self.GAME_OVER=True

        #TOI HEALTH DEPLETED
        # if self.TOI_HEALTH_BAR<=0:
        #     print('TOI died, ',end="")
        #     self.GAME_OVER=True

        #TIME EXPIRED
        if time.time()-self.START_TIME >=self.GAME_DURATION:
            print("GAME OVER: time's up",end="")
            self.GAME_OVER=True

        if self.GAME_OVER:
            print(', game was'+(" " if self.last_game_valid else " NOT ")+"valid")
            self.cancelAllGoals()
            self.robotsToInitialPoses()
            self.RESET_START=time.time()
            self.termination_timer.shutdown()
            self.reset_timer=rospy.Timer(self.TERMINATION_TIMER_DURATION,self.checkInitialConditions)


    def cancelAllGoals(self):
        for robot in self.ROBOTS:
            robot_number=self.ROBOT_NUMBERS[robot]  
            cancel_goal=GoalID()
            self.goal_cancel_pubs[robot_number].publish(cancel_goal)


    def robotsToInitialPoses(self):
        for robot in self.ROBOTS:
            robot_number=self.ROBOT_NUMBERS[robot]  
            initial_pose=PoseStamped()
            initial_pose.header.frame_id='map'
            initial_pose.pose.position.x=self.ROBOT_INITIAL_POSES[robot_number][0]
            initial_pose.pose.position.y=self.ROBOT_INITIAL_POSES[robot_number][1]
            initial_pose.pose.orientation.w=1
            self.goal_pubs[robot_number].publish(initial_pose)
            rospy.sleep(1.5)


    def checkInitialConditions(self,_):
        reset_complete=True
        for robot in self.ROBOTS:
            robot_number=self.ROBOT_NUMBERS[robot]
            reset_complete*=(abs(self.robot_poses[robot_number].x - self.ROBOT_INITIAL_POSES[robot_number][0])<.2)* \
                            (abs(self.robot_poses[robot_number].x == self.ROBOT_INITIAL_POSES[robot_number][1])<.2)
                            # *(self.robot_poses[robot_number].theta ==self.ROBOT_INITIAL_POSES[robot_number][2])
        if reset_complete:
            print("preparation completed")
            self.last_game_valid=True
            self.reset_timer.shutdown()
            self.cancelAllGoals()
            self.setToiGoal()

        if time.time()-self.RESET_START>=self.RESET_TIMEOUT:
            print("PREPARATION PHASE RESERVED TIME EXPIRED")
            self.last_game_valid=False
            self.reset_timer.shutdown()
            self.cancelAllGoals()
            self.setToiGoal()


    def updateRobotPose(self,robot,pose3d):
        pose2d = Pose2D(round(pose3d.position.x,3),
                        round(pose3d.position.y,3),
                        0.0
                        # round(euler_from_quaternion(pose3d.orientation)[2],3)
                        )
        robot_number=self.ROBOT_NUMBERS[robot]
        self.robot_poses[robot_number]=pose2d


    @staticmethod
    def nameFromOdometry(frame):
        return frame.split('_')[0]


if __name__ == '__main__':
    print('referee node for game start, termination and restart')
    rospy.init_node('referee',anonymous=False)
    try_line_x=.5
    referee=Referee(toi_name='robot1',
                    try_line_x=try_line_x,
                    goal_x_range=[try_line_x+.05,try_line_x+2],
                    goal_y_range=[-2.5,2.5],
                    game_duration=15)

    try:
        rospy.spin()
    except KeyboardInterrupt or rospy.ROSInternalException:
        rospy.loginfo("turtle motion aborted!")



        """
        WINNING CONDITION: x_TOI>1.5

        TOI GOAL: generated inside rectangular area:
                        x in [1.6,2], y in [-1,+1]







HOW TO RESET WORLD?
GAZEBO:
https://campus-rover.gitbook.io/lab-notebook/faq/reset-world-gazebo

but controller/odom does not 

https://answers.ros.org/question/213049/how-do-i-reset-the-odom-topic-back-to-0-without-restarting-the-robot/



        # self.initial_pose_tf=[]
        # self.tf_broadcaster=tf2_ros.StaticTransformBroadcaster()
        # self.RESET_SERVICE="/gazebo/reset_world"
        # print('service for resetting episode online')
        # rospy.wait_for_service(self.RESET_SERVICE)
        # self.reset_world=rospy.ServiceProxy(self.RESET_SERVICE,Empty)






ROBOTS=['robot1','robot2','robot3','robot4','robot5','robot6']
DEFENDERS=['robot2','robot3']
ATTACKERS=['robot4','robot5','robot6']
TOI_NUMBER=int()
TOI=str()
TOI_HEALTH_BAR=float()
GAME_DURATION=float()
TRY_LINE=float()
GOAL_X_RANGE=float()
GOAL_Y_RANGE=float()
SETPOINT_TOPIC='/Setpoint'
setpoint_pub=None
toi_goal=SetpointMulti()
odom_subs=[]


def Referee(toi_number,try_line_x,goal_x_range,goal_y_range,game_duration):
    global  TOI_NUMBER, TOI, \
            TOI_HEALTH_BAR,GAME_DURATION, TRY_LINE, \
            GOAL_X_RANGE,GOAL_Y_RANGE, toi_goal, \
            setpoint_pub, odom_subs

    TOI_NUMBER=toi_number-1
    TOI=ROBOTS[TOI_NUMBER]

    #episode termination conditions
    TOI_HEALTH_BAR=100
    GAME_DURATION=game_duration

    #coordinate to surpass to consider the game won
    TRY_LINE=try_line_x

    #rectangle in which TOI goal point is generated (randomly)
    GOAL_X_RANGE=goal_x_range
    GOAL_Y_RANGE=goal_y_range
    toi_goal=SetpointMulti(robot_name=TOI)

    setpoint_pub=rospy.Publisher(SETPOINT_TOPIC,SetpointMulti,queue_size=1)

    for robot in ROBOTS:
        odometry_topic="/"+robot+"/odom"
        if robot==TOI:
            behaviour=checkToiTry
        elif robot in DEFENDERS:
            behaviour=defendToi
        else:
            behaviour=attackToi
        # odom_subs.append(rospy.Subscriber(odometry_topic, Odometry,behaviour, queue_size=5))
    rospy.sleep(5)
    setToiGoal()


def setToiGoal():
    global toi_goal
    print('genereting goal')
    #generate random goal
    goal_x=np.random.uniform(*GOAL_X_RANGE)
    goal_y=np.random.uniform(*GOAL_Y_RANGE)
    toi_goal.pose=Pose2D(goal_x,goal_y,0.0)
    print(toi_goal)
    setpoint_pub.publish(toi_goal)


def checkToiTry(odom_msg):
    print('t')


def defendToi(odom_msg):
    print('d')


def attackToi(odom_msg):
    print('a')



    

    def initializePose(self,robot):
        
        tf_stamped=TransformStamped()
        tf_stamped.header.stamp=rospy.Time.now()
        tf_stamped.child_frame_id=robot+'_tf/odom'
        tf_stamped.header.frame_id='map'
        self.initial_pose_tf.append(tf_stamped)

        robot_number=self.ROBOT_NUMBERS[robot]
        initial_pose_pub=rospy.Publisher('/robot'+str(robot_number+1)+'/initialpose',PoseWithCovarianceStamped, queue_size = 1)
        initial_pose=PoseWithCovarianceStamped()
        initial_pose.header.frame_id="map"
        initial_pose.header.stamp = rospy.get_rostime()
        initial_pose.pose.pose.position.x=self.ROBOT_INITIAL_POSES[robot_number][0]
        initial_pose.pose.pose.position.y=self.ROBOT_INITIAL_POSES[robot_number][1]
        initial_pose_pub.publish(initial_pose)

        os.system('rosservice call /move_base'+str(robot_number+1)+'/clear_costmaps "{}"')


        """