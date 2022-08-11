#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
import std_srvs.srv
import rosnode
import actionlib
# from actionlib.action_server import ActionServer
from move_base_msgs.msg import MoveBaseAction


# from reference_listener import TFListener
import tf2_ros
import tf2_msgs.msg

class TFListener:
    def __init__(self,child,father,use_lookup,direct_transform):
        
        self.DIRECT_TRANSFORM=direct_transform
        self.LISTEN_WITH_LOOKUP=use_lookup
        self.messageRead=False

        if self.LISTEN_WITH_LOOKUP:
            self.TFBuffer = tf2_ros.Buffer()
            self.Listener = tf2_ros.TransformListener(self.TFBuffer)
        else:
            self.Listener=rospy.Subscriber('/tf',tf2_msgs.msg.TFMessage,\
                                            self.TFCallback,queue_size=1)

        # self.nodeName=rospy.get_name()
        self.target_frame_id =child
        self.starting_frame_id =father
        # self.target_frame_id = rospy.get_param('~frame_id','%s_father_link'%self.nodeName)
        # self.starting_frame_id = rospy.get_param('~child_frame_id','%s_child_link'%self.nodeName)
        # self.target_frame_id = 'imu_link'
        # self.starting_frame_id = 'scan_link'
        print('child frame: '+child)
        print('father frame: '+father)

        # self.listenerTimer=rospy.timer(rospy.Duration(0.1), self.listenTF)

        if self.LISTEN_WITH_LOOKUP:
            rate = rospy.Rate(10.0)
            while not rospy.is_shutdown():
                try:
                    self.transform = self.TFBuffer.lookup_transform(\
                        self.target_frame_id, self.starting_frame_id, rospy.Time())
                except  (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
                            tf2_ros.ExtrapolationException):
                    rate.sleep()
                    continue

                self.messageRead=True
                rate.sleep()

    def TFCallback(self,data):
        for data_content in data.transforms:
            if data_content.child_frame_id==self.target_frame_id and (\
                (self.DIRECT_TRANSFORM\
                and data_content.header.frame_id==self.starting_frame_id)\
            or not self.DIRECT_TRANSFORM ):
                self.transform=data_content.transform
                self.messageRead=True


class initialPositionSetter:

    def __init__(self,child,father,use_lookup,direct_transform):
        # topics to listen
        self.odometryListener=TFListener(child=child,father=father,\
                    use_lookup=use_lookup,direct_transform=direct_transform)
        # selects the initialization method
        self.PUSH_ON_INITIALPOSE=True
        # waits until move_base server is online
        self.MBClient=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.MBClient.wait_for_server()
        # sets up the costmap cleaner routine
        self.costCleaner=rospy.ServiceProxy('move_base/clear_costmaps',\
                                             std_srvs.srv.Empty)
        #rosservice call /move_base/clear_costmaps "{}"
    
        if self.PUSH_ON_INITIALPOSE:
            pubCounter=0
            # initializes initialpose publisher
            self.initialPositionPusher=rospy.Publisher('initialpose',\
                geometry_msgs.msg.PoseWithCovarianceStamped, queue_size = 5)
            self.poseToPush=geometry_msgs.msg.PoseWithCovarianceStamped()
            rate=rospy.Rate(10)
            while not rospy.is_shutdown() and pubCounter<25:
                # pushes received odometry messase to the initialpose topic
                if self.odometryListener.messageRead:
                    self.poseToPush.header.frame_id='map'
                    # self.poseToPush.header.stamp = rospy.get_rostime()
                    self.poseToPush.pose.pose.position=self.odometryListener.transform.translation
                    self.poseToPush.pose.pose.orientation=self.odometryListener.transform.rotation
                    self.initialPositionPusher.publish(self.poseToPush)
                    # loop termination condition
                    pubCounter=pubCounter+1
                rate.sleep()
            #removes obstacles from costmap
            self.costCleaner()
            # kills itself
            rosnode.kill_nodes([rospy.get_name()])
        '''
        else:
            send empty request to /global_localization
        '''

if __name__=='__main__':

    rospy.init_node('initializer',anonymous=True)
    targetFrame=rospy.get_param('~target_frame','base_footprint')
    sourceFrame=rospy.get_param('~source_frame','odom')
    pos=initialPositionSetter(child=targetFrame,\
            father=sourceFrame,use_lookup=False,direct_transform=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('bye')