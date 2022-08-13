#!/usr/bin/env python3
import rospy
import os
# import actionlib
from math import pi as PI
import numpy as np
# from actionlib.action_server import ActionServer
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D,PoseWithCovarianceStamped,TransformStamped
import tf2_ros
from tf.transformations import euler_from_quaternion#,quaternion_from_euler
from fleet_turtle.msg import SetpointMulti
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
# from nav_msgs.srv import GetPlan,GetPlanRequest,GetPlanResponse




if __name__ == '__main__':
    print('test tf')
    rospy.init_node('test_tf',anonymous=False)


    tf_broadcaster=tf2_ros.TransformBroadcaster()
    # tf_broadcaster=tf2_ros.StaticTransformBroadcaster()



    tf_stamped=TransformStamped()
    tf_stamped.header.stamp=rospy.Time.now()
    tf_stamped.child_frame_id='robot1_tf/odom'
    tf_stamped.header.frame_id='/map'
    tf_stamped.transform.translation.x=1
    tf_stamped.transform.translation.y=1
    tf_stamped.transform.rotation.w=1

    # tf_broadcaster.sendTransform(tf_stamped)

    [tf_broadcaster.sendTransform(tf_stamped) for conter in range(0,100000)]