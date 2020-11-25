#!/usr/bin/env python  
import roslib
roslib.load_manifest('ar_marker_localization')
import rospy
import math
import tf
import geometry_msgs.msg
#import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('camera_tf_listener')

    listener = tf.TransformListener()

    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    #spawner(4, 2, 0, 'turtle2')

    #turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
 

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/camera_link', '/ar_marker_0', rospy.Time(0))
	    
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

	print('translation', trans)
	print('quaternion', rot)

        rate.sleep()
