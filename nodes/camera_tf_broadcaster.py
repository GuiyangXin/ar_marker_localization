#!/usr/bin/env python  
import roslib
roslib.load_manifest('ar_marker_localization')
import rospy

import tf
#import turtlesim.msg

'''
def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")
'''

if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')
    cameraName = rospy.get_param('~camera')
    #rospy.Subscriber('/%s/pose' % turtlename,
    #                 turtlesim.msg.Pose,
    #                 handle_turtle_pose,
    #                 turtlename)
    rospy.spin()
