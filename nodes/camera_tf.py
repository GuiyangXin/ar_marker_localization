#!/usr/bin/env python  
import roslib
import rospy
from geometry_msgs.msg import Pose
# import math
import tf

roslib.load_manifest('ar_marker_localization')


def talker(trans1, rot1):

    T = Pose()
    T.position.x = trans1[0]
    T.position.y = trans1[1]
    T.position.z = trans1[2]
    T.orientation.x = rot1[0]
    T.orientation.y = rot1[1]
    T.orientation.z = rot1[2]
    T.orientation.w = rot1[3]

    # rospy.loginfo(T)
    pub.publish(T)
    # rate.sleep()


if __name__ == '__main__':
    rospy.init_node('camera_tf')

    listener = tf.TransformListener()
    pub = rospy.Publisher('camera2markerTransformation', Pose, queue_size=10)
    rate = rospy.Rate(10.0)  # 10hz
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/base_link', '/ar_marker_0', rospy.Time(0))
            talker(trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

    # print('translation', trans)
    # print('quaternion', rot)
