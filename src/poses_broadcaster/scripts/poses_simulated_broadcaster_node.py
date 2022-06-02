#!/usr/bin/env python3

# Author: Stephen Tellis

import math

# import ROS libraries
import rospy, tf

# Import pre-defined msg/srv/actions


# import custom message types


# Self created libs 


# Self created local classes


class DetectPoses:

    def __init__(self):
        self._br = tf.TransformBroadcaster()

    def _send_first_transform(self):
        """!
        A transform moving along the 
        +ve y axis
        """
        t = rospy.Time.now().to_sec() * (math.pi/50)
        self._br.sendTransform((0.8, 1*math.cos(t), 0.2),
                        (0.0, 0.0, 0.0, 1),
                         #(0.0, 1.0*0.7071, 0.0, 0.7071),
                         rospy.Time.now(),
                         "point1",
                         "base_link")

    def _send_second_transform(self):
        """!
        A transform moving along the 
        +ve y axis
        """
        t = rospy.Time.now().to_sec() * (math.pi/50)
        self._br.sendTransform((0.85, 0.75*math.cos(t + 1), 0.2),
                        (0.0, 0.0, 0.0, 1),
                         #(0.0, 1.0*0.7071, 0.0, 0.7071),
                         rospy.Time.now(),
                         "point2",
                         "base_link")

if __name__ == "__main__":
    rospy.init_node('poses_tf_broadcaster')
    dp = DetectPoses()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        dp._send_first_transform()
        dp._send_second_transform()
        rate.sleep()