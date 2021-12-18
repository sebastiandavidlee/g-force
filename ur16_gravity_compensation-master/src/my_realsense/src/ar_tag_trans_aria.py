#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np

from geometry_msgs.msg import Vector3

def getPos():

    # create subscriber to listen to tool frame topic

    pub = rospy.Publisher('/ar_tag_track/trans', Vector3, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    r = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        try:
            # tran datatype: geometry_msgs/TransformStamped
            # g_ca (ar_tag --> camera)
            # get translation from tf, and no rotation
            trans_ca = tfBuffer.lookup_transform("camera_link", "ar_marker_3", rospy.Time())
            translation = trans_ca.transform.translation
            x_constant = 0.3
            x = translation.x - x_constant
            g_ca = np.eye(4)
            g_ca[0][3] = translation.x
            g_ca[1][3] = translation.y
            g_ca[2][3] = translation.z
            print('g_ca:')
            
            print(g_ca)

            # g_tc (camera --> tool)
            # rotation + translation
            g_tc = np.eye(4)
            g_tc[0:3, 0:3] = np.array([[0, 0, 1],
                                       [1, 0, 0],
                                       [0, 1, 0]])
            g_tc[0][3] = 0.0
            g_tc[1][3] = 0.016
            g_tc[2][3] = 0.0284


            # g_bt (toll --> base)
            trans_bt = tfBuffer.lookup_transform("base_link", "tool0", rospy.Time())
            translation = trans_bt.transform.translation
            g_bt = np.eye(4)
            g_bt[0:3,0:3] = np.array([[0, 0, 1],
                                      [1, 0, 0],
                                      [0, 1, 0]])
            g_bt[0][3] = translation.x
            g_bt[1][3] = translation.y
            g_bt[2][3] = translation.z
            print('g_bt:')
            print(g_bt)

            # g_ba
            g_ba = np.dot(np.dot(g_bt, g_tc), g_ca)
            print('g_ba:')
            print(g_ba)

            # ar_tag position relative to base_frame
            vector = Vector3()
            vector.x = g_ba[0][3]
            vector.y = g_ba[1][3]
            vector.z = g_ba[2][3]
            print('pos:')
            print(vector)
            print('-----------------')

            pub.publish(vector)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
        r.sleep()


if __name__ == '__main__':

    rospy.init_node('ar_tag_trans', anonymous=True)
    try:
       getPos()
    except rospy.ROSInterruptException:
        pass

