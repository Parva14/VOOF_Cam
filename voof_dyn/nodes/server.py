#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from voof_dyn.cfg import voofConfig

def callback(config, level):

    rospy.loginfo("""Reconfigure Request: {tune_imu}, {tune_squal_1},\ 
          {tune_squal_2}, {tune_cam_1}, {tune_cam_1}""".format(**config))

    return config

if __name__ == "__main__":
    rospy.init_node("voof_dyn", anonymous = False)

    srv = Server(voofConfig, callback)
    rospy.spin()

