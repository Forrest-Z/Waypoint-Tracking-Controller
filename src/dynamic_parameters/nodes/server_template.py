#!/usr/bin/python3

import rospy

from dynamic_reconfigure.server import Server
from dynamic_parameters.cfg import TemplateConfig


def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
      {str_param}, {bool_param}, {size}""".format(**config))
    return config


if __name__ == "__main__":
    rospy.init_node("server_template", anonymous = False)

    srv2 = Server(TemplateConfig, callback)
    rospy.spin()
