#! /usr/bin/env python3

import roslib
import rospy
import actionlib

from search_follow_avoid.msg import AvoidAction

class AvoidSwerver:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('avoid', AvoidAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    rospy.sleep(10)
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('avoid')
  server = AvoidSwerver()
  rospy.spin()