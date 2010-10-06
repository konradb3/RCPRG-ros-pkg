#! /usr/bin/env python

import roslib; roslib.load_manifest('sarkofag_launch')
import rospy
import actionlib

from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import *

if __name__ == '__main__':
  rospy.init_node('simple_trajectory')
  client = actionlib.SimpleActionClient('sarkofag_controller/joint_trajectory_action', JointTrajectoryAction)
  client.wait_for_server()

  goal = JointTrajectoryGoal()
  goal.trajectory.joint_names = ['crank_joint']
  goal.trajectory.points.append(JointTrajectoryPoint([0.6], [0], [], rospy.Duration(5.0)))
  goal.trajectory.points.append(JointTrajectoryPoint([-0.6], [0], [], rospy.Duration(10.0)))

  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

  client.send_goal(goal)

  client.wait_for_result()

