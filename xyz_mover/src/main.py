#!/usr/bin/env python
from __future__ import print_function
from re import X

import roslib
roslib.load_manifest('cube_spotter')
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from cube_spotter.msg import cubeData
from cube_spotter.msg import cubeArray
import numpy as np;

from xyz_mover.msg import Instruction

from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import GetJointPosition


# Simulation setup
# https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_simulation/

class cubeTracker:

  def __init__(self):

    # Whether the robot is ready to move (assume it isn't)
    self.readyToMove=False

    # Home postion for the robot to move to
    self.jointPose=[0.0,0,0,1.3]

    # Create the subscribers
    self.image_sub = rospy.Subscriber('states',OpenManipulatorState,self.getStates)
    self.joint_state_sub = rospy.Subscriber('joint_states',JointState,self.getJoints)

    self.instructionSub = rospy.Subscriber('instructions', Instruction, self.getInstructions)

    # Create the service caller to move the robot
    self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)

    # Send the robot "home"
    self.jointRequest=JointPosition()
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
    self.jointRequest.position=[0.0,0,0,1.3]
    request=SetJointPosition()
    request.joint_position=self.jointRequest
    request.path_time=1

    self.setPose(str(),self.jointRequest,1.0)


  # Get the robot's joint positions
  def getJoints(self,data):
    self.jointPose=data.position


  # Get instructions
  def getInstructions(self, data):
    if self.readyToMove:
      self.jointRequest=JointPosition()
      self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]
      self.jointRequest.position = [data.joint1, data.joint2, data.joint3, data.joint4]
      request=SetJointPosition()
      request.joint_position=self.jointRequest
      request.path_time=1

      self.setPose(str(),self.jointRequest,1.0)


  # Get data on if the robot is currently moving
  def getStates(self,data):
    if (data.open_manipulator_moving_state=='"STOPPED"'):
      self.readyToMove=True
    else:
      self.readyToMove=False



# Main 
def main(args):

  ic = cubeTracker()
  rospy.init_node('cube_tracker', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)