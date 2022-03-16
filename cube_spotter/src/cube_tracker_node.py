#!/usr/bin/env python
from __future__ import print_function
from re import X

import roslib
roslib.load_manifest('cube_spotter')
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from cube_spotter.msg import cubeData
from cube_spotter.msg import cubeArray
import numpy as np;

from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import GetJointPosition


class cubeTracker:

  def __init__(self):

    # Where the block is in the image (start at the centre)
    self.targetX=0.5
    self.targetY=0.5

    # Whether the robot is ready to move (assume it isn't)
    self.readyToMove=False

    # Home postion for the robot to move to
    self.jointPose=[0.0,0,0,1.3]

    # Create the subscribers
    self.image_sub = rospy.Subscriber('states',OpenManipulatorState,self.getStates)
    self.joint_state_sub = rospy.Subscriber('joint_states',JointState,self.getJoints)
    self.moving_sub = rospy.Subscriber('cubes',cubeArray,self.getTarget)

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





  # Get data on if the robot is currently moving
  def getStates(self,data):
    if (data.open_manipulator_moving_state=='"STOPPED"'):
      self.readyToMove=True
    else:
      self.readyToMove=False



  # Using the data from all the subscribers, call the robot's services to move the end effector
  def aimCamera(self):
    if self.readyToMove==True: # If the robot state is not moving

      # Extremely simple - aim towards the target using joints [0] and [3]
      if (abs(self.targetY-0.5)>0.1):
        self.jointRequest.position[3]=self.jointPose[3]+(self.targetY-0.5)

      if (abs(self.targetX-0.5)>0.1):
        self.jointRequest.position[0]=self.jointPose[0]-(self.targetX-0.5)

      # This command sends the message to the robot
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(1) # Sleep after sending the service request as you can crash the robot firmware if you poll too fast



  # Find the normalised XY co-ordinate of a cube
  def getTarget(self,data):

    # Example = track the biggest red object
    area=[]
    coX=[]
    coY=[]

    # Get the red cubes
    for c in range(len(data.cubes)):
      if (data.cubes[c].cube_colour=='red'):
        area.append(data.cubes[c].area)
        coX.append(data.cubes[c].normalisedCoordinateX)
        coY.append(data.cubes[c].normalisedCoordinateY)
      
    
    # Find the biggest red cube
    if (len(area))>0:
      index_max = max(range(len(area)), key=area.__getitem__)
      self.targetX=coX[index_max]
      self.targetY=coY[index_max]
    else: # If you dont find a target, report the centre of the image to keep the camera still
      self.targetX=0.5
      self.targetY=0.5

# Main 
def main(args):

  ic = cubeTracker()
  rospy.init_node('cube_tracker', anonymous=True)
  try:
    #rospy.spin()
    while not rospy.is_shutdown():
      ic.aimCamera() # This is the actual code which controls the robot
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)