#! /usr/bin/env python
# from re import X

import math
import roslib
roslib.load_manifest('cube_spotter')
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from cube_spotter.msg import cubeData
from cube_spotter.msg import cubeArray
from cube_spotter.msg import cube_co
from cube_spotter.msg import Cube_co_Array
from math import cos, pi, sin, tan
import numpy as np;

from std_msgs.msg import Float64
from geometry_msgs.msg import Point

from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import GetJointPosition

from robotics_cswk_kin.srv import FKinMsg, FKinMsgRequest, FKinMsgResponse


class cubeTracker:

  def __init__(self):

    self.Cube_Cordinates = rospy.Publisher('Cordinates', Cube_co_Array, queue_size=9)

    # Where the block is in the image (start at the centre)
    self.targetX=0.5
    self.targetY=0.5

    # Whether the robot is ready to move (assume it isn't)
    self.readyToMove=False

    # Home postion for the robot to move to
    self.jointPose=[0.0,-1.05,0.357,0.703]

    # Create the subscribers
    self.image_sub = rospy.Subscriber('states',OpenManipulatorState,self.getStates)
    self.joint_state_sub = rospy.Subscriber('joint_states',JointState,self.getJoints)
    self.moving_sub = rospy.Subscriber('cubes',cubeArray,self.getTarget)

    # Create the service caller to move the robot
    self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
    self.setGripper = rospy.ServiceProxy('goal_tool_control', SetJointPosition)


    ## EXAMPLE POSES - Start by moving the robot to some example positions, and use the gripper

    self.fwd_kin_proxy = rospy.ServiceProxy("/fwd_kin", FKinMsg)
    rospy.wait_for_service("/fwd_kin")

    # As the last movement called was the arm, we dont update the request again below, but it would be necessary if switching between the arm and the gripper.


  # Get the robot's joint positions
  def getJoints(self,data):
    self.jointPose=data.position


  # Get data on if the robot is currently moving
  def getStates(self,data):
    if (data.open_manipulator_moving_state=='"STOPPED"'):
        self.readyToMove=True
    else:
        self.readyToMove=False



  # Find the normalised XY co-ordinate of a cube
  def getTarget(self,data):

    # Example = track the biggest red object
    area=[]
    coX=[]
    coY=[]
    cubeColors=[]
    
    Cube_Array_coridinates = Cube_co_Array()

    for c in range(len(data.cubes)):
      if not data.cubes[c].cube_colour in cubeColors:
        area.append(data.cubes[c].area)
        coX.append(data.cubes[c].normalisedCoordinateX)
        coY.append(data.cubes[c].normalisedCoordinateY)
        cubeColors.append(data.cubes[c].cube_colour)

    fkin_request = FKinMsgRequest()
    response = self.fwd_kin_proxy(fkin_request)

    camera_x = response.position.x
    camera_y = response.position.y
    camera_z = response.position.z + 0.08
    camera_angle = response.angle.data
    

    # Find the biggest red cube
    for i in range(len(cubeColors)):
        c = cube_co()
        self.targetX=coX[i]
        self.targetY=coY[i]

        if 0.2 < self.targetX < 0.8 and 0.2 < self.targetY < 0.8:
          angle_x = (coX[i] - 0.5) * 52.4 / 180.0 * math.pi
          angle_y = (coY[i] - 0.5) * 41.4 / 180.0 * math.pi

          Distance = ((camera_z) / cos(pi / 2 - camera_angle - angle_y))

          Real_x = (tan(angle_x) * Distance)
          Real_y = (tan(pi / 2 - camera_angle - angle_y)*camera_z)

          New_Real_x = (cos(-self.jointPose[0]) * Real_y) + (sin(-self.jointPose[0]) * Real_x)
          New_Real_y = (cos(-self.jointPose[0]) * Real_x) + (sin(-self.jointPose[0]) * Real_y)

          red_x_co = New_Real_x + camera_x
          red_y_co = New_Real_y - camera_y

          print("Cube" + cubeColors[i])

          c.x_co.data = red_x_co
          c.y_co.data = red_y_co
          c.CubeColour.data = cubeColors[i]
          Cube_Array_coridinates.Position.append(c)

    else: # If you dont find a target, report the centre of the image to keep the camera still
        self.targetX=0.5
        self.targetY=0.5

    self.Cube_Cordinates.publish(Cube_Array_coridinates)
    return

    for c in range(len(data.cubes)):
        if (data.cubes[c].cube_colour=='blue'):
            area.append(data.cubes[c].area)
            coX.append(data.cubes[c].normalisedCoordinateX)
            coY.append(data.cubes[c].normalisedCoordinateY)
    
    # Find the biggest blue cube
    if (len(area))>0:
        c = cube_co()
        index_max = max(range(len(area)), key=area.__getitem__)
        self.targetX=coX[index_max]
        self.targetY=coY[index_max]
        blue_x_co = (cos(self.jointPose[0])*(tan(pi - camera_angle)*(camera_z))) + camera_x
        blue_y_co = (sin(self.jointPose[0])*(tan(pi - camera_angle)*(camera_z))) + camera_y
        c.x_co = blue_x_co
        c.y_co = blue_y_co
        c.CubeColour = 'blue'
        Cube_Array_coridinates.Position.append(c)

    else: # If you dont find a target, report the centre of the image to keep the camera still
        self.targetX=0.5
        self.targetY=0.5


    for c in range(len(data.cubes)):
        if (data.cubes[c].cube_colour=='yellow'):
            area.append(data.cubes[c].area)
            coX.append(data.cubes[c].normalisedCoordinateX)
            coY.append(data.cubes[c].normalisedCoordinateY)
    
    # Find the biggest yellow cube
    if (len(area))>0:
        c = cube_co()
        index_max = max(range(len(area)), key=area.__getitem__)
        self.targetX=coX[index_max]
        self.targetY=coY[index_max]

        yellow_x_co = (cos(self.jointPose[0])*(tan(pi - camera_angle)*(camera_z))) + camera_x
        yellow_y_co = (sin(self.jointPose[0])*(tan(pi - camera_angle)*(camera_z))) + camera_y
        c.x_co = yellow_x_co
        c.y_co = yellow_y_co
        c.CubeColour = 'yellow'
        Cube_Array_coridinates.Position.append(c)

    else: # If you dont find a target, report the centre of the image to keep the camera still
        self.targetX=0.5
        self.targetY=0.5


    # self.Cube_Cordinates.publish(Cube_Array_coridinates)


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
