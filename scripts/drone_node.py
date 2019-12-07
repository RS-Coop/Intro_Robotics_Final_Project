#!/usr/bin/env python
import rospy
from Drone_control import DroneController

Initializes a node for a drone
rospy.init('Drone1')

drone = DroneController()

drone.run_node()
