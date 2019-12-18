#!/usr/bin/env python
import rospy
from Drone_Control import SwarmController

#Initializes a node for a swarm
rospy.init_node('Swarm')

#Initializes a drone
swarm = SwarmController()

#Handler for rospy shutdown
rospy.on_shutdown(swarm.failsafe)

print("Swarm ready")

#Runs loop
swarm.run_node()

print("Swarm landed")
