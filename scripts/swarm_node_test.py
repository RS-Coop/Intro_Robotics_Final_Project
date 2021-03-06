#!/usr/bin/env python
import rospy
from Drone_Control import SwarmController

#Initializes a node for a swarm
rospy.init_node('Swarm')

#Initializes a drone
swarm = SwarmController('test') #Different namespace so wont fly drone

#Handler for rospy shutdown
#Should not be necessary as we wont be flying
rospy.on_shutdown(swarm.failsafe)

print("Swarm ready")

#Runs loop
swarm.run_node()

print("Swarm landed")
