import rospy
from Intro_Robotics_Final_Project import QR, EdgeList, DroneCommand
#This class deals with controlling all drones
class SwarmController:
    drones = [] #List of DroneController objects, for now just 1

    def __init__(self):
        #Initialize pubs and subs
        #Publishers
        drone_command_pub = rospy.Publisher('/swarm/drone_command', DroneCommand, queue_size=1)
        #Subscribers
        qr_sub = rospy.Subscriber('/swarm/qr_code', QR, qr_callback())
        edges_sub = rospy.Subscriber('/swarm/edges', EdgeList, edge_callback())

        sleep(1.0)

################################################################################
#Main methods and publishing methods

    #Loop to be run inside main script
    #DONE: Loops while core is running
    def run_node(self):
        while not rospy.is_shutdown():
            continue

    #Dispatches drone from a vertex to a edge
    #TODO: Select an edge leaving the vertex and go there
    def dispatch_drone(self):
        pass

    #Drone follows line untill it reaches vertex
    #TODO: Follow line
    def follow_line(self):
        pass

    #Launches all drones in swarm
    #DONE: Launch all drones in drones list
    def launch_swarm(self):
        for drone in self.drones:
            drone.takeoff()

    #Lands all drones in swarm
    #DONE: Land all drones in drones list
    def land_swarm(self):
        for drone in self.drones:
            drone.land()

    #Failsafe
    def failsafe(self):
        for drone in self.drones:
            drone.failsafe()

################################################################################
#Callbacks

    def qr_callback(self):
        pass

    def edge_callback(self):
        pass

################################################################################
#Tests
