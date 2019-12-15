import rospy
from Intro_Robotics_Final_Project.msg import QR, EdgeList, DroneCommand
#This class deals with controlling all drones
class SwarmController:
    drones = [] #List of DroneController objects, for now just 1
    qr_data = {"hasQR" : None, "centroid" : None}
    edge_data = {"edges" : []}

    def __init__(self):
        #Initialize pubs and subs
        #Publishers
        drone_command_pub = rospy.Publisher('/swarm/drone_command', DroneCommand, queue_size=1)
        #Subscribers
        qr_sub = rospy.Subscriber('/swarm/qr_code', QR, self.qr_callback)
        edges_sub = rospy.Subscriber('/swarm/edges', EdgeList, self.edge_callback)

        sleep(1.0)

################################################################################
#Main methods and publishing methods

    #Loop to be run inside main script
    #DONE: Loops while core is running
    def run_node(self):
        while not rospy.is_shutdown():
            pass

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

    def qr_callback(self, data):
        self.qr_data["hasQR"] = data["hasQR"]
        self.qr_data["hasQR"] = data["centroid"]

    def edge_callback(self, data):
        currentIndex = 0
        for i in range(0, len(data.colors)):
            self.edge_data["edges"].append({
                    "color" : data.colors[i], 
                    "angle" : data.angle[i],
                    "centroid" : (currentIndex, currentIndex+1)
                })
            currentIndex+=2

################################################################################
#Tests
