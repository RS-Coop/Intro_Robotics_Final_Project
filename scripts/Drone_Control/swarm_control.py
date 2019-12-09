import rospy

#This class deals with controlling all drones
class SwarmController:
    drones = [] #List of DroneController objects, for now just 1

    def __init__(self):
        #Initialize pubs and subs
        #Issues drone commands
        drone_command_pub = rospy.Publisher('/swarm/drone_command', DroneCommand, queue_size=1)

        #NOTE: Not sure of the flow of info here
        #will need to obtain information to follow line
        #will also need to get info from map
        '''
        drone_loc_sub = rospy.Subscriber('multi_agent/current_locations', Odom_Tag, queue_size=1)
        location_req_sub = rospy.Subscriber('multi_agent/location_requests', Odometry, queue_size=1)
        '''

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

    def image_processor_callback(self):
        pass

    def map_callback(self):
        pass

################################################################################
#Tests
