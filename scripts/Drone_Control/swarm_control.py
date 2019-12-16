import rospy
from Intro_Robotics_Final_Project.msg import QR, EdgeList, DroneCommand
#This class deals with controlling all drones

class SwarmController:
    drones = [] #List of DroneController objects, for now just 1
    qr_data = {"hasQR" : None, "centroid" : None, "value" : None}
    # edge_data
    '''
    [
        {
            "color" : String, # (orange, purple)
            "angle" : float,
            "centroid" : (int, int) # (x, y)
        }
    ]
    '''
    edge_data = {"edges" : []}
    # Graph Edges:
    '''
    [
        {
            color: string,
            v1: int,
            v2: int
        }
    ]
    '''    
    graph_edges = []

    CENTER_QR = 0
    DETERMINE_NEXT_LINE = 1
    NAVIGATE_TO_LINE = 2
    FOLLOW_LINE = 3
    MOVE_ONTO_LINE = 4
    LAND = 5

    current_state = 0
    current_edge = None

    def __init__(self):
        #Initialize pubs and subs
        #Publishers
        self.drone_command_pub = rospy.Publisher('/swarm/drone_command', DroneCommand, queue_size=1)
        #Subscribers
        self.qr_sub = rospy.Subscriber('/swarm/qr_code', QR, self.qr_callback)
        self.edges_sub = rospy.Subscriber('/swarm/edges', EdgeList, self.edge_callback)

        # sleep(1.0)

################################################################################
#Main methods and publishing methods

    #Loop to be run inside main script
    #DONE: Loops while core is running
    def run_node(self):
        while not rospy.is_shutdown():
            if self.current_state == self.CENTER_QR:
                self.center_qr()

            elif self.current_state == self.DETERMINE_NEXT_LINE:
                self.determine_next_line()

            elif self.current_state == self.NAVIGATE:
                self.navigate_to_line()

            elif self.current_state == self.FOLLOW_LINE:
                self.follow_line()

            elif self.current_state == self.MOVE_ONTO_LINE:
                self.move_onto_line()

            elif self.current_state == self.LAND:
                self.land_swarm()

    #Centers the drone over the QR code
    #DONE: Based on the centroid move the drone
    #TODO: Test this
    #NOTE: This will be within some range of error, right now 10 pixels
    def center_qr(self):
        center = [320, 184] #Not completely sure about this
        centroid = self.qr_data["centroid"]

        x_err = center[0]-centroid[0]
        y_err = center[1]-centroid[1]

        cmd = DroneCommand()
        while abs(x_err) > 10 and abs(y_err) > 10:
            #Determine the drone cmd
            cmd.drone_id = self.drones[0].ID #Note this would need to be changed for multiple drones
            #x movement
            cmd.cmd_type[0] = "x"
            cmd.intensity[0] = 0 #Will default to base intensity
            cmd.direction[0] = np.sign(x_err)
            #y movement
            cmd.cmd_type[0] = "y"
            cmd.intensity[0] = 0 #Will default to base intensity
            cmd.direction[0] = np.sign(y_err)

            #Issue drone commands
            self.drone_command_pub.publish(cmd)

        self.current_state = self.DETERMINE_NEXT_LINE

    #Determines next line to follow out of the vertex
    #TODO:
    def determine_next_line(self):
        # Add the currently visible lines to the graph
        add_edges_to_graph(edge_data)
        # If there is an unexplored edge out of the current vertex, switch to line following state
        for edge in edge_data:
            existingEdge = is_edge_in_graph(edge, graph_edges, qr_data["value"])
            if existingEdge["v2"] == None:
                current_state = self.follow_line
            else:
                # If there are no unexplored edges out of the current vertex, and
                current_state = self.LAND

    #Establishes the drone on a new line to follow
    #TODO:
    #NOTE:
    def move_onto_line(self):
        self.current_state = FOLLOW_LINE

    #Dispatches drone from a vertex to a edge
    #TODO: Select an edge leaving the vertex and go there
    def navigate_to_line(self):
        pass

    #Follows line untill it reaches vertex, adjusts as neccesary
    #TODO: Do this
    #NOTE:
    def follow_line(self):
        cmd = DroneCommand()
        while self.qr_data["hasQR"] == False:
            angle = self.current_edge["angle"]
            if np.abs(angle) > 15:
                cmd.drone_id = self.drones[0].ID

                #angular adjustment
                cmd.cmd_type[0] = "angular"
                cmd.intensity[0] = 0 #Will default to base intensity
                cmd.direction[0] = np.sign(angle)
                #move forward
                cmd.cmd_type[0] = "x"
                cmd.intensity[0] = 0 #Will default to base intensity
                cmd.direction[0] = 1

            self.drone_command_pub.publish(cmd)

        #Add the end vertex to the edge
        self.current_edge["v2"] = self.qr_data["value"]

        #Change state
        self.current_state = self.CENTER_QR

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

    # This helper funciton will add all of the edge colors in the edges array to the graph
    def add_edges_to_graph(self, edges):
        for edge in edges:
            if(is_edge_in_graph(edge, graph_edges, qr_data["value"]) == None):
                new_edge = {"color": edge["color"], "v1": qr_data["value"], v2: None}
                graph_edges.append(new_edge)

    # Returns None if not found, return the graph edge otherwise
    def is_edge_in_graph(self, edge, graph, qrValue):
        for g_edge in graph:
            if (g_edge["color"] == edge["color"] and (g_edge["v1"] == qrValue or g_edge["v2"] == qrValue)):
                return g_edge
        return None
            

################################################################################
#Callbacks

    def qr_callback(self, data):
        self.qr_data["hasQR"] = data.existing
        self.qr_data["hasQR"] = data.centroid
        self.qr_data["value"] = data.value

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
