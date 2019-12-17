import rospy
import numpy as np
from Globals import Globals as G
from Intro_Robotics_Final_Project.msg import QR, EdgeList, DroneCommand

#This class deals with controlling all drones
class SwarmController:
    # Region definitions for image
    CENTER = G.BEBOP_CENTER
    CENTER_X_ERROR = G.QR_ERROR
    CENTER_Y_ERROR = G.QR_ERROR

    #Drones in swarm (NOTE: Right now just 1)
    drones = ['/bebop']
    #QR Code data from img processor
    qr_data = {"hasQR" : None, "centroid" : None, "value" : None}
    #Line data from img processor
    '''
    [
        {
            "color" : String, # (orange, purple)
            "angle" : float,
            "centroid" : (int, int) # (x, y)
        }
    ]
    '''
    edge_data = []

    #Edges in graph
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

    #Current state
    current_state = G.TAKEOFF
    #Current edge being followed
    current_graph_edge = None

    def __init__(self, namespace='/swarm'):
        #Publishers
        self.drone_command_pub = rospy.Publisher(namespace+'/drone_command', DroneCommand, queue_size=1)
        #Subscribers
        self.qr_sub = rospy.Subscriber(namespace+'/qr_code', QR, self.qr_callback)
        self.edges_sub = rospy.Subscriber(namespace+'/edges', EdgeList, self.edge_callback)

        # sleep(1.0)

################################################################################
#Main methods and publishing methods

    #Loop to be run inside main script
    #DONE: Loops while core is running
    #NOTE: Navigate to line not implemented
    def run_node(self):
        while not rospy.is_shutdown():
            if self.run_state():
                break

    # Run based off state, if state is land, return true
    def run_state(self):
        print("State:", self.current_state)
        if self.current_state == G.TAKEOFF:
            self.takeoff() #Center qr code
            return False
        elif self.current_state == G.CENTER_QR:
            self.center_qr() #Center qr code
            return False
        elif self.current_state == G.DETERMINE_NEXT_LINE:
            self.determine_next_line() #Choose a new line to follow
            return False
        #NOTE: Not yet being used
        elif self.current_state == G.NAVIGATE:
            self.navigate_to_line()
            return False
        elif self.current_state == G.FOLLOW_LINE:
            self.follow_line() #Follow current edge
            return False
        elif self.current_state == G.MOVE_ONTO_LINE:
            self.move_onto_line() #Move into position over new line
            return False
        elif self.current_state == G.LAND:
            self.land_swarm() #Land the drones
            return True

    #Centers the drone over the QR code
    #DONE: Based on the centroid move the drone
    #TODO: Test this
    #NOTE: This will be within some range of error, right now 10 pixels
    def center_qr(self):
        centroid = self.qr_data["centroid"]

        x_err = self.CENTER[0]-centroid[0]
        y_err = centroid[1]-self.CENTER[1]

        print("XERR:", x_err, "YERR:", y_err)
        print("", abs(x_err), ">", self.CENTER_X_ERROR, "or", abs(y_err), ">", self.CENTER_Y_ERROR)

        if abs(x_err) > self.CENTER_X_ERROR or abs(y_err) > self.CENTER_Y_ERROR:
            cmd = DroneCommand()
            #Determine the drone cmd
            cmd.drone_id = self.drones[0] #Note this would need to be changed for multiple drones
            #x movement
            cmd.cmd_type.append("x")
            cmd.intensity.append(0) #Will default to base intensity
            cmd.direction.append(np.sign(x_err))
            #y movement
            cmd.cmd_type.append("y")
            cmd.intensity.append(0) #Will default to base intensity
            cmd.direction.append(np.sign(y_err))

            #Issue drone commands
            self.drone_command_pub.publish(cmd)

        else:
            self.current_state = G.DETERMINE_NEXT_LINE

    #Determines next line to follow out of the vertex
    #TODO:
    def determine_next_line(self):
        # Add the currently visible lines to the graph
        self.add_edges_to_graph(self.edge_data)

        # If there is an unexplored edge out of the current vertex, switch to line following state
        for edge in self.edge_data:
            # Get the current edge in graph_edges
            existingEdge = self.get_edge_in_graph(edge, self.graph_edges, self.qr_data["value"])
            # If the current edge started at the current QR (has it for v1 instaed of v2) then explore it
            if existingEdge != None and existingEdge["v2"] == None:
                self.current_edge = existingEdge
                self.current_state = G.MOVE_ONTO_LINE
                return
        # If there are no unexplored edges out of the current vertex, and
        self.current_edge = None
        self.current_state = G.LAND

    #Establishes the drone on a new line to follow
    #TODO: Move the drone onto
    #NOTE:
    def move_onto_line(self):
        if self.qr_data["hasQR"] == True:
            cmd = DroneCommand()
            for edge in self.edge_data:
                if edge["color"] == self.current_edge["color"]:
                    angle = edge["angle"]
                    break

            if angle > 10:
                #angular adjustment
                cmd.cmd_type.append("angular")
                cmd.intensity.append(0) #Will default to base intensity
                cmd.direction.append(np.sign(angle))

            else:
                cmd.cmd_type.append("x")
                cmd.intensity.append(1) #Will default to base intensity
                cmd.direction.append(1)

        else:
            self.current_state = G.FOLLOW_LINE

    #Dispatches drone from a vertex to a edge
    #TODO: Select an edge leaving the vertex and go there
    def navigate_to_line(self):
        pass

    #Follows line untill it reaches vertex, adjusts as neccesary
    #TODO: Do this
    #NOTE:
    def follow_line(self):
        if self.qr_data["hasQR"] == False:
            cmd = DroneCommand()
            for edge in self.edge_data:
                if edge["color"] == self.current_edge["color"]:
                    angle = edge["angle"]
            if np.abs(angle) > G.ANGLE_BOUND:
                cmd.drone_id = self.drones[0]

                #angular adjustment
                cmd.cmd_type.append("angular")
                cmd.intensity.append(0) #Will default to base intensity
                cmd.direction.append(np.sign(angle))
                #move forward
                cmd.cmd_type.append("x")
                cmd.intensity.append(0) #Will default to base intensity
                cmd.direction.append(1)

            self.drone_command_pub.publish(cmd)

        else:
            #Add the end vertex to the edge
            self.update_v2(self.current_edge, self.graph_edges, self.current_edge["v1"],self.qr_data["value"])
            self.current_edge = None
            #Change state
            self.current_state = G.CENTER_QR

    # TODO
    # Has the drones take off!
    def takeoff(self):
        # Make the drones take off here: TODO
        # Change state
        self.current_state = G.CENTER_QR

    #Launches all drones in swarm
    #DONE: Launch all drones in drones list
    def launch_swarm(self):
        for drone in self.drones:
            drone.takeoff()
            drone.move_camera()

        sleep(2.0)

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
        try:
            for edge in edges:
                edgeFromGraph = self.get_edge_in_graph(edge, self.graph_edges, self.qr_data["value"])
                # If there is no matching edge in the graph:
                if(edgeFromGraph == None):
                    new_edge = {"color": edge["color"], "v1": self.qr_data["value"], "v2": None}
                    self.graph_edges.append(new_edge)
                    pass
        except:
            print("WARN: UNEXPECTED ISSUE IN add_edges_to_graph")

    # Returns None if not found, return the graph edge otherwise
    def get_edge_in_graph(self, edge, graph, qrValue):
        for g_edge in graph:
            if (g_edge["color"] == edge["color"] and (g_edge["v1"] == qrValue or g_edge["v2"] == qrValue)):
                return g_edge
        return None

    # Returns None if not found, return the graph edge otherwise
    def update_v2(self, edge, graph, v1, qrValue):
        for g_edge in graph:
            if (g_edge["color"] == edge["color"] and g_edge["v1"] == v1):
                g_edge["v2"] = qrValue
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

            newDict = {
                        "color" : data.colors[i],
                        "pos_avg" : {}
                      }

            newDict["pos_avg"]["O_TOP"] = (data.pos_avg[currentIndex], data.pos_avg[currentIndex+1])
            newDict["pos_avg"]["O_BOTTOM"] = (data.pos_avg[currentIndex+2], data.pos_avg[currentIndex+3])
            newDict["pos_avg"]["O_LEFT"] = (data.pos_avg[currentIndex+4], data.pos_avg[currentIndex+5])
            newDict["pos_avg"]["O_RIGHT"] = (data.pos_avg[currentIndex+6], data.pos_avg[currentIndex+7])
            newDict["pos_avg"]["I_TOP"] = (data.pos_avg[currentIndex+8], data.pos_avg[currentIndex+9])
            newDict["pos_avg"]["I_BOTTOM"] = (data.pos_avg[currentIndex+10], data.pos_avg[currentIndex+11])
            newDict["pos_avg"]["I_LEFT"] = (data.pos_avg[currentIndex+12], data.pos_avg[currentIndex+13])
            newDict["pos_avg"]["I_RIGHT"] = (data.pos_avg[currentIndex+14], data.pos_avg[currentIndex+15])

            self.edge_data.append(newDict)
            currentIndex+=16

################################################################################
#Tests
