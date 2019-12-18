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
    #Color of current edge being followed
    current_edge_color = None
    current_qr_code = None

    def __init__(self, namespace='/swarm'):
        #Publishers
        self.drone_command_pub = rospy.Publisher(namespace+'/drone_command', DroneCommand, queue_size=1)
        #Subscribers
        self.qr_sub = rospy.Subscriber('/swarm/qr_code', QR, self.qr_callback)
        self.edges_sub = rospy.Subscriber('/swarm/edges', EdgeList, self.edge_callback)

        rospy.sleep(2.0)

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
        print("State: ", self.current_state)
        print("QR: ", self.qr_data)
        if self.current_state == G.TAKEOFF:
            self.launch_swarm() #Center qr code
            return False
        elif self.current_state == G.CENTER_QR:
            self.center_qr() #Center qr code
            return False
        elif self.current_state == G.DETERMINE_NEXT_LINE:
            self.determine_next_line() #Choose a new line to follow
            return False
        #NOTE: Not yet being used
        elif self.current_state == G.NAVIGATE_TO_LINE:
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
        if self.qr_data["hasQR"] == True:
            current_qr_code = self.qr_data["value"]
            centroid = self.qr_data["centroid"]

            x_err = self.CENTER[0]-centroid[0] #pos means forward
            y_err = self.CENTER[1]-centroid[1]  #pos means left

            # print("XERR:", x_err, "YERR:", y_err)
            # print("", abs(x_err), ">", self.CENTER_X_ERROR, "or", abs(y_err), ">", self.CENTER_Y_ERROR)

            if abs(x_err) > self.CENTER_X_ERROR or abs(y_err) > self.CENTER_Y_ERROR:
                cmd = DroneCommand()
                #Determine the drone cmd
                cmd.drone_id = self.drones[0] #Note this would need to be changed for multiple drones
                #x movement
                cmd.cmd_type.append(G.X)
                cmd.intensity.append(0) #Will default to base intensity
                cmd.direction.append(np.sign(x_err))
                #y movement
                cmd.cmd_type.append(G.Y)
                cmd.intensity.append(0.1) #Will default to base intensity
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
        # print("Graph:", self.graph_edges)

        # If there is an unexplored edge out of the current vertex, switch to line following state
        for edge in self.edge_data:
            # Get the current edge in graph_edges
            existingEdge = self.get_edge_in_graph(edge["color"], self.graph_edges, self.current_qr_code)
            # If the current edge started at the current QR (has it for v1 instaed of v2) then explore it
            # print(existingEdge)
            if existingEdge != None and existingEdge["v2"] == None:
                self.current_edge_color = existingEdge["color"]
                self.current_state = G.MOVE_ONTO_LINE
                return
        # If there are no unexplored edges out of the current vertex, and
        self.current_edge_color = None
        self.current_state = G.LAND

    #Establishes the drone on a new line to follow
    #TODO: Move the drone onto
    #NOTE:
    def move_onto_line(self):
        if self.qr_data["hasQR"] == True:
            cmd = DroneCommand()

            #If the line is not vertical
            if self.is_line_vertical == False:
                print("line not vertical")
                #Rotate to get the line vertical
                centroid, angle = self.get_line_pose(self.current_edge_color)

                cmd.cmd_type.append("angular")
                cmd.intensity.append(0) #Will default to base intensity
                #Do some stuff here to turn
                x_err = self.CENTER[0] - centroid[0]
                cmd.direction.append(np.sign(x_err)) #Not sure about this agrument

            #If the line is not centered
            elif self.is_line_centered == False:
                print("line not centered")
                #Shift left or right to center line
                centroid, angle = self.get_line_pose(self.current_edge_color)
                #We should just care about x error
                x_err = self.CENTER[1]-centroid[1] #pos means left

                cmd.cmd_type.append(G.Y)
                cmd.intensity.append(0.1) #Will defualt to base intensity
                cmd.direction.append(np.sign(x_err))

            #If the line is vertical and centered
            else:
                print("line vertical and centered")
                #Move forward
                cmd.cmd_type.append(G.X)
                cmd.intensity.append(0) #Will default to base intensity
                cmd.direction.append(1)
                # #Change state
                # self.current_state = G.FOLLOW_LINE

            self.drone_command_pub.publish(cmd)

        else:
            self.current_state = G.FOLLOW_LINE

    #Dispatches drone from a vertex to a edge
    #TODO: Select an edge leaving the vertex and go there
    def navigate_to_line(self):
        pass

    #Follows line untill it reaches vertex, adjusts as neccesary
    #TODO: What data do I have for moving, then fix movement commands
    #NOTE: Not yet fully implemented
    def follow_line(self):
        if self.qr_data["hasQR"] == False:
            cmd = DroneCommand()
            #If the line is not centered
            if self.is_line_centered == False:
                #Shift left or right to center line
                centroid, angle = self.get_line_pose(self.current_edge_color)
                #We should just care about x error
                x_err = self.CENTER[1]-centroid[1] #pos means left

                cmd.cmd_type.append(G.Y)
                cmd.intensity.append(0.1) #Will defualt to base intensity
                cmd.direction.append(np.sign(x_err))

            #If the line is not vertical
            elif self.is_line_vertical == False:
                #Rotate to get the line vertical
                centroid, angle = self.get_line_pose(self.current_edge_color)

                cmd.cmd_type.append(G.THETA)
                cmd.intensity.append(0.1) #Will default to base intensity
                cmd.direction.append(np.sign(angle)) #Not sure about this agrument

            #If the line is vertical and centered
            else:
                #Move forward
                cmd.cmd_type.append(G.X)
                cmd.intensity.append(0) #Will default to base intensity
                cmd.direction.append(1)

            self.drone_command_pub.publish(cmd)

        else:
            #Add the end vertex to the edge
            current_edge = self.get_edge_pose(self.current_edge_color)
            self.update_v2(self.current_edge_color, self.graph_edges, current_edge["v1"], self.qr_data["value"])
            self.current_edge_color = None
            #Change state
            self.current_state = G.CENTER_QR
            self.current_qr_code = self.qr_data["value"]

    # Return true if the line is veritical in the image with a certain error
    def is_line_vertical(self):
        centroid, angle = get_line_pose(line_color)

        if abs(angle) > self.ANGLE_ERROR:
            return True

        return False

    # Return true if the line is centered in the image with a certain error
    def is_line_centered(self):
        centroid, angle = get_line_pose(line_color)

        x_err = np.abs(self.CENTER[1]-centroid[1])

        if x_err > self.CENTER_X_ERROR:
            return True

        return False

    # Returns centroid, angle
    def get_line_pose(self, line_color):
        currentLine = None

        for i in range(0, len(self.edge_data)):
            if(self.edge_data[i]['color'] == line_color):
                currentLine = self.edge_data[i]

        # print(currentLine)
        zone_names_outer = ["outer top", "outer bottom", "outer left", "outer right"]
        zone_names_inner = ["inner top", "inner bottom", "inner left", "inner right"]

        if currentLine == None:
            return None, None
        else:
            firstPoint = (None, None)
            secondPoint = (None, None)

            for i in zone_names_outer:
                # print(currentLine["pos_avgs"][i])
                # print(currentLine["pos_avgs"][i] != (None, None))

                if((currentLine["pos_avgs"][i] != (None, None)) and (firstPoint == (None,None))):
                    firstPoint = currentLine["pos_avgs"][i]
                    # print("firstPoint", i)
                elif((currentLine["pos_avgs"][i] != (None,None)) and (secondPoint == (None,None))):
                    secondPoint = currentLine["pos_avgs"][i]
                    # print("secondPoint", i)
                    break

            #A second point was not found in the outer ring, checking inner ring
            if(firstPoint != (None,None) and secondPoint == (None,None)):
                for i in zone_names_inner:
                    if(currentLine["pos_avgs"][i] != (None,None)):
                        secondPoint = currentLine["pos_avgs"][i]
                        # print("secondPoint", i)
                        break

            #If either point is not found, not enough data to calculate centroid and angle, return None
            if(firstPoint == (None,None) or secondPoint == (None,None)):
                return None, None
            else:
                centroid = ((firstPoint[0] + secondPoint[0]) / 2 , (firstPoint[1] + secondPoint[1]) / 2)
                x = secondPoint[0] - firstPoint[0]
                y = secondPoint[1] - firstPoint[1]
                theta = np.degrees(np.arctan2(x , y))
                return centroid, theta

        return None, None

    #Launches all drones in swarm
    #DONE: Launch all drones in drones list
    def launch_swarm(self):
        self.current_state = G.CENTER_QR
        self.current_qr_code = self.qr_data["value"]

        for drone in self.drones:
            print(drone)
            cmd = DroneCommand()
            cmd.drone_id = drone
            cmd.cmd_type.append(G.TAKEOFF)
            self.drone_command_pub.publish(cmd)

        print('Launched')
        self.current_state = G.CENTER_QR

    #Lands all drones in swarm
    #DONE: Land all drones in drones list
    def land_swarm(self):
        for drone in self.drones:
            cmd = DroneCommand()
            cmd.drone_id = drone
            cmd.cmd_type.append(G.LAND)
            self.drone_command_pub.publish(cmd)

        print('Landed')

    #Failsafe
    def failsafe(self):
        for drone in self.drones:
            cmd = DroneCommand()
            cmd.drone_id = drone
            cmd.cmd_type.append(G.EMERGENCY)
            self.drone_command_pub.publish(cmd)

        print('Failed Safely')

    # This helper funciton will add all of the edge colors in the edges array to the graph
    def add_edges_to_graph(self, edges):
        try:
            for edge in edges:
                edgeFromGraph = self.get_edge_in_graph(edge["color"], self.graph_edges, self.current_qr_code)
                # If there is no matching edge in the graph:
                if(edgeFromGraph == None):
                    # if self.current_qr_code != 0:
                    #     new_edge = {"color": edge["color"], "v1": self.current_qr_code, "v2": None}
                    # else:
                    #     new_edge = {"color": edge["color"], "v1": self.current_qr_code, "v2": None}
                    new_edge = {"color": edge["color"], "v1": self.current_qr_code, "v2": None}
                    self.graph_edges.append(new_edge)
                    pass
        except:
            print("WARN: UNEXPECTED ISSUE IN add_edges_to_graph")

    # Returns None if not found, return the graph edge otherwise
    def get_edge_in_graph(self, edge_color, graph, qrValue):
        for g_edge in graph:
            if (g_edge["color"] == edge_color and (g_edge["v1"] == qrValue or g_edge["v2"] == qrValue)):
                return g_edge
        return None

    # Returns None if not found, return the graph edge otherwise
    def update_v2(self, edge_color, graph, v1, qrValue):
        for g_edge in graph:
            if (g_edge["color"] == edge_color and g_edge["v1"] == v1):
                g_edge["v2"] = qrValue
        return None

    #
    def get_edge_pose(self, edge_color):
        for g_edge in self.graph_edges:
            if (g_edge["color"] == edge_color):
                return g_edge
        return None



################################################################################
#Callbacks

    def qr_callback(self, data):
        self.qr_data["hasQR"] = data.existing
        self.qr_data["centroid"] = data.centroid
        self.qr_data["value"] = data.value

    def edge_callback(self, data):
        currentIndex = 0
        for i in range(0, len(data.colors), 16):

            newDict = {
                        "color" : data.colors[i],
                        "pos_avg" : {
                                    'O_TOP' : (None,None),
                                    'O_BOTTOM' : (None,None),
                                    'O_LEFT' : (None,None),
                                    'O_RIGHT' : (None,None),
                                    'I_TOP' : (None,None),
                                    'I_BOTTOM' : (None,None),
                                    'I_LEFT' : (None,None),
                                    'I_RIGHT' : (None,None)
                                    }
                      }

            if (data.pos_avgs[currentIndex], data.pos_avgs[currentIndex+1]) == (0,0):
                newDict["pos_avg"]["O_TOP"] = (data.pos_avgs[currentIndex], data.pos_avgs[currentIndex+1])
            if (data.pos_avgs[currentIndex+2], data.pos_avgs[currentIndex+3]) == (0,0):
                newDict["pos_avg"]["O_BOTTOM"] = (data.pos_avgs[currentIndex+2], data.pos_avgs[currentIndex+3])
            if (data.pos_avgs[currentIndex+4], data.pos_avgs[currentIndex+5]) == (0,0):
                newDict["pos_avg"]["O_LEFT"] = (data.pos_avgs[currentIndex+4], data.pos_avgs[currentIndex+5])
            if (data.pos_avgs[currentIndex+6], data.pos_avgs[currentIndex+7]) == (0,0):
                newDict["pos_avg"]["O_RIGHT"] = (data.pos_avgs[currentIndex+6], data.pos_avgs[currentIndex+7])
            if (data.pos_avgs[currentIndex+8], data.pos_avgs[currentIndex+9]) == (0,0):
                newDict["pos_avg"]["I_TOP"] = (data.pos_avgs[currentIndex+8], data.pos_avgs[currentIndex+9])
            if (data.pos_avgs[currentIndex+10], data.pos_avgs[currentIndex+11]) == (0,0):
                newDict["pos_avg"]["I_BOTTOM"] = (data.pos_avgs[currentIndex+10], data.pos_avgs[currentIndex+11])
            if (data.pos_avgs[currentIndex+12], data.pos_avgs[currentIndex+13]) == (0,0):
                newDict["pos_avg"]["I_LEFT"] = (data.pos_avgs[currentIndex+12], data.pos_avgs[currentIndex+13])
            if (data.pos_avgs[currentIndex+14], data.pos_avgs[currentIndex+15]) == (0,0):
                newDict["pos_avg"]["I_RIGHT"] = (data.pos_avgs[currentIndex+14], data.pos_avgs[currentIndex+15])

            self.edge_data.append(newDict)
            currentIndex+=16

################################################################################
#Tests
