import rospy
#This class deals with controlling the Sawyer arm
class ArmController:

    #Initiate pubs and subs
    def __init__(self, namespace):
        self.ID = namespace

        #pubs

        #subs
