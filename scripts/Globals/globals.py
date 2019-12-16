import numpy as np

class Globals:
    #State definitions
    CENTER_QR = 0
    DETERMINE_NEXT_LINE = 1
    NAVIGATE_TO_LINE = 2
    FOLLOW_LINE = 3
    MOVE_ONTO_LINE = 4
    LAND = 5

    #Error for QR BB
    QR_ERROR = 10
    
    #Angle bound to determine if theta needs correcting
    ANGLE_BOUND = 15

    # Line Colors
    ORANGE = "orange"
    PURPLE = "purple"
    # Color ranges
    LINE_COLORS = {
        ORANGE:[np.array([5,100,150]), np.array([15,255,255])],
        PURPLE:[np.array([275,100,150]), np.array([285,255,255])]
    }
