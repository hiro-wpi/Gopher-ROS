# This function provides which room the robot is in given its coordinate.
def get_robot_location(x, y):
    '''
    params: coordinate x, coordinate y
    return: room number or corridor
    '''
    if (-6< x <-4):
        if (0< y <8):
            return "ROOM1"
        elif (-9< y <-3):
            return "ROOM2"
        else:
            return "CORRIDOR"

    if (-4<= x <-3):
        if (0< y <8):
            return "ROOM1"
        elif (1-(x+4) < y+10) and (y < -3):
            return "ROOM2"
        else:
            return "CORRIDOR"

    if (-3<= x <-2):
        if(0 < y) and (y-6 < 1-(x+3)):
            return "ROOM1"
        elif (-11< y <-3):
            return "ROOM2"
        else:
            return "CORRIDOR"

    elif (-2<= x <0):
        if (0< y <6):
            return "ROOM1"
        elif (-11< y <-3):
            return "ROOM2"
        else:
            return "CORRIDOR"

    elif (3< x <9):
        if (0<= y <9):
            return "ROOM3"
        elif (-6<= y <0)::
            return "ROOM4"
        elif (-12< y <-6)::
            return "ROOM5"
        else:
            return "CORRIDOR"

    else:
        return "CORRIDOR"
