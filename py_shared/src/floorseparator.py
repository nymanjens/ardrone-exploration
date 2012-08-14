""" settings """
Z_THRESHOLD = 300 # mm

""" functions """
def is_floor(x):
    return abs(x[2]) < Z_THRESHOLD

def is_wall(x):
    return x[2] > Z_THRESHOLD

