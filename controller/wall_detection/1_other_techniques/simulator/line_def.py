""" definition of room: list of (start, end)-points [x,y] """
from numpy import array

selection = 'room'

lines_list = {
    'rect': 1000 * array([
        ( [0, 0], [0, 9] ),
        ( [0, 9], [6, 9] ),
        ( [6, 9], [6, 0] ),
        ( [6, 0], [0, 0] ),
    ]),
    'room': 1000 * array([
        ( [0, 0], [0, 4] ),
        ( [0, 4], [3, 4] ),
        ( [3, 4], [3, 7] ),
        ( [3, 7], [0, 7] ),
        ( [0, 7], [0, 9] ),
        ( [0, 9], [6, 9] ),
        ( [6, 9], [6, 0] ),
        ( [6, 0], [0, 0] ),
    ]),
}

lines = lines_list[selection]

