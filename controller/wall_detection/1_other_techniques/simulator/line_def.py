""" definition of room: list of (start, end)-points [x,y] """
from numpy import array

selection = 'rect'

if selection == 'rect':
    # easy room (rect)
    lines = 1000 * array([
        ( [0, 0], [0, 9] ),
        ( [0, 9], [6, 9] ),
        ( [6, 9], [6, 0] ),
        ( [6, 0], [0, 0] ),
    ])
elif selection == 'room':
    # complex room (room)
    lines = 1000 * array([
        ( [0, 0], [0, 4] ),
        ( [0, 4], [3, 4] ),
        ( [3, 4], [3, 7] ),
        ( [3, 7], [0, 7] ),
        ( [0, 7], [0, 9] ),
        ( [0, 9], [6, 9] ),
        ( [6, 9], [6, 0] ),
        ( [6, 0], [0, 0] ),
    ])

