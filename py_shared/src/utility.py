""" imports """
# std imports
from numpy import isnan, array
from threading import Thread
# ros imports
import rospy
from rospy.topics import Publisher
from std_msgs.msg import String

def decode_string_to_dict(string_obj):
    """ string format: 'x:2.0,y:3.0,z:4.0' """
    s = string_obj.data.strip(',')
    tuples = []
    for item in s.split(','):
        tup = item.split(':')
        if tup[0] == 'rotmx':
            pass # exception: rotmx is not a float
        else:
            tup[1] = float(tup[1])
            if isnan(tup[1]):
                print "[utility.py:decode_string_to_dict()] Warning: {} = NaN".format(tup[0])
                print "                                     (in {})".format(s)
        tuples.append(tup)
    return dict(tuples)

def encode_array_to_string(arr):
    """ string format: '4.22,3.69,9.24|0,1.0,2.4589|' """
    return '|'.join(','.join(str(x) for x in p) for p in arr)

def decode_string_to_array(string_obj):
    """ string format: '4.22,3.69,9.24|0,1.0,2.4589|' """
    s = string_obj.data.strip('|')
    arr = s.split('|')
    for i, point in enumerate(arr):
        arr[i] = [float(x) for x in point.split(',')]
    return arr

def encode_tuplearray_to_string(arr):
    """ string format: '1.0,2.0;3,4|5,6;7,8' """
    return '|'.join(';'.join((','.join(str(x) for x in p)) for p in tup) for tup in arr)

def decode_string_to_tuplearray(string_obj):
    """ string format: '4.22,3.69,9.24|0,1.0,2.4589|' """
    s = string_obj.data.strip('|')
    if s.strip() == "":
        return []
    arr = s.split('|')
    for i, tup in enumerate(arr):
        tup = tup.split(';')
        for j, point in enumerate(tup):
            tup[j] = array([float(x) for x in point.split(',')])
        arr[i] = tup
    return arr


class StatePublisher(Publisher):
    state = None
    
    def __init__(self, topic):
        super(StatePublisher, self).__init__(topic, String)
    
    def publish(self, state):
        if not self.state or self.state != state:
            self.state = state
            super(StatePublisher, self).publish(state)

class StateSubscriber:
    state = None
    
    def __init__(self, topic, defaultstate=None):
        self.state = defaultstate
        rospy.Subscriber(topic, String, self.updateState)
    
    def updateState(self, string_obj):
        self.state = string_obj.data

class MaxOneProcess:
    """ make sure at most one process is running
    
    example usage: rospy.Subscriber("/map/points", String, MaxOneProcess(process_map_points).startProcess)
    
    """
    function = None
    thread = None
    
    def __init__(self, function):
        self.function = function
    
    def startProcess(self, *args):
        # enforce max one process
        if self.thread and self.thread.is_alive():
            return
        # start thread
        self.thread = Thread(target=self.function, args=args)
        self.thread.start()

class Minimizer:
	argmin = None
	mincost = float('inf')
	def add(self, arg, cost):
		if cost < self.mincost:
			self.mincost = cost
			self.argmin = arg



