#!/usr/bin/python
""" imports """
# std imports
#...
# ros imports
import roslib; roslib.load_manifest('controller'); import rospy
from std_msgs.msg import String
# custom imports
import utility

""" listeners """
class HybridDataCreator:
    publisher = None
    navdata = None
    diffnavdata = None
    slamdata = None
    diffslamdata = None
    hybriddata = None
    
    def __init__(self):
        self.hybriddata = {
            'x': 0,
            'y': 0,
            'h': 0,
            'theta': 0,
            'phi': 0,
            'psi': 0,
            'timestamp': 0,
            'confidence': 0,
            'num_found_points': 0,
        }
        self.publisher = rospy.Publisher('/ardrone/hybriddata', String)
    
    
    def navdataEvent(self, data):
        data = utility.decode_string_to_dict(data)
        # correct for other axis scheme
        for key in ('y', 'phi', 'psi'):
            data[key] *= -1
        # new data-step --> parse previous step (with or without slamdata)
        self.parseDataStep()
        # refresh navdata
        data['h'] = data['altitude']
        self.diffnavdata = self.diff(data, self.navdata)
        self.navdata = data
        # reset diffslamdata to detect future missing packet
        self.diffslamdata = None
    
    def slamdataEvent(self, data):
        data = utility.decode_string_to_dict(data)
        del data['rotmx']
        self.diffslamdata = self.diff(data, self.slamdata)
        self.slamdata = data
    
    @staticmethod
    def diff(dict1, dict2, check_time=True):
        """ calculate difference between 2 dicts """
        if not dict1 or not dict2:
            return None
        diff = {}
        for key in dict1.keys():
            diff[key] = dict1[key] - dict2[key]
        # check if diff is still relevant (same dataset)
        if check_time and diff['timestamp'] > 2: # 2 seconds
            print "Note: timestamps differ too much ({} s) --> new dataset".format(diff['timestamp'])
            return None
        return diff
    
    def parseDataStep(self):
        """ TMP """
        if self.navdata:
            self.navdata['confidence'] = 100
            self.navdata['num_found_points'] = 100
            data_str = ','.join('%s:%s' % (key,val) for (key,val) in self.navdata.items())
            self.publisher.publish(data_str)
        return
        """ END TMP """
        
        # no parsing before first navdata packet
        if not self.navdata:
            return
        # inital parse after first navdata packet
        elif not self.diffnavdata:
            # set first value of keys
            for key in ['x', 'y', 'h', 'phi', 'theta', 'psi', 'timestamp']:
                self.hybriddata[key] = self.navdata[key]
        # no slamdata received
        elif not self.diffslamdata:
            # set difference keys
            for key in ['x', 'y', 'h', 'psi']:
                self.hybriddata[key] += self.diffnavdata[key]
            # set absolute keys
            for key in ['theta', 'phi', 'timestamp']:
                self.hybriddata[key] = self.navdata[key]
            # set confidence & num found points
            self.hybriddata['confidence'] = 0 # because this is SLAM confidence, which is missing
            self.hybriddata['num_found_points'] = 0
        # navdata and slamdata present
        else:
            # get alpha (weight of slam data)
            confidence = self.slamdata['confidence']
            #MIN_THRESHOLD = .5
            #MAX_THRESHOLD = .85
            MIN_THRESHOLD = .5
            MAX_THRESHOLD = .8
            if not confidence or confidence < MIN_THRESHOLD:
                alpha = 0
            elif confidence > MAX_THRESHOLD:
                alpha = 1
            else:
                alpha = (confidence - MIN_THRESHOLD) / (MAX_THRESHOLD - MIN_THRESHOLD)
            # set difference keys
            for key in ['x', 'y', 'h', 'psi']:
                dnav = self.diffnavdata[key]
                #dslam = self.diffslamdata[key] # old
                dslam = self.slamdata[key] - self.hybriddata[key]
                dhybrid = (1-alpha) * dnav + alpha * dslam
                self.hybriddata[key] += dhybrid
            # set absolute keys
            for key in ['theta', 'phi', 'timestamp']:
                self.hybriddata[key] = self.navdata[key]
            # set confidence & num_found_points
            self.hybriddata['confidence'] = confidence
            self.hybriddata['num_found_points'] = self.slamdata['num_found_points']
            
            """ !! DEBUG """
            from numpy import isnan
            if isnan(self.hybriddata['x']):
                print "nan in hybriddata!"
                rospy.signal_shutdown("")
            """ !! END DEBUG """
        
        # publish hybriddata
        data_str = ','.join('%s:%s' % (key,val) for (key,val) in self.hybriddata.items())
        self.publisher.publish(data_str)
        #print "[{:.2f}] publishing".format(self.hybriddata['timestamp'])

""" main ros code """
hybridDataCreator = HybridDataCreator()
rospy.init_node('hybriddata_creator')
rospy.Subscriber("/ardrone/navdata/elf", String, hybridDataCreator.navdataEvent)
rospy.Subscriber("/ardrone/slamdata", String, hybridDataCreator.slamdataEvent)
rospy.spin()
print "\n\n<done>"

