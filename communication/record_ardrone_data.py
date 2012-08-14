#!/usr/bin/python

""" settings """
DEFAULT_REPLAY_TYPENUM = 0

""" imports """
# std imports
import sys, copy, argparse, os
import numpy as np
# ROS imports
import roslib; roslib.load_manifest('communication'); import rospy
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from std_msgs.msg import String
# custom imports
from saveloadlib import *
import utility, saveloadlib

class NavdataRecorder:
    cache = None
    folder = None
    
    def __init__(self, folder):
        self.cache = []
        self.folder = folder
    
    def __del__(self):
        if self.cache:
            # init
            saver = SaveLoadLib('{}/navdata.pkl'.format(self.folder))
            # first, check if folder exists
            folder = os.path.dirname(saver.fpath)
            if not os.path.exists(folder):
                os.makedirs(folder)
            # print info
            print "\nNavdataRecorder: saving {} frames of navdata".format(len(self.cache))
            # actual save
            saver.save('navdata', self.cache)
        
    def save(self, data):
        data = utility.decode_string_to_dict(data)
        self.cache.append(data)
        print "saved navdata #{}".format(len(self.cache)-1)

class VideoRecorder:
    typenum = None
    saveloadlib = None

    def __init__(self, folder, typenum=0):
        self.typenum = typenum
        self.saveloadlib = VideoSaveLoadLib("{}/video{}".format(folder, typenum))
    
    def save(self, data):
        self.saveloadlib.saveFrame(data)
        print "saved video{} frame #{}".format(self.typenum, len(self.saveloadlib.cached_vid_metadata)-1)

""" parse commmand line args """
parser = argparse.ArgumentParser()
parser.add_argument('dataset', default=['recorded'], nargs='*',)
args = parser.parse_args()
FOLDER = 'dataset_new_' + args.dataset[0] if args.dataset[0] != 'recorded' else 'recorded'

""" create recorders (necessary because __del__ has to be called at END) """
navdatarecorder = NavdataRecorder(FOLDER)
video0recorder = VideoRecorder(FOLDER, 0)
video1recorder = VideoRecorder(FOLDER, 1)
video2recorder = VideoRecorder(FOLDER, 2)

""" ROS init """
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/ardrone/navdata/elf", String, navdatarecorder.save)
rospy.Subscriber("/ardrone_video0/image", Image, video0recorder.save)
rospy.Subscriber("/ardrone_video1/image", Image, video1recorder.save)
rospy.Subscriber("/ardrone_video2/image", Image, video2recorder.save)
rospy.spin()
print "\n\n<done>\n"



