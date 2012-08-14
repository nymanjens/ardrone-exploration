#!/usr/bin/python

""" imports """
# std import
import copy, time, argparse
import numpy as np
from threading import Thread, Event
# ROS imports
import roslib; roslib.load_manifest('communication'); import rospy
# custom imports
from libcom.publishlib import Publisher
from libardrone.spacebarcatcher import SpacebarCatcher
from saveloadlib import *

""" player threads """
class PlayerThread(Thread):
    data = None
    verbose = None
    N = 0
    timeDeltas = None
    finishedEvent = None
    continueEvent = None
    
    def __init__(self, data, firstLastTimestamp, continueEvent, verbose=False, **kwargs):
        # init vars
        self.data = data
        self.verbose = verbose
        self.N = len(data)*2 # *2 because smooth repeat doubles data
        # init child settings
        self.initializeSettings(**kwargs)
        # enable smooth repeat (in while loop)
        self.data = self.smoothenData(self.data)
        # generate time data
        self.generateTimeDeltas(firstLastTimestamp)
        # set events
        self.continueEvent = continueEvent
        self.finishedEvent = Event()
        # create thread
        Thread.__init__(self)
        self.start()
    
    def initializeSettings(self, **kwargs):
        pass
    
    @staticmethod
    def getFirstLastTimestampFromThreads(threadsAndData):
        firstStamps = []
        lastStamps = []
        for threadClass, data, kwargs in threadsAndData:
            firstStamps.append(threadClass.getFirstFrameTimestamp(data))
            lastStamps.append(threadClass.getLastFrameTimestamp(data))
        return min(firstStamps), max(lastStamps)
    
    @classmethod
    def getFirstFrameTimestamp(cls, data):
        return cls.getTimestampFromFrame(data[0])
    
    @classmethod
    def getLastFrameTimestamp(cls, data):
        return cls.getTimestampFromFrame(data[-1])

    def generateTimeDeltas(self, (firstTimestamp, lastTimestamp)):
        tprev = firstTimestamp
        self.timeDeltas = []
        for i, frame in enumerate(self.data):
            # get deltaT
            t = self.getTimestampFromFrame(frame)
            deltaT = abs(t - tprev)
            # exception: last frame of batch
            if i == self.N/2:
                deltaT = lastTimestamp-tprev
            # save deltaT
            self.timeDeltas.append(deltaT)
            tprev = t
    
    @staticmethod
    def smoothenData(data):
        rep = copy.copy(data)
        rep.reverse()
        return data + rep
    
    def run(self):
        tic = toc = 0
        while True:
            for i, deltaT, frame in zip(range(self.N), self.timeDeltas, self.data):
                # syncronize with other threads
                if i % (self.N/2) == 0:
                    self.finishedEvent.set()
                    self.continueEvent.wait()
                    self.finishedEvent.clear()
                    tic = toc = 0
                # check if script is shutting down
                if rospy.is_shutdown():
                    self.finishedEvent.set()
                    return
                # sleep
                sleeptime = deltaT - (toc-tic)
                if sleeptime < 0:
                    print "Warning: sleeptime = {}".format(sleeptime)
                rospy.sleep(max(0, sleeptime))
                # publish frame
                tic = time.time()
                self.publishFrame(frame)
                # output
                if self.verbose:
                    print "[i={}, t={}] {}: publishing frame".format(
                        i, self.getTimestampFromFrame(frame), self.__class__.__name__)
                toc = time.time()
    
    @staticmethod
    def getTimestampFromFrame(frame):
        raise NotImplementedError('abstract method')
    
    def publishFrame(self, frame):
        raise NotImplementedError('abstract method')

class NavdataPlayerThread(PlayerThread):
    @staticmethod
    def getTimestampFromFrame(frame):
        return frame['timestamp']
    
    def publishFrame(self, frame):
        # check spacebar pressed
        if MODIFY_SPACEBAR_PRESSED:
            frame['spacebar_pressed'] = 1 if spacebarCatcher.getSpacebarPressed() else 0
        else:
            if frame['spacebar_pressed']:
                print "[replayer] spacebar_pressed"
            # correct timestamp
            frame['timestamp'] = time.time()
        # publish
        publisher.publish_elf(frame)
        
class VideoPlayerThread(PlayerThread):
    vid_type = 0
    def initializeSettings(self, vid_type=0):
        self.vid_type = vid_type
    
    @staticmethod
    def getTimestampFromFrame(frame):
        return frame.header.stamp.to_time()
    
    def publishFrame(self, frame):
        # correct timestamp
        frame.header.stamp = rospy.Time.now()
        # publish
        publisher.pub_vid_data[self.vid_type].publish(frame)
        
""" parse commmand line args """
parser = argparse.ArgumentParser()
parser.add_argument('dataset', default=['sumo'], nargs='*',)
parser.add_argument('-s', '--modifyspacebar', action='store_true')
parser.add_argument('-p', '--plot', action='store_true')
args = parser.parse_args()
FOLDER = 'dataset_' + args.dataset[0] if args.dataset[0] != 'recorded' else 'recorded'
MODIFY_SPACEBAR_PRESSED = args.modifyspacebar
PLOT_TIMESTAMPS = args.plot

""" ROS init """
rospy.init_node("replay_ardrone_data")
publisher = Publisher(elf_video=True)

""" load frames """
navdataframes = SaveLoadLib(FOLDER + '/navdata.pkl').load(silent_fail=True)
vid0frames = VideoSaveLoadLib.loadFrames(FOLDER + '/video0')
vid1frames = VideoSaveLoadLib.loadFrames(FOLDER + '/video1')
vid2frames = VideoSaveLoadLib.loadFrames(FOLDER + '/video2')

""" create navdata if missing """
if not navdataframes:
    video = vid0frames if vid0frames else vid1frames
    navdataframes = []
    for frame in video:
        timestamp = VideoPlayerThread.getTimestampFromFrame(frame)
        navdataframes.append({
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
            'phi': 0.0, 'psi': 0.0, 'theta': -90.0,
            'timestamp': timestamp,
            'altitude': 1400.0,
            'spacebar_pressed': 0.0,
        })

""" create threads """
threadsAndData = [
    [NavdataPlayerThread, navdataframes, {}],
    [VideoPlayerThread, vid0frames, {'vid_type': 0}],
    [VideoPlayerThread, vid1frames, {'vid_type': 1}],
#TMP#    [VideoPlayerThread, vid2frames, {'vid_type': 2}],
]
# filter out non-existing video's
threadsAndData = [ entry for entry in threadsAndData if entry[1] ]
# get first and last timestamp
firstLastTimestamp = PlayerThread.getFirstLastTimestampFromThreads(threadsAndData)
# create continueEvent
continueEvent = Event()
# create spacebarCatcher
if MODIFY_SPACEBAR_PRESSED:
    spacebarCatcher = SpacebarCatcher()
# create (and start) threads
threads = []
for i, (threadClass, data, kwargs) in enumerate(threadsAndData):
    threads.append(threadClass(data, firstLastTimestamp, continueEvent, verbose=False, **kwargs))

""" plot time course """
if PLOT_TIMESTAMPS:
    import pylab as pl
    for i, (threadClass, data, kwargs) in enumerate(threadsAndData):
        times = [threadClass.getTimestampFromFrame(frame) for frame in data]
        pl.plot(times)
    pl.show()

print "started playing"

""" synchronize threads """
while not rospy.is_shutdown():
    for thread in threads:
        if rospy.is_shutdown(): break
        while not rospy.is_shutdown():
            if thread.finishedEvent.wait(timeout=0.05):
                break
    if rospy.is_shutdown(): break
    continueEvent.set()
    rospy.sleep(0.2)
    continueEvent.clear()
    rospy.sleep(0.2)

""" save modified data """
if MODIFY_SPACEBAR_PRESSED:
    SaveLoadLib(FOLDER + '/modified_navdata.pkl').save('navdata', navdataframes)

""" shutdown threads """
print "\n<Shutting down>\n"
continueEvent.set()
for thread in threads:
    thread.join()
if MODIFY_SPACEBAR_PRESSED:
    spacebarCatcher.join()







