import ast, time
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from std_msgs.msg import String
import conversionlib
from settings import *
import rospy

class Publisher:
    pub_vid_data = None
    pub_vid_info = None
    pub_navdata = None
    
    def __init__(self, elf_video=False):
        if elf_video:
            self.pub_vid_data = {
                0: rospy.Publisher('/ardrone_video0/image', Image),
                1: rospy.Publisher('/ardrone_video1/image', Image),
                2: rospy.Publisher('/ardrone_video2/image', Image),
            }
        else:
            self.pub_vid_data = {
                0: rospy.Publisher('/ardrone_video0/compressed', CompressedImage),
                1: rospy.Publisher('/ardrone_video1/image', Image),
                2: rospy.Publisher('/ardrone_video2/image', Image),
            }
        self.pub_vid_info = {
            0: rospy.Publisher('/ardrone_video0/camera_info', CameraInfo),
            1: rospy.Publisher('/ardrone_video1/camera_info', CameraInfo),
            2: rospy.Publisher('/ardrone_video2/camera_info', CameraInfo),
        }
    
    def publish_video(self, vid_type, vid_data, seq=None):
        # convert data
        if vid_type == 1 or vid_type == 2: # /dev/video1 data
            vid_data = conversionlib.convert(vid_data, VID1_WIDTH, VID1_HEIGHT)
        
        # gather data/info
        mesginfo = CameraInfo()
        if vid_type == 0: # video0
            mesgdata = CompressedImage()
            mesgdata.format = "jpeg"
            mesginfo.width = VID0_WIDTH
            mesginfo.height = VID0_HEIGHT
            channel = 0
        elif vid_type == 1: # video1
            mesgdata = Image()
            mesgdata.width = VID1_WIDTH
            mesgdata.height = VID1_HEIGHT
            mesgdata.encoding = "rgb8"
            mesginfo.width = VID1_WIDTH
            mesginfo.height = VID1_HEIGHT
            channel = 1
        elif vid_type == 2: # video2
            mesgdata = Image()
            mesgdata.width = VID1_WIDTH
            mesgdata.height = VID1_HEIGHT
            mesgdata.encoding = "rgb8"
            mesginfo.width = VID1_WIDTH
            mesginfo.height = VID1_HEIGHT
            channel = 2
        elif vid_type == 'changed_video1': # e.g. resize, delay
            mesgdata = Image()
            mesgdata.width = VID1_WIDTH
            mesgdata.height = VID1_HEIGHT
            mesgdata.encoding = "rgb8"
            mesginfo.width = VID1_WIDTH
            mesginfo.height = VID1_HEIGHT
            channel = 2
        elif vid_type == 'elf_video':
            mesgdata = Image()
            mesgdata.width = VID0_WIDTH
            mesgdata.height = VID0_HEIGHT
            mesgdata.encoding = "rgb8"
            mesginfo.width = VID0_WIDTH
            mesginfo.height = VID0_HEIGHT
            mesginfo.header.seq = seq
            channel = 0
        else:
            raise Exception("bad vid_type: (vid_type = %s)" % vid_type)
        mesgdata.header.stamp = rospy.Time.now()
        mesgdata.data = vid_data
        # note: distortion model not used by PTAM(M)
        #mesginfo.distortion_model = 'plumb_bob'
        #mesginfo.D = [-0.33584000000000003, 0.12129000000000001, -0.0011500000000000002, 0.00039000000000000005, 0.0]
        #mesginfo.K = [431.90402, 0.0, 313.01760999999999, 0.0, 432.04253999999997, 246.35373999999999, 0.0, 0.0, 1.0]
        #mesginfo.R = [ 0.99995000000000012, 0.0089900000000000015, -0.0040000000000000001, \
        #              -0.0089700000000000005, 0.99996000000000007, 0.0029100000000000003,  \
        #               0.0040300000000000006, -0.0028700000000000002, 0.99999000000000005]
        #mesginfo.P = [395.29849999999999, 0.0, 315.97555, 0.0, 0.0, 395.29849999999999, 238.45780999999999, 0.0, 0.0, 0.0, 1.0, 0.0]
        #mesginfo.header.frame_id = 'mono_optical_frame'
        mesginfo.header.stamp = mesgdata.header.stamp
        
        # publish data/info
        self.pub_vid_data[channel].publish(mesgdata)
        self.pub_vid_info[channel].publish(mesginfo)
    
    def publish_ds(self, messagetype_id, data):
        if messagetype_id == 100:
            if not self.pub_navdata:
                self.pub_navdata = rospy.Publisher('/ardrone/navdata/ds', String)
            self.pub_navdata.publish(data)
        else:
            self.publish_video(messagetype_id, data)
    
    def publish_dummy_elf(self):
        dummy_data = "vy:0.0,phi:0.0,psi:-12.0,num_frames:3422.0,battery:64.0,timestamp:{0}," \
                     "altitude:722.0,spacebar_pressed:0.0,vx:0.0,theta:0.0,y:-223.83571252,x:-68.3425048458," \
                     "vz:0.0,z:0.0".format(time.time())
        if not self.pub_navdata:
            self.pub_navdata = rospy.Publisher('/ardrone/navdata/elf', String)
        self.pub_navdata.publish(dummy_data)
    
    def publish_elf(self, navdata, vid_data=None):
        # publish navdata
        if not self.pub_navdata:
            self.pub_navdata = rospy.Publisher('/ardrone/navdata/elf', String)
        navdata_str = ','.join('%s:%s' % (key,val) for (key,val) in navdata.items())
        self.pub_navdata.publish(navdata_str)
        # publish vid_data
        if vid_data:
            self.publish_video('elf_video', vid_data, navdata['num_frames'])
    

