import subprocess, re

""" utility help function """
def get_self_wlan_ip():
    try:
        s = subprocess.check_output(["ifconfig", "wlan1"], stderr=subprocess.DEVNULL)
        return re.search(r'192.168.1....', s).group(0)
    except AttributeError:
        pass
    except subprocess.CalledProcessError:
        print "jkl"
        pass
    try:
        s = subprocess.check_output(["ifconfig", "wlan0"])
        return re.search(r'192.168.1....', s).group(0)
    except AttributeError:
        return "192.168.1.2"

""" ip settings """
SELF_IP = get_self_wlan_ip()
REMOTE_IP = "192.168.1.1"
SELF_PORT_VIDEO = 7779
REMOTE_PORT_VID = { 0: 7777, 1: 7777, }

""" communication settings """
N_HEADER = 3 # bytes

""" video settings """
# general
REQUESTED_VID_TYPES = [0, 1]

# video0
VID0_SUBSAMPLED = True
if not VID0_SUBSAMPLED:
    VID0_WIDTH = 640
    VID0_HEIGHT = 480
else:
    VID0_WIDTH = 320
    VID0_HEIGHT = 240

# video1
VID1_WIDTH = 176
VID1_HEIGHT = 144


