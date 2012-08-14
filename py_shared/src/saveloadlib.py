import pickle, os

""" settings """
DATA_PATH = os.path.dirname(__file__) + "/../saveloaddata/"

class SaveLoadLib:
    fpath = ""
    
    def __init__(self, fname='data.pkl'):
        self.fpath =  DATA_PATH + "/" + fname
    
    def save(self, name, value):
        data = self.load_all()
        data[name] = value
        self.save_all(data)
        #print "SaveLoadLib: saved %s=%s to %s" % (name, value, self.fpath)
    
    def load(self, name=None, silent_fail=False):
        data = self.load_all()
        if not data:
            if silent_fail:
                return None
            else:
                raise Exception("SaveLoadLib: {} does not exist".format(os.path.realpath(self.fpath)))
        if name in data:
            return data[name]
        else:
            if name == '*':
                return data
            elif name == None:
                return data.values()[0]
            print "SaveLoadLib: Warning: data[{}] does not exist (fpath = {})".format(name, self.fpath)
            return None
    
    def save_all(self, data):
        pickle.dump(data, open(self.fpath, 'w'))
        
    def load_all(self):
        try:
            return pickle.load(open(self.fpath, 'r'))
        except:
            return {}

class VideoSaveLoadLib:
    fname = None
    cached_vid_frames = None
    cached_vid_metadata = None
    
    BREAK_CODE = """caPHWxzDZOfvyqnaQg7Z7VM31Y6L9YWnwD65dMQ3HxSH3OSrC8NwVK5V8D*D
                    t9LWG4HEHkQ3TzbBE88OyR01BQwQH6570fbdzIgRRWlDyo411ZnhRrcvcAb2
                    SrC8NwVK5V8Dt9LWG4HEHkQ3TzbfDIzHwaXUt9TGk2QkMmRup1$74b0fW9DB"""
    
    def __init__(self, fname):
        self.fname = fname
        self.cached_vid_frames = []
        self.cached_vid_metadata = []
    
    def __del__(self):
        if not self.cached_vid_frames:
            return
        """ write raw data """
        # get filepath
        fpath = self.pathFromFname(self.fname, '.dat')
        print "VideoSaveLoadLib: saving {} frames to {}".format(len(self.cached_vid_frames), fpath)
        # check if folder exists
        folder = os.path.dirname(fpath)
        if not os.path.exists(folder):
            os.makedirs(folder)
        # put array in large string
        write_data = self.BREAK_CODE.join(self.cached_vid_frames)
        # write data
        open(fpath, 'w').write(write_data)
        
        """ write meta-data """
        fpath = self.pathFromFname(self.fname, '.pkl')
        pickle.dump(self.cached_vid_metadata, open(fpath, 'w'))
        print "done"
    
    @staticmethod
    def pathFromFname(fname, ext):
        return DATA_PATH + fname + ext
    
    def saveFrame(self, imageData):
        self.cached_vid_frames.append(imageData.data)
        imageData.data = ""
        self.cached_vid_metadata.append(imageData)
    
    @staticmethod
    def loadFrames(fname):
        """ get raw data """
        # get filepath
        fpath = VideoSaveLoadLib.pathFromFname(fname, '.dat')
        # check if exists
        if not os.path.exists(fpath):
            return None
        # get raw data
        data = open(fpath, 'r').read()
        # decode raw data
        rawdata = data.split(VideoSaveLoadLib.BREAK_CODE)
        
        """ get meta-data """
        fpath = VideoSaveLoadLib.pathFromFname(fname, '.pkl')
        data = pickle.load(open(fpath, 'r'))
        
        """ merge data """
        for i, dat in enumerate(data):
            dat.data = rawdata[i]
        return data


""" OLD VIDEO FUNCTIONS """
# fname = os.path.dirname(__file__) + '/video%d.dat'
# frame_fname = os.path.dirname(__file__) + '/image.jpg'
# def save(typenum, data):
#    pickle.dump(data, open(fname % typenum, 'w'))
# def load(typenum):
#    return pickle.load(open(fname % typenum, 'r'))
# def convert_pickle_frame_to_text():
#    data = load()
#    frame = data[5]
#    save_frame(frame)
# def save_frame(frame):
#    open(frame_fname, 'w').write(frame)

