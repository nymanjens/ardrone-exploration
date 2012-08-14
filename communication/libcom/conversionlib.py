import sys, numpy, Image
import numpy as np

################ UTILITY FUNCTION ################
def convert(data, VID_WIDTH, VID_HEIGHT):
	data = YUV422FromYUV420P(data, VID_WIDTH, VID_HEIGHT)
	data = YUV444FromYUV422(data)
	return RGBFromYUV444(data, VID_WIDTH, VID_HEIGHT)

################ SUBSAMPLING CONVERSIONS ################
def YUV422FromYUV420P(YUV420P, W, H):
    """Converts YUV420 planar to YUV422
    
    Input:
        - YUV420P: [Y0 Y1 ... U0 U1 ... V1 V2 ...]
        - W: width
        - H: height
    
    Output: [Y0 U0 Y1 V1 Y2 U2 Y3 V3]
    
    Author: Jens Nyman
    """
    # assertion
    if not len(YUV420P) == W*H*3/2:
        print "YUV422FromYUV420P(): Error: length has to be W*H*3/2 = %d, length given is %d" % (W*H*3/2, len(YUV420P))
        sys.exit()
    
    # get help vars
    N = W * H
    n = N / 4
    # get planes
    Yplane = YUV420P[0:N]
    Uplane = YUV420P[N:N+n]
    Vplane = YUV420P[N+n:]
    # create output var
    YUV422 = [0] * (N * 2)
    # add Y, U and V
    YUV422[0::2] = Yplane
    for h in range(H/2):
        YUV422[h*4*W+1 : h*4*W+W*2 : 4] = YUV422[h*4*W+2*W+1 : h*4*W+2*W+W*2 : 4] = Uplane[h*W/2 : (h+1)*W/2]
        YUV422[h*4*W+3 : h*4*W+W*2 : 4] = YUV422[h*4*W+2*W+3 : h*4*W+2*W+W*2 : 4] = Vplane[h*W/2 : (h+1)*W/2]
    # return
    return YUV422

################ RESIZE CONVERSIONS ################

def resize(vid_data, (oldW, oldH), (newW, newH)):
    image = Image.fromstring('RGB', (oldW, oldH), vid_data)
    image = image.resize((newW, newH), Image.BILINEAR)
    return image.tostring()


def crop(data, size):
    """ function to crop image (test if static border influence PTAMM) """
    N = 35
    N = 60
    mode = 'RGB'
    img = Image.fromstring(mode, size, data)
    npimg = np.array(img)
    npimg = npimg[N:-N,N:-N]
    img = Image.fromarray(npimg)
    img = img.resize(size, Image.BILINEAR)
    return img.tostring()

### DEBUG CODE ###
#H = 4
#W = 4
#out = YUV422FromYUV420P('YYYY1234YYYY5678UBubVRvr', W, H)
#for h in range(H):
    #print out[h*W*2 : (h+1)*W*2]


################ COLOR CONVERSIONS ################
#def clip(v):
    #""" Clip to 0-255 """
    #return numpy.clip(v, 0, 255)

def RGBFromYUV444(YUV444, W, H):
    #Y = numpy.array(YUV444[::3])
    #U = numpy.array(YUV444[1::3])
    #V = numpy.array(YUV444[2::3])
    #B = numpy.int(numpy.round(1.164*(Y - 16) + 2.018*(U - 128)))
    #G = numpy.int(numpy.round(1.164*(Y - 16) - 0.813*(V - 128) - 0.391*(U - 128)))
    #R = numpy.int(numpy.round(1.164*(Y - 16) + 1.596*(V - 128)))
    #return clip("".join(r+g+b for r, g, b in zip(R,G,B)))
    
    image = Image.fromstring('YCbCr', (W, H), YUV444)
    return image.convert("RGB").tostring()
    
################ NEXT FUNCTIONS ARE COPIED FROM http://york.wikidot.com/color-space ################
def YUV444FromYUV422(YUV422):
    """Returns the YUV444 string from a given YUV422 string
 
    An input byte stream with the order:
        Y0 U0 Y1 V1 Y2 U2 Y3 V3
 
    is reordered into:
        [Y0 U0 V1] [Y1 U0 V1] [Y2 U2 V3] [Y3 U2 V3]
 
    Example
    -------
    >>> YUV444FromYUV422('00112233YUYVYUYV')
    '001101223323YUVYUVYUVYUV'
 
    See Also
    --------
    http://en.wikipedia.org/wiki/Chroma_subsampling
    """
    Y = YUV422[::2]
    U = "".join((u*2 for u in YUV422[1::4]))
    V = "".join((v*2 for v in YUV422[3::4]))
    return "".join(y+u+v for y, u, v in zip(Y,U,V))


def YUV422FromYUV444(YUV444):
    """Returns the YUV422 string from a given YUV444 string
 
    An input byte stream with the order:
        [Y0 U0 V0] [Y1 U1 V1] [Y2 U2 V2] [Y3 U3 V3]
 
    is reordered into:
        Y0 U0 Y1 V1 Y2 U2 Y3 V3
 
    Example
    -------
    >>> YUV422FromYUV444('000111222333YUVYUVYUVYUV')
    '00112233YUYVYUYV'
 
    See Also
    --------
    http://en.wikipedia.org/wiki/Chroma_subsampling
    """
    Y = YUV444[::3]
    U = YUV444[1::6]
    V = YUV444[5::6]
    UV = "".join((u+v for u, v in zip(U,V)))
    return "".join((y+uv for y, uv in zip(Y,UV)))


def YUV444FromYUV420(YUV420, width):
    """Returns the YUV444 string from a given YUV420 string
 
    An input byte stream with the order:
        Yo0 Uo0 Yo1 Yo2 Uo2 Yo3
        Ye0 Ve0 Ye1 Ye2 Ve2 Ye3
 
    is reordered into:
        [Yo0 Uo0 Ve0] [Yo1 Uo0 Ve0] [Yo2 Uo2 Ve2] [Yo3 Uo2 Ve2]
        [Ye0 Uo0 Ve0] [Ye1 Uo0 Ve0] [Ye2 Uo2 Ve2] [Ye3 Uo2 Ve2]
 
    Example
    -------
    >>> YUV444FromYUV420('001223001223ooooooeeeeeeYUYYUYYVYYVY', 4)
    '000100222322000100222322ooeooeooeooeeoeeoeeoeeoeYUVYUVYUVYUVYUVYUVYUVYUV'
 
    See Also
    --------
    http://en.wikipedia.org/wiki/Chroma_subsampling
    """
    from numpy import array, column_stack, hstack
 
    col = width*3/2
    row = len(YUV420)/col
    YUV420 = array(tuple(YUV420)).reshape((row,col))
    O420 = YUV420[0::2,:].flat
    E420 = YUV420[1::2,:].flat
 
    Yo = column_stack((O420[0::3], O420[2::3])).flat
    Ye = column_stack((E420[0::3], E420[2::3])).flat
    Uo0 = O420[1::6]
    Ve0 = E420[1::6]
    Uo2 = O420[4::6]
    Ve2 = E420[4::6]
 
    row /= 2
    col *= 2
    O444 = column_stack((Yo[0::4],Uo0,Ve0,Yo[1::4],Uo0,Ve0,
                         Yo[2::4],Uo2,Ve2,Yo[3::4],Uo2,Ve2)).reshape((row,col))
    E444 = column_stack((Ye[0::4],Uo0,Ve0,Ye[1::4],Uo0,Ve0,
                         Ye[2::4],Uo2,Ve2,Ye[3::4],Uo2,Ve2)).reshape((row,col))
 
    return "".join(hstack((O444, E444)).flat)

