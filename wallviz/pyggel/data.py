"""
pyggle.data
This library (PYGGEL) is licensed under the LGPL by Matthew Roe and PYGGEL contributors.

The data module holds all classes used to create, store and access OpenGL data,
like textures, display lists and vertex arrays.
"""

from include import *
import view

class Texture(object):
    """An object to load and store an OpenGL texture"""
    bound = None
    _all_loaded = {}
    def __init__(self, filename=None):
        """Create a texture
           filename can be be a filename for an image, or a pygame.Surface object"""
        view.require_init()
        self.filename = filename

        self.size = (0,0)

        if type(filename) is type(""):
            self._load_file()
        else:
            self._compile(filename)

    def _get_next_biggest(self, x, y):
        """Get the next biggest power of two x and y sizes"""
        if x == y == 1:
            return x, y
        nw = 16
        nh = 16
        while nw < x:
            nw *= 2
        while nh < y:
            nh *= 2
        return nw, nh

    def _load_file(self):
        """Loads file"""
        if not self.filename in self._all_loaded:
            image = pygame.image.load(self.filename)

            self._compile(image)
            if self.filename:
                self._all_loaded[self.filename] = [self]
        else:
            tex = self._all_loaded[self.filename][0]

            self.size = tex.size
            self.gl_tex = tex.gl_tex
            self._all_loaded[self.filename].append(self)

    def _compile(self, image):
        """Compiles image data into texture data"""

        self.gl_tex = glGenTextures(1)

        size = self._get_next_biggest(*image.get_size())

        image = pygame.transform.scale(image, size)

        tdata = pygame.image.tostring(image, "RGBA", 1)
        
        glBindTexture(GL_TEXTURE_2D, self.gl_tex)

        xx, xy = size
        self.size = size
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, xx, xy, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, tdata)

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)

    def bind(self):
        """Binds the texture for usage"""
        if not Texture.bound == self:
            glBindTexture(GL_TEXTURE_2D, self.gl_tex)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)
            Texture.bound = self

    def __del__(self):
        """Clear the texture data"""
        if self.filename in self._all_loaded and\
           self in self._all_loaded[self.filename]:
            self._all_loaded[self.filename].remove(self)
            if not self._all_loaded[self.filename]:
                del self._all_loaded[self.filename]
                try:
                    glDeleteTextures([self.gl_tex])
                except:
                    pass #already cleared...


class BlankTexture(Texture):
    _all_loaded = {}
    def __init__(self, size=(1,1), color=(1,1,1,1)):
        """Create an empty data.Texture
           size must be a two part tuple representing the pixel size of the texture
           color must be a four-part tuple representing the (RGBA 0-1) color of the texture"""
        view.require_init() # It seems to need init on python2.6
        
        self.size = size
        self.filename = repr(size)+repr(color)
        self.gl_tex = None
        if self.filename in self._all_loaded:
            tex = self._all_loaded[self.filename][0]

            self.size = tex.size
            self.gl_tex = tex.gl_tex
            self._all_loaded[self.filename].append(self)
        else:
            i = pygame.Surface(size)
            if len(color) == 4:
                r, g, b, a = color
            else:
                r, g, b = color
                a = 1
            r *= 255
            g *= 255
            b *= 255
            a *= 255
            i.fill((r,g,b,a))
            
            self.gl_tex = glGenTextures(1)
            self._compile(i)

            self._all_loaded[self.filename] = [self]

class DisplayList(object):
    """An object to compile and store an OpenGL display list"""
    def __init__(self):
        """Creat the list"""
        self.gl_list = glGenLists(1)

    def begin(self):
        """Begin recording to the list - anything rendered after this will be compiled into the list and not actually rendered"""
        glNewList(self.gl_list, GL_COMPILE)

    def end(self):
        """End recording"""
        glEndList()

    def render(self):
        """Render the display list"""
        glCallList(self.gl_list)

    def __del__(self):
        """Clear the display list data"""
        try:
            glDeleteLists(self.gl_list, 1)
        except:
            pass #already cleared!

class VertexArray(object):
    """An object to store and render an OpenGL vertex array of vertices, colors and texture coords"""
    def __init__(self, render_type=None, max_size=100):
        """Create the array
           render_type is the OpenGL constant used in rendering, ie GL_POLYGON, GL_TRINAGLES, etc.
           max_size is the size of the array"""
        if render_type is None:
            render_type = GL_QUADS
        self.render_type = render_type
        self.texture = BlankTexture()

        self.max_size = max_size

        self.verts = numpy.empty((max_size, 3), dtype=object)
        self.colors = numpy.empty((max_size, 4), dtype=object)
        self.texcs = numpy.empty((max_size, 2), dtype=object)

    def render(self):
        """Render the array"""
        self.texture.bind()

        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        glEnableClientState(GL_TEXTURE_COORD_ARRAY)

        glVertexPointer(3, GL_FLOAT, 0, self.verts)
        glColorPointer(4, GL_FLOAT, 0, self.colors)
        glTexCoordPointer(2, GL_FLOAT, 0, self.texcs)

        glDrawArrays(self.render_type, 0, self.max_size)

        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
        glDisableClientState(GL_TEXTURE_COORD_ARRAY)

class FrameBuffer(object):
    """An object contains functions to render to a texture instead of to the main display.
       This object renders using FBO's, which are not available to everyone, but they are far faster and more versatile."""
    def __init__(self, size=(512,512), clear_color=(0,0,0,0)):
        """Create the FrameBuffer.
           size must be the (x,y) size of the buffer, will round up to the next power of two
           clear_color must be the (r,g,b) or (r,g,b,a) color of the background of the texture"""
        view.require_init()
        if not FBO_AVAILABLE:
            raise AttributeError("Frame buffer objects not available!")

        _x, _y = size
        x = y = 2
        while x < _x:
            x *= 2
        while y < _y:
            y *= 2
        size = x, y

        self.size = size
        self.clear_color = clear_color

        self.texture = BlankTexture(self.size, self.clear_color)

        if not bool(glGenRenderbuffersEXT):
            print("glGenRenderbuffersEXT doesn't exist")
            exit()
        self.rbuffer = glGenRenderbuffersEXT(1)
        glBindRenderbufferEXT(GL_RENDERBUFFER_EXT,
                              self.rbuffer)
        glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT,
                                 GL_DEPTH_COMPONENT,
                                 size[0],
                                 size[1])

        self.fbuffer = glGenFramebuffersEXT(1)
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,
                             self.fbuffer)
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                                  GL_COLOR_ATTACHMENT0_EXT,
                                  GL_TEXTURE_2D,
                                  self.texture.gl_tex,
                                  0)
        glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT,
                                     GL_DEPTH_ATTACHMENT_EXT,
                                     GL_RENDERBUFFER_EXT,
                                     self.rbuffer)

        self.worked = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT) == GL_FRAMEBUFFER_COMPLETE_EXT

        glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0)
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0)

    def enable(self):
        """Turn this buffer on, swaps rendering to the texture instead of the display."""
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, self.fbuffer)
        r,g,b = self.clear_color[:3]
        glClearColor(r, g, b, 1)
        glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT)

        glPushAttrib(GL_VIEWPORT_BIT)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glViewport(0,0,*self.size)
        gluPerspective(45, 1.0*self.size[0]/self.size[1], 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glEnable(GL_DEPTH_TEST)
        
    def disable(self):
        """Turn off the buffer, swap rendering back to the display."""
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0)
        glClearColor(*view.screen.clear_color)
        glPopAttrib()

    def __del__(self):
        """Clean up..."""
        try:
            glDeleteFramebuffersEXT(1, [self.fbuffer])
        except:
            pass

        try:
            glDeleteRenderbuffersEXT(1, [self.rbuffer])
        except:
            pass

class TextureBuffer(object):
    """An object contains functions to render to a texture, using the main display.
       This object renders using the main display, copying to the texture, and then clearing.
       This object is considerably slower than teh FrameBuffer object, and less versatile,
       because you cannot use these objects mid-render, if you do you will lose whatever was rendered before them!"""
    def __init__(self, size=(512,512), clear_color=(0,0,0,0)):
        """Create the FrameBuffer.
           size must be the (x,y) size of the buffer, will round up to the next power of two
               if size is greater than the display size, it will be rounded down to the previous power of two
           clear_color must be the (r,g,b) or (r,g,b,a) color of the background of the texture"""
        _x, _y = size
        x = y = 2
        while x < _x:
            x *= 2
        while y < _y:
            y *= 2
        while x > view.screen.screen_size[0]:
            x /= 2
        while y > view.screen.screen_size[1]:
            y /= 2
        size = x, y

        self.size = size
        self.clear_color = clear_color

        self.texture = BlankTexture(self.size, self.clear_color)
        self.worked = True

    def enable(self):
        """Turn on rendering to this buffer, clears display buffer and preps it for this object."""
        r,g,b = self.clear_color[:3]

        glClearColor(r, g, b, 1)
        glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT)
        glClearColor(*view.screen.clear_color)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glViewport(0,0,*self.size)
        gluPerspective(45, 1.0*self.size[0]/self.size[1], 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glEnable(GL_DEPTH_TEST)

    def disable(self):
        """Turn of this buffer, and clear the display."""
        self.texture.bind()
        glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0,0,self.size[0], self.size[1], 0)

        glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT)
