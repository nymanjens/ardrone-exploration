"""
pyggle.view
This library (PYGGEL) is licensed under the LGPL by Matthew Roe and PYGGEL contributors.

The view module contains functions and objects used to manipulate initiation, settings,
and changing of the screen window and OpenGL states.
"""

from OpenGL import error
oglError = error

from include import *

class _Screen(object):
    """A simple object to store screen settings."""
    def __init__(self):
        """Create the screen."""
        self.screen_size = (640, 480)
        self.screen_size_2d = (640, 480)
        self.rect = pygame.rect.Rect(0,0,*self.screen_size)
        self.rect2d = pygame.rect.Rect(0,0,*self.screen_size_2d)
        self.fullscreen = False
        self.hwrender = True
        self.decorated = True
        self.lighting = True
        self.fog = True
        self.fog_color = (.5,.5,.5,.5)

        self.clear_color = (0,0,0,0)

        self.cursor = None
        self.cursor_visible = True
        self.cursor_center = False

        self.debug = True

        self.have_init = False

        self.icon = None
        self.title = None

        self.clips = [(0,0,self.screen_size[0],self.screen_size[1])]

    def set_size(self, size, size2d):
        """Set the screen size."""
        if size:
            self.screen_size = size
        if size2d:
            self.screen_size_2d = size2d
            self.rect2d = pygame.rect.Rect(0,0,*size2d)

        size = self.screen_size
        self.clips = [(0,0,size[0],size[1])]
        self.rect = pygame.rect.Rect(0,0,*size)

        return self.screen_size

    def get_params(self):
        """Return the pygame window initiation parameters needed."""
        params = OPENGL|DOUBLEBUF
        if self.fullscreen:
            params = params|FULLSCREEN
        if self.hwrender:
            params = params|HWSURFACE
        if not self.decorated:
            params = params|NOFRAME
        return params

    def push_clip(self, new):
        """Push a new rendering clip onto the stack - used to limit rendering to a small area."""
        if self.clips: #we have an old one to compare to...
            a,b,c,d = new
            e,f,g,h = self.clips[-1] #last
            new = (max((a, e)), max((b, f)), min((c, g)), min((d, h)))
        self.clips.append(new)
        glScissor(*new)

    def push_clip2d(self, pos, size):
        """Convert a 2d pos/size rect into GL coords for clipping."""
        rx = 1.0 * self.screen_size[0] / self.screen_size_2d[0]
        ry = 1.0 * self.screen_size[1] / self.screen_size_2d[1]

        x, y = pos
        w, h = size

        self.push_clip((int(x*rx), self.screen_size[1]-int(y*ry)-int(h*ry), int(w*rx), int(h*ry)))

    def pop_clip(self):
        """Pop the last clip off the stack."""
        if len(self.clips) == 1:
            return #don't pop the starting clip!
        self.clips.pop()
        glScissor(*self.clips[-1])

    def get_mouse_pos(self):
        """Return mouse pos in relation to the real screen size."""
        return pygame.mouse.get_pos()

    def get_mouse_pos2d(self):
        """Return mouse pos in relation to 2d screen size."""
        rx = 1.0 * self.screen_size_2d[0] / self.screen_size[0]
        ry = 1.0 * self.screen_size_2d[1] / self.screen_size[1]

        mx, my = pygame.mouse.get_pos()

        return int(mx*rx), int(my*ry)

screen = _Screen()

def init(screen_size=None, screen_size_2d=None,
         use_psyco=True, icon_image=None,
         fullscreen=False, hwrender=True,
         decorated=True):
    """Initialize the display, OpenGL and whether to use psyco or not.
       screen_size must be the pixel dimensions of the display window (defaults to 640x480)
       screen_size_2d is the 2d size of the screen that the 2d elements use (defaults to screen_size),
           the 2d elements are handled as if this is the real screen size,
           and then scaled to fit the real screen size at render time,
           this allows multiple screen resolutions without resorting to hacking the 2d or,
           like some 3d engines do, make the 2d elements really 3d that are projected funny.
       use_psyco must be a boolean value indicating whether psyco should be used or not (only if available)
       icon_image must be a string indicating the image to load from disk, or a pygame Surface to use for the window icon
       full_screen indicates whether the render screen is fullscreen or not
       hwrender indicates whether hwrendering should be used for pygame operations
       decorated indicates whether the display window should have a border and top bar or not"""
    if screen_size and not screen_size_2d:
        screen_size_2d = screen_size

    screen_size = screen.set_size(screen_size, screen_size_2d)

    if use_psyco:
        try:
            import psyco
            psyco.background()
        except:
            pass

    screen.fullscreen = fullscreen
    screen.hwrender = hwrender
    screen.decorated = decorated

    pygame.init()

    if type(icon_image) is type(""):
        pygame.display.set_icon(pygame.image.load(icon_image))
    elif icon_image:
        pygame.display.set_icon(icon_image)
    screen.icon = icon_image

    set_title()

    build_screen()

    glEnable(GL_TEXTURE_2D)
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    glEnable(GL_LIGHTING)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
    glEnable(GL_SCISSOR_TEST)
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_BLEND)

    glPointSize(10)

    clear_screen()
    set_fog_color()
    glFogi(GL_FOG_MODE, GL_LINEAR)
    glFogf(GL_FOG_DENSITY, .35)
    glHint(GL_FOG_HINT, GL_NICEST)
    glFogf(GL_FOG_START, 10.0)
    glFogf(GL_FOG_END, 125.0)
    set_fog(True)
    glAlphaFunc(GL_GEQUAL, .5)
    set_background_color()

    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE)

    glFrontFace(GL_CCW)
    glCullFace(GL_BACK)
    glEnable(GL_CULL_FACE)

    screen.have_init = True

def set_background_color(rgb=(0,0,0)):
    """Set the background color (RGB 0-1) of the display."""
    rgba = rgb+(0,)
    glClearColor(*rgba)
    screen.clear_color = rgba
    

def set_title(text="PYGGEL App"):
    pygame.display.set_caption(text)
    screen.title = text

def set_lighting(boolean):
    """Enable/Disable OpenGL lighting."""
    screen.lighting = boolean
    if boolean:
        glEnable(GL_LIGHTING)
    else:
        glDisable(GL_LIGHTING)

def toggle_lighting():
    """Toggle OpenGL lighting."""
    set_lighting(not screen.lighting)

def set_cursor(image, center=False):
    """Set the cursor to image.Image or image.Animation."""
    screen.cursor = image
    screen.cursor_visible = True
    screen.cursor_center = center
    pygame.mouse.set_visible(0)

def set_cursor_visible(boolean=True):
    """Enable/Disable cursor visiblity."""
    screen.cursor_visible = boolean
    if boolean and not screen.cursor:
        pygame.mouse.set_visible(1)
    else:
        pygame.mouse.set_visible(0)

def toggle_cursor_visible():
    """Toggle cursor visibility."""
    screen.cursor_visible = not screen.cursor_visible

def set_fog_color(rgba=(.5,.5,.5,.5)):
    """Set the fog color (RGBA 0-1)"""
    glFogfv(GL_FOG_COLOR, rgba)
    screen.fog_color = rgba

def set_fog(boolean):
    """Enable/Disable fog."""
    screen.fog = boolean
    if boolean:
        glEnable(GL_FOG)
    else:
        glDisable(GL_FOG)

def toggle_fog():
    """Toggle fog."""
    set_fog(not screen.fog)

def set_fog_depth(min=10, max=125):
    """Set the min/max depth of fog."""
    glFogf(GL_FOG_START, min)
    glFogf(GL_FOG_END, max)

def set_debug(boolean):
    """Enable/Disable OpenGL debugging - specifically, this turns on/off calling of glGetError after every call."""
    screen.debug = boolean
    if boolean:
        oglError.ErrorChecker.registerChecker(None)
    else:
        oglError.ErrorChecker.registerChecker(lambda:None)

def toggle_debug():
    """Toggle OpenGL debugging."""
    set_debug(not screen.debug)

def build_screen():
    """Create the display window using the current set of screen parameters."""
    pygame.display.set_mode(screen.screen_size, screen.get_params())

def set2d():
    """Enable 2d rendering."""
    screen_size = screen.screen_size
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()

    glOrtho(0, screen_size[0], screen_size[1], 0, -50, 50)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glDisable(GL_DEPTH_TEST)

def set3d():
    """Enable 3d rendering."""
    screen_size = screen.screen_size
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glViewport(0,0,*screen_size)
    gluPerspective(45, 1.0*screen_size[0]/screen_size[1], 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glEnable(GL_DEPTH_TEST)

def refresh_screen():
    """Flip the screen buffer, displaying any changes since the last clear."""
    if screen.cursor and screen.cursor_visible and pygame.mouse.get_focused():
        glPushMatrix()
        glDisable(GL_LIGHTING)
        screen.cursor.pos = screen.get_mouse_pos2d()
        rx = 1.0 * screen.screen_size[0] / screen.screen_size_2d[0]
        ry = 1.0 * screen.screen_size[1] / screen.screen_size_2d[1]
        glScalef(rx, ry, 1)
        if screen.cursor_center:
            x, y = screen.cursor.pos
            x -= int(screen.cursor.get_width() / 2)
            y -= int(screen.cursor.get_height() / 2)
            screen.cursor.pos = (x, y)
        screen.cursor.render()
        if screen.lighting:
            glEnable(GL_LIGHTING)
        glPopMatrix()
    pygame.display.flip()

def clear_screen(scene=None):
    """Clear buffers."""
    glDisable(GL_SCISSOR_TEST)
    if scene and scene.graph.skybox:
        glClear(GL_DEPTH_BUFFER_BIT)
    else:
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)
    glEnable(GL_SCISSOR_TEST)

def require_init():
    """Called if a function requires the view to have been init'd - raises TypeError if not."""
    if not screen.have_init:
        raise TypeError, "view must be init'd before this action can occur (pyggel.init or pyggel.view.init)"
