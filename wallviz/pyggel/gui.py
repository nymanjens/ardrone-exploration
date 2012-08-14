"""
pyggle.gui
This library (PYGGEL) is licensed under the LGPL by Matthew Roe and PYGGEL contributors.

The gui module contains classes to create and use a simple Graphical User Interface.
"""

from include import *
import view, font, event, misc
import time
import image as _image
import os

tdef = "theme"
class Theme(object):
    """A class used to create/load themes and content for widgets."""
    def __init__(self, app):
        """Create the theme
           app must be the App object that is creating this theme"""
        self.theme = self.make_default_theme()
        self.app = app
        self.path = ""
        self.make_fonts()

    def make_fonts(self):
        """Build all the fonts the theme calls for."""
        g = {}
        for a in self.theme["Fonts"]:
            b = self.theme["Fonts"][a]
            g[a] = (font.Font(b["fontfile"], b["fontsize"]),
                    font.MEFont(b["fontfile"], b["fontsize"]))
            for i in b["images"]:
                g[a][0].add_image(i, self.data(b["images"][i]))
                g[a][1].add_image(i, self.data(b["images"][i]))
        self.app.update_fonts(g)

    def load(self, filename):
        """Load a theme from filename."""
        if filename:
            try:
                self.path = os.path.split(filename)[0]
            except:
                self.path = ""
        else:
            self.path = ""

        if misc.test_safe(filename)[0]:
            exec "g={%s}"%open(filename, "rU").read()
            for widget in g:
                for val in g[widget]:
                    self.theme[widget][val] = g[widget][val]

            self.make_fonts()
        else:
            print "Warning! Theme: %s is not safe!"%filename

    def make_default_theme(self):
        """Create a base theme so that anything not specified in a loaded theme (if any) are missed."""
        g = {"Fonts":{
                "default":{
                    "fontfile":None,
                    "fontsize":32,
                    "images":{}
                    }
                },
            "App":{
                "font":"default"
                },
            "Widget":{
                "font":"default"
                },
            "NewLine":{
                "font":"default"
                 },
            "Spacer":{
                "font":"default"
                },
            "Icon":{
                "font":"default"
                },
            "Frame":{
                "font":"default",
                "size":(100,100),
                "background-image":None,
                "image-border":None
                },
            "Label":{
                "font":"default",
                "text":"label...",
                "background-image":None,
                "font-color":(1,1,1,1),
                "font-color-inactive":(1,1,1,.5),
                "font-underline":False,
                "font-italic":False,
                "font-bold":False,
                "image-border":None
                },
            "Button":{
                "font":"default",
                "text":"button...",
                "background-image":None,
                "background-image-hover":None,
                "background-image-click":None,
                "font-color":(1,1,1,1),
                "font-color-hover":(0,1,0,1),
                "font-color-click":(1,0,0,1),
                "font-underline":False,
                "font-italic":False,
                "font-bold":False,
                "font-underline-hover":False,
                "font-italic-hover":False,
                "font-bold-hover":False,
                "font-underline-click":False,
                "font-italic-click":False,
                "font-bold-click":False,
                "image-border":None
                },
            "Checkbox":{
                "font":"default",
                "background-image":None,
                "check-image":None,
                "image-border":None
                },
            "Radio":{
                "size":(100,100),
                "font":"default",
                "background-image":None,
                "option-background-image":None,
                "option-check-image":None,
                "font-color":(1,1,1,1),
                "font-color-inactive":(1,1,1,.5),
                "font-underline":False,
                "font-italic":False,
                "font-bold":False,
                "image-border":None
                },
            "MultiChoiceRadio":{
                "size":(100,100),
                "font":"default",
                "background-image":None,
                "option-background-image":None,
                "option-check-image":None,
                "font-color":(1,1,1,1),
                "font-color-inactive":(1,1,1,.5),
                "font-underline":False,
                "font-italic":False,
                "font-bold":False,
                "image-border":None
                },
            "Input":{
                "font":"default",
                "text":"input...",
                "width":100,
                "background-image":None,
                "font-color":(1,1,1,1),
                "font-color-inactive":(1,1,1,.5),
                "font-underline":False,
                "font-italic":False,
                "font-bold":False,
                "image-border":None
                },
            "MoveBar":{
                "font":"default",
                "font-color":(1,1,1,1),
                "font-color-inactive":(1,1,1,.5),
                "font-underline":False,
                "font-italic":False,
                "font-bold":False,
                "title":"Window...",
                "width":100,
                "background-image":None,
                "image-border":None
                },
            "Window":{
                "font":"default",
                "font-color":(1,1,1,1),
                "font-color-inactive":(1,1,1,.5),
                "font-underline":False,
                "font-italic":False,
                "font-bold":False,
                "size":(100,100),
                "background-image":None,
                "movebar-background-image":None,
                "image-border":None
                },
            "Menu":{
                "name":"menu...",
                "font":"default",
                "font-color":(1,1,1,1),
                "font-color-hover":(0,1,0,1),
                "font-color-click":(1,0,0,1),
                "background-image":None,
                "background-image-hover":None,
                "background-image-click":None,
                "menu-background-image":None,
                "option-background-image":None,
                "option-background-image-hover":None,
                "option-background-image-click":None,
                "option-font-color":(1,1,1,1),
                "option-font-color-hover":(0,1,0,1),
                "option-font-color-click":(1,0,0,1),
                "sub-background-image":None,
                "sub-background-image-hover":None,
                "sub-background-image-click":None,
                "sub-icon":None,
                "sub-font-color":(0,0,1,1),
                "sub-font-color-hover":(0,1,1,1),
                "sub-font-color-click":(1,1,0,1),
                "font-underline":False,
                "font-italic":False,
                "font-bold":False,
                "font-underline-hover":False,
                "font-italic-hover":False,
                "font-bold-hover":False,
                "font-underline-click":False,
                "font-italic-click":False,
                "font-bold-click":False,
                "option-font-underline":False,
                "option-font-italic":False,
                "option-font-bold":False,
                "option-font-underline-hover":False,
                "option-font-italic-hover":False,
                "option-font-bold-hover":False,
                "option-font-underline-click":False,
                "option-font-italic-click":False,
                "option-font-bold-click":False,
                "sub-font-underline":False,
                "sub-font-italic":False,
                "sub-font-bold":False,
                "sub-font-underline-hover":False,
                "sub-font-italic-hover":False,
                "sub-font-bold-hover":False,
                "sub-font-underline-click":False,
                "sub-font-italic-click":False,
                "sub-font-bold-click":False,
                "image-border":None
                }

            }
        return g

    def get(self, widget, val):
        """Get the value of the theme declaration for widget.val"""
        return self.theme[widget.widget_name][val]

    def data(self, name):
        """Return the path/to/name for a data object."""
        if self.path:
            return os.path.join(self.path, name)
        return name

class Packer(object):
    """An object used to position widgets so they don't overlap, in a set pattern."""
    def __init__(self, app, packtype="wrap", size=(10,10)):
        """Create the Packer
           app must be the object creating the packer
           packtype must be None or string of wrap type to use, available values are:
               "wrap", "center", None
           size must be a tuple (x, y) representing the dimensions of the area for packing.
               packer will try and keep widgets inside this confine, but
               if a single widget is too wide, it will spill over the width, and
               the packer will not limit widgets in the height value - just use it for centering and such"""
        self.app = app
        self.packtype = packtype
        self.size = size

        self.need_to_pack = False

    def pack(self):
        """Position all app.widgets."""
        self.app.widgets.reverse()
        getattr(self, "pack_%s"%self.packtype)()
        self.app.widgets.reverse()
        self.need_to_pack = False

    def pack_wrap(self):
        """Position app.widgets in a manner that simply tried to keep them from spilling over the width."""
        nw = 0
        nh = 0
        newh = 0

        for i in self.app.widgets:
            if isinstance(i, NewLine):
                nw = 0
                nh += newh + i.size[1]
                newh = 0
                i.pos = (nw, nh)
                continue
            if i.override_pos:
                continue
            w, h = i.size
            if nw + w > self.size[0] and nw:
                nh += newh + 1
                newh = h
                nw = w
                pos = (0, nh)
            else:
                pos = (nw, nh)
                nw += w
                if h > newh:
                    newh = h
            i.force_pos_update(pos)

    def pack_center(self):
        """Position app.widgets in a manner that tries to center them in the center of the size given."""
        rows = [[]]
        w = 0
        for i in self.app.widgets:
            if isinstance(i, NewLine):
                rows.append([i])
                rows.append([])
                continue
            if i.override_pos:
                continue
            rows[-1].append(i)
            w += i.size[0]
            if w >= self.size[0]:
                rows.append([])
                w = 0

        sizes = []
        for row in rows:
            h = 0
            w = 0
            for widg in row:
                if widg.size[1] > h:
                    h = widg.size[1]
                w += widg.size[0]
            sizes.append((w, h))

        center = self.size[1] / 2
        height = 0
        for i in sizes:
            height += i[1]
        top = center - height / 2
        for i in xrange(len(rows)):
            w = self.size[0] / 2 - sizes[i][0] / 2
            for widg in rows[i]:
                widg.force_pos_update((w, top))
                w += widg.size[0]
            top += sizes[i][1]

    def pack_None(self):
        """Position app.widgets so they merely do not overlap."""
        nw = 0
        nh = 0
        newh = 0

        for i in self.app.widgets:
            if isinstance(i, NewLine):
                nw = 0
                nh += newh + i.size[1]
                newh = 0
                i.pos = (nw, nh)
                continue
            if i.override_pos:
                continue
            w, h = i.size
            pos = (nw, nh)
            nw += w
            if h > newh:
                newh = h
            i.force_pos_update(pos)

class App(object):
    """A simple Application class, to hold and control all widgets."""
    def __init__(self, event_handler):
        """Create the App.
           event_handler must be the event.Handler object that the gui will use to get events,
               sets event_handler's current gui to this App"""
        self.event_handler = event_handler
        self.event_handler.gui = self
        self.event_handler.all_guis.append(self)

        self.widgets = []

        self.dispatch = event.Dispatcher()

        self.fonts = {"default":(font.Font(), font.MEFont())}
        self.theme = Theme(self)

        self.packer = Packer(self, size=view.screen.screen_size_2d)

        self.visible = True

        self.pos = (0,0)
        self.size = view.screen.screen_size_2d

    def activate(self):
        """Sets event_handler's active gui to self."""
        self.event_handler.gui = self
        self.visible=True
        for i in self.event_handler.all_guis:
            if not i == self:
                i.visible = False
                for x in i.widgets:
                    x.unfocus()
    def kill(self):
        """Removes App from event_handler."""
        if self in self.event_handler.all_guis:
            self.event_handler.all_guis.remove(self)

    def get_font(self, name):
        """Return theme Font and MEFont bound to name."""
        return self.fonts[name]

    def get_regfont(self, name):
        """Return theme Font bound to name."""
        return self.fonts[name][0]

    def get_mefont(self, name):
        """Return theme MEFont bound to name."""
        return self.fonts[name][1]

    def update_fonts(self, fonts):
        """Sets all App/Theme fonts to fonts.
           fonts must be a dict of {"name":(Font, MEFont)} obejcts"""
        self.fonts = fonts
        for i in self.widgets:
            if i.widget_name in ("Frame", "Window"):
                i.update_fonts(fonts)

    def get_mouse_pos(self):
        """Return mouse pos based on App position - always (0,0)"""
        return view.screen.get_mouse_pos2d()

    def new_widget(self, widget):
        """Add a new widget to the App."""
        if not widget in self.widgets:
            self.widgets.insert(0, widget)
            if not widget.override_pos:
                self.packer.need_to_pack = True

    def handle_mousedown(self, button, name):
        """Callback for mouse click events from the event_handler."""
        if not self.visible:
            return False
        for i in self.widgets:
            if i.visible:
                if i.handle_mousedown(button, name):
                    return True
        return False

    def handle_mouseup(self, button, name):
        """Callback for mouse release events from the event_handler."""
        if not self.visible:
            return False
        for i in self.widgets:
            if i.visible:
                if i.handle_mouseup(button, name):
                    return True
        return False

    def handle_mousehold(self, button, name):
        """Callback for mouse hold events from the event_handler."""
        if not self.visible:
            return False
        for i in self.widgets:
            if i.visible:
                if i.handle_mousehold(button, name):
                    return True
        return False

    def handle_mousemotion(self, change):
        """Callback for mouse motion events from event_handler."""
        if not self.visible:
            return False
        for i in self.widgets:
            if i.visible:
                if i.handle_mousemotion(change):
                    return True

    def handle_uncaught_event(self, event):
        """Callback for uncaught_event events from event_handler."""
        if not self.visible:
            return False
        for i in self.widgets:
            if i.visible:
                if i.handle_uncaught_event(event):
                    return True
        return False

    def handle_keydown(self, key, string):
        """Callback for key press events from event_handler."""
        if not self.visible:
            return False
        for i in self.widgets:
            if i.visible:
                if i.handle_keydown(key, string):
                    return True
        return False

    def handle_keyup(self, key, string):
        """Callback for key release events from event_handler."""
        if not self.visible:
            return False
        for i in self.widgets:
            if i.visible:
                if i.handle_keyup(key, string):
                    return True
        return False

    def handle_keyhold(self, key, string):
        """Callback for key hold events from event_handler."""
        if not self.visible:
            return False
        for i in self.widgets:
            if i.visible:
                if i.handle_keyhold(key, string):
                    return True
        return False

    def next_widget(self):
        """Cycle widgets so next widget is top one."""
        self.widgets.append(self.widgets.pop(0))
        while not self.widgets[0].visible:
            self.widgets.append(self.widgets.pop(0))

    def set_top_widget(self, widg):
        """Moves widget 'widg' to top position."""
        if widg in self.widgets:
            self.widgets.remove(widg)
        self.widgets.insert(0, widg)
        for i in self.widgets:
            if i.visible:
                if not i == widg:
                    i.unfocus()

    def render(self, camera=None):
        """Renders all widgets, camera can be None or the camera object used to render the scene."""
        if not self.visible:
            return False
        self.widgets.reverse()
        for i in self.widgets:
            if i.visible: i.render()
        self.widgets.reverse()

class Widget(object):
    """The base widget class - all other widgets should inherit from this widget."""
    widget_name = "Widget"
    def __init__(self, app, pos=None, font=tdef, image_border=tdef, special_name=None):
        """Create the widget
           app must be the App/Frame/Window this widget is attached to
           pos must be None (to use app.packer) or the (x,y) pos of the widget
           font must be tdef/None or the (Font, MEFont) fonts to use
           image_border must be tdef or the pixel size of the border tiles of the background image (if any)
           special_name must be None or the name used to grab a value different from widget_name from the theme"""
        self.app = app
        self.pos = pos
        self.size = (0,0)
        if pos:
            self.override_pos = True
        else:
            self.override_pos = False

        if special_name:
            self.widget_name = special_name
        if image_border == tdef:
            try:
                image_border = self.app.theme.get(self, "image-border")
            except:
                image_border = None

        self.image_border = image_border

        if font in (tdef, None):
            font = self.app.theme.get(self, "font")
        self.theme = self.app.theme
        self.font, self.mefont = self.app.fonts[font]

        self.dispatch = event.Dispatcher()

        self.visible = True
        self.app.new_widget(self)
        self.image = None
        self.background = None #background image!
        self.tsize = (0,0)
        self.tshift = (0,0)

        self._mhold = False
        self._mhover = False
        self.key_active = False
        self.key_hold_lengths = {}
        self.khl = 200 #milliseconds to hold keys for repeat!

    def get_root_app(self):
        """Return the root App object this widget, or it's parent (etc.) is attached to."""
        app = self.app
        while hasattr(app, "app") and app.app:
            app = app.app
        return app

    def load_background(self, filename):
        """Load and scale an image ot be used for the background for a widget."""
        try:
            x, y = pygame.image.load(self.theme.data(filename)).get_size()
        except:
            x, y = pygame.image.load(filename).get_size()
        x = int(x/3)
        y = int(y/3)
        try:
            new, tsize = _image.load_and_tile_resize_image(self.theme.data(filename), (self.size[0]+x*2, self.size[1]+y*2),
                                                           border_size=self.image_border)
        except:
            new, tsize = _image.load_and_tile_resize_image(filename, (self.size[0]+x*2, self.size[1]+y*2),
                                                           border_size=self.image_border)
        size = new.get_size()

        return new, size, tsize, tsize

    def pack(self):
        """Position this widget with others using app.packer."""
        self.app.packer.pack()

    def _collidem(self):
        """Returns whether mouse is colliding with the widget or not."""
        x, y = self.app.get_mouse_pos()
        a, b = self.pos
        w, h = self.size
        return (x >= a and x <= a+w) and (y >= b and y <= b+h)

    def focus(self):
        """Focus this widget so it is at the top of rendering and event calls."""
        self.app.set_top_widget(self)
        self.key_active = True
        self.dispatch.fire("focus")
        self._mhover = self._collidem()

    def handle_mousedown(self, button, name):
        """Handle a mouse down event from the App."""
        self._mhover = self._collidem()
        if name == "left":
            if self._mhover:
                self._mhold = True
                self.focus()
                return True
            self.unfocus()

    def handle_mouseup(self, button, name):
        """Handle a mouse release event from the App."""
        self._mhover = self._collidem()
        if name == "left":
            if self._mhold and self._mhover:
                self._mhold = False
                self.dispatch.fire("click")
                return True

    def handle_mousehold(self, button, name):
        """Handle a mouse hold event from the App."""
        if name == "left":
            if self._mhold:
                return True

    def handle_mousemotion(self, change):
        """Handle a mouse motion event from the App."""
        self._mhover = self._collidem()
        for i in self.app.widgets:
            if not i == self:
                i._mhover = False
        return self._mhover

    def can_handle_key(self, key, string):
        """Return whether key/string is used by this widget."""
        return False

    def handle_keydown(self, key, string):
        """Handle a key down event from the App."""
        if self.can_handle_key(key, string):
            if self.key_active:
                self.dispatch.fire("keypress", key, string)
                return True

    def handle_keyhold(self, key, string):
        """Handle a key hold event from the App."""
        if self.can_handle_key(key, string):
            if self.key_active:
                if key in self.key_hold_lengths:
                    if time.time() - self.key_hold_lengths[key] >= self.khl*0.001:
                        self.handle_keydown(key, string)
                        self.key_hold_lengths[key] = time.time()
                else:
                    self.key_hold_lengths[key] = time.time()
                return True

    def handle_keyup(self, key, string):
        """Handle a key release event from the App."""
        if self.can_handle_key(key, string):
            if self.key_active:
                if key in self.key_hold_lengths:
                    del self.key_hold_lengths[key]
                return True

    def handle_uncaught_event(self, event):
        """Handle any non mouse or key event from the App."""
        pass

    def get_clip(self, offset=(0,0)):
        """Return the render clipping region of the image."""
        x, y = self.pos
        w, h = self.size
        x += self.tsize[0] + offset[0]
        y += self.tsize[1] + offset[1]
        w -= self.tsize[0]*2
        h -= self.tsize[1]*2
        return (x, y), (w, h)

    def force_pos_update(self, pos):
        """Force the widget to change it's position."""
        self.pos = pos

    def render(self, offset=(0,0)):
        """Render the widget at widget.pos + offset."""
        x, y = self.pos
        x += offset[0]
        y += offset[1]
        if self.background:
            self.background.pos = (x, y)
            self.background.render()
            self.background.pos = self.pos
        if self.image:
            self.image.pos = (x+self.tshift[0], y+self.tshift[1])
            self.image.render()
            self.image.pos = (self.pos[0]+self.tshift[0], self.pos[1]+self.tshift[1]) #need to reset!

    def unfocus(self):
        """Remove the widget's focus."""
        self.key_active=False
        self.key_hold_lengths = {}
        self.dispatch.fire("unfocus")
        self._mhold=False
        self._mhover=False

class Frame(App, Widget):
    """A widget capable of having other widgets inside it, like an app, while having the attributes of a Widget as well."""
    widget_name = "Frame"
    def __init__(self, app, pos=None, size=tdef, background_image=tdef, font=tdef, image_border=tdef,
                 special_name=None):
        """Create the frame
           app must be the App/Frame/Window this widget is attached to
           pos must be None (to use app.packer) or the (x,y) pos of the widget
           size must be tdef or the (x,y) size of the frame
           background_image must be tdef or the filename of the image to use as the background
           font must be tdef/None or the (Font, MEFont) fonts to use
           image_border must be tdef or the pixel size of the border tiles of teh background image (if any)
           special_name must be None or the name used to grab a value different from widget_name from the theme"""
        Widget.__init__(self, app, pos, font, image_border, special_name)

        if size == tdef:
            size = self.theme.get(self, "size")
        if background_image == tdef:
            background_image = self.theme.get(self, "background-image")
        self.size = size

        self.widgets = []

        self.fonts = self.app.fonts

        if background_image:
            self.background, self.size, self.tsize, self.tshift = self.load_background(background_image)
        x, y = self.size
        x -= self.tsize[0]*2
        y -= self.tsize[1]*2
        self.packer = Packer(self, size=(x, y))
        self.pack()

    def _collidem_c(self):
        """Returns whether mouse is touching the renderable area of the frame."""
        x, y = self.app.get_mouse_pos()
        a, b = self.pos
        w, h = self.size
        c, d = self.tshift
        f, g = self.tsize
        return (a+c <= x <= a+w-f) and (b+d <= y <= b+h-g)

    def get_mouse_pos(self):
        """Return the mouse pos in relation to teh renderable area of the frame."""
        x, y = self.app.get_mouse_pos()
        x -= self.pos[0] + self.tshift[0]
        y -= self.pos[1] + self.tshift[1]
        return x, y

    def handle_mousedown(self, button, name):
        Widget.handle_mousedown(self, button, name)
        if self._mhover:
            App.handle_mousedown(self, button, name)
        return self._collidem()
    handle_mousedown.__doc__ = Widget.handle_mousedown.__doc__

    def handle_mouseup(self, button, name):
        if self._mhold:
            Widget.handle_mouseup(self, button, name)
            App.handle_mouseup(self, button, name)
            return True
    handle_mouseup.__doc__ = Widget.handle_mouseup.__doc__

    def handle_mousehold(self, button, name):
        Widget.handle_mousehold(self, button, name)
        if self._mhold:
            App.handle_mousehold(self, button, name)
            return True
    handle_mousehold.__doc__ = Widget.handle_mousehold.__doc__

    def handle_mousemotion(self, change):
        Widget.handle_mousemotion(self, change)
        if self._collidem_c():
            App.handle_mousemotion(self, change)
            return True
        for i in self.widgets:
            i._mhover = False
        return self._collidem()
    handle_mousemotion.__doc__ = Widget.handle_mousemotion.__doc__

    def render(self, offset=(0,0)):
        Widget.render(self, offset)
        view.screen.push_clip2d(*self.get_clip(offset))
        self.widgets.reverse()

        x, y = self.pos
        x += offset[0]
        y += offset[1]
        offset = (x+self.tshift[0], y+self.tshift[1])
        for i in self.widgets:
            if i.visible: i.render(offset)
        self.widgets.reverse()
        view.screen.pop_clip()
    render.__doc__ = Widget.render.__doc__

    def unfocus(self):
        Widget.unfocus(self)
        for i in self.widgets:
            i.unfocus()
    unfocus.__doc__ = Widget.unfocus.__doc__

class NewLine(Widget):
    """A widget that simply forces other widgets to a new line in the packer."""
    widget_name = "NewLine"
    def __init__(self, app, height=0, special_name=None):
        """Create the NewLine
           app must be the App/Frame/Window this widget is attached to
           height must be None or the minimum height of the new line
           special_name must be None or the name used to grab a value different from widget_name from the theme"""
        Widget.__init__(self, app, None, tdef, None, special_name)
        self.size = (0, height)
        self.pack()

class Spacer(Widget):
    """A widget that simply forces other widgets over in the packer."""
    widget_name = "Spacer"
    def __init__(self, app, size=(0,0), special_name=None):
        """Create the spacer
           app must be the App/Frame/Window this widget is attached to
           size must be the (x,y) size of the space
           special_name must be None or the name used to grab a value different from widget_name from the theme"""
        Widget.__init__(self, app, None, tdef, None, special_name)
        self.size = size
        self.pack()

class Icon(Widget):
    """A widget that simply contains an image to be rendered."""
    widget_name = "Icon"
    def __init__(self, app, pos=None, image=None):
        """Create the icon
           app must be the App/Frame/Window this widget is attached to
           pos must be None (to use app.packer) or the (x,y) pos of the widget
           image must be the image.Image, image.Animation or filename of the image"""
        Widget.__init__(self, app, pos, tdef, None, None)
        if image:
            if isinstance(image, _image.Animation) or\
               isinstance(image, _image.Image):
                self.image = image
            else:
                if type(image) is type("") and image.split(".")[-1] in ("gif", "GIF"):
                    self.image = _image.GIFImage(image)
                else:
                    self.image = _image.Image(image)

            self.size = self.image.get_size()
        self.pack()

class Label(Widget):
    """A widget that simply displays text, with a background if specified."""
    widget_name = "Label"
    def __init__(self, app, text=tdef, pos=None, background_image=tdef, font_color=tdef,
                 font_color_inactive=tdef, font=tdef,
                 font_underline=tdef, font_italic=tdef, font_bold=tdef,
                 image_border=tdef,
                 special_name=None):
        """Create the label
           app must be the App/Frame/Window this widget is attached to
           text must be tdef or the text of the label
           pos must be None (to use app.packer) or the (x,y) pos of the widget
           background_image must be tdef or the filename of the image to use as the background
           font_color must be tdef or the (R,G,B,A)(0-1) color of the text
           font_color_inactive must be tdef or the (R,G,B,A)(0-1) color of the text for when the widget is not focused
           font must be tdef/None or the (Font, MEFont) fonts to use
           font_underline must be tdef or True/False - whether text is underlined
           font_italic must be tdef or True/False - whether text is italic
           font_bold must be tdef or True/False - whether text is bold
           image_border must be tdef or the pixel size of the border tiles of teh background image (if any)
           special_name must be None or the name used to grab a value different from widget_name from the theme"""
        Widget.__init__(self, app, pos, font, image_border, special_name)

        if text == tdef:
            text = self.theme.get(self, "text")
        if background_image == tdef:
            background_image = self.theme.get(self, "background-image")
        if font_color == tdef:
            font_color = self.theme.get(self, "font-color")
        if font_color_inactive == tdef:
            font_color_inactive = self.theme.get(self, "font-color-inactive")
        if font_underline == tdef:
            font_underline = self.theme.get(self, "font-underline")
        if font_italic == tdef:
            font_italic = self.theme.get(self, "font-italic")
        if font_bold == tdef:
            font_bold = self.theme.get(self, "font-bold")

        self.font_color = font_color
        self.font_color_inactive = font_color_inactive

        self.text = text
        self.image = self.font.make_text_image(self.text, font_color, None, font_underline, font_italic, font_bold)
        self.image.color = self.font_color_inactive
        self.image.compile()
        self.size = self.image.get_size()
        if background_image:
            self.background, self.size, self.tsize, self.tshift = self.load_background(background_image)
        self.pack()

    def unfocus(self):
        Widget.unfocus(self)
        if self.image.color == self.font_color:
            self.image.color = self.font_color_inactive
    unfocus.__doc__ = Widget.unfocus.__doc__

    def focus(self):
        Widget.focus(self)
        if self.image.color == self.font_color_inactive:
            self.image.color = self.font_color
    focus.__doc__ = Widget.focus.__doc__

class Button(Widget):
    """A widget that renders text with a background, if spedified, for each state, regular, hover, click, and fires events when clicked."""
    widget_name = "Button"
    def __init__(self, app, text=tdef, pos=None, callbacks=[],
                 background_image=tdef, background_image_hover=tdef, background_image_click=tdef,
                 font=tdef, font_color=tdef, font_color_hover=tdef, font_color_click=tdef,
                 font_underline=tdef, font_italic=tdef, font_bold=tdef,
                 font_underline_hover=tdef, font_italic_hover=tdef, font_bold_hover=tdef,
                 font_underline_click=tdef, font_italic_click=tdef, font_bold_click=tdef,
                 image_border=tdef,
                 special_name=None):
        """Create the button
           app must be the App/Frame/Window this widget is attached to
           text must be tdef or the text of the label
           pos must be None (to use app.packer) or the (x,y) pos of the widget
           callbacks must be a list of functions or methods to call whent the button is clicked
           background_image must be tdef or the filename of the image to use as the background
           background_image_hover must be tdef or the filename of the image to use as the background when mouse is hovering
           background_image_click must be tdef or the filename of the image to use as the background when the button is clicked
           font must be tdef/None or the (Font, MEFont) fonts to use
           font_color must be tdef or the (R,G,B,A)(0-1) color of the text
           font_color_hover must be tdef or the (R,G,B,A)(0-1) color of the text when mouse is hovering
           font_color_click must be tdef or the (R,G,B,A)(0-1) color of the text when the button is clicked
           font_underline must be tdef or True/False - whether text is underlined
           font_underline_hover must be tdef or True/False - whether text is underlined when mouse is hovering
           font_underline_click must be tdef or True/False - whether text is underlined when the button is clicked
           font_italic must be tdef or True/False - whether text is italic
           font_italic_hover must be tdef or True/False - whether text is italic when mouse is hovering
           font_italic_click must be tdef or True/False - whether text is italic when button is clicked
           font_bold must be tdef or True/False - whether text is bold
           font_bold_italic must be tdef or True/False - whether text is bold when mouse is hovering
           font_bold_click must be tdef or True/False - whether text is bold when button is clicked
           image_border must be tdef or the pixel size of the border tiles of teh background image (if any)
           special_name must be None or the name used to grab a value different from widget_name from the theme"""
        Widget.__init__(self, app, pos, font, image_border, special_name)

        if text == tdef:
            text = self.theme.get(self, "text")
        if background_image == tdef:
            background_image = self.theme.get(self, "background-image")
        if background_image_hover == tdef:
            background_image_hover = self.theme.get(self, "background-image-hover")
        if background_image_click == tdef:
            background_image_click = self.theme.get(self, "background-image-click")
        if font_color == tdef:
            font_color = self.theme.get(self, "font-color")
        if font_color_hover == tdef:
            font_color_hover = self.theme.get(self, "font-color-hover")
        if font_color_click == tdef:
            font_color_click = self.theme.get(self, "font-color-click")
        if font_underline == tdef:
            font_underline = self.theme.get(self, "font-underline")
        if font_italic == tdef:
            font_italic = self.theme.get(self, "font-italic")
        if font_bold == tdef:
            font_bold = self.theme.get(self, "font-bold")
        if font_underline_hover == tdef:
            font_underline_hover = self.theme.get(self, "font-underline-hover")
        if font_italic_hover == tdef:
            font_italic_hover = self.theme.get(self, "font-italic-hover")
        if font_bold_hover == tdef:
            font_bold_hover = self.theme.get(self, "font-bold-hover")
        if font_underline_click == tdef:
            font_underline_click = self.theme.get(self, "font-underline-click")
        if font_italic_click == tdef:
            font_italic_click = self.theme.get(self, "font-italic-click")
        if font_bold_click == tdef:
            font_bold_click = self.theme.get(self, "font-bold-click")

        self.text = text
        self.ireg = self.font.make_text_image(self.text, font_color, None, font_underline, font_italic, font_bold)
        self.ihov = self.font.make_text_image(self.text, font_color_hover, None, font_underline_hover, font_italic_hover, font_bold_hover)
        self.icli = self.font.make_text_image(self.text, font_color_click, None, font_underline_click, font_italic_click, font_bold_click)
        self.ireg.compile()
        self.ihov.compile()
        self.icli.compile()
        self.image = self.ireg
        self.size = self.image.get_size()

        for i in callbacks:
            self.dispatch.bind("click", i)

        breg, bhov, bcli = background_image, background_image_hover, background_image_click
        if breg:
            self.breg, size, tsize, tshift = self.load_background(breg)
        else:
            self.breg, size, tsize, tshift = None, self.size, self.tsize, self.tshift
        if bhov:
            self.bhov, a, b, c = self.load_background(bhov)
        else:
            self.bhov = None
        if bcli:
            self.bcli, a, b, c = self.load_background(bcli)
        else:
            self.bcli = None
        self.size, self.tsize, self.tshift = size, tsize, tshift
        self.background = self.breg

        self.pack()

        self.handle_mousemotion((0,0)) #make sure we are set to hover at start if necessary!

    def render(self, offset=(0,0)):
        if self._mhover:
            if self._mhold:
                self.image = self.icli
                self.background = self.bcli
            else:
                self.image = self.ihov
                self.background = self.bhov
        else:
            self.image = self.ireg
            self.background = self.breg
        Widget.render(self, offset)
    render.__doc__ = Widget.render.__doc__

class Checkbox(Widget):
    """A widget that contains two states, which are swapped when clicked, and fires an event when changed."""
    widget_name = "Checkbox"
    def __init__(self, app, pos=None, background_image=tdef, check_image=tdef, font=tdef, image_border=tdef,
                 special_name=None):
        """Create the label
           app must be the App/Frame/Window this widget is attached to
           pos must be None (to use app.packer) or the (x,y) pos of the widget
           background_image must be tdef or the filename of the image to use as the image when state is 0
           check_image must be tdef or the filename of the image to use as the image when state is 1
           font must be tdef/None or the (Font, MEFont) fonts to use
           image_border must be tdef or the pixel size of the border tiles of teh background image (if any)
           special_name must be None or the name used to grab a value different from widget_name from the theme"""
        Widget.__init__(self, app, pos, font, image_border, special_name)

        if background_image == tdef:
            background_image = self.theme.get(self, "background-image")
        if check_image == tdef:
            check_image = self.theme.get(self, "check-image")

        off, on = background_image, check_image

        if off:
            try:
                self.off = _image.Image(self.theme.data(off))
            except:
                self.off = _image.Image(off)
        else:
            self.off = self.font.make_text_image("( )")
            self.off.compile()
        if on:
            try:
                self.on = _image.Image(self.theme.data(on))
            except:
                self.on = _image.Image(on)
        else:
            self.on = self.font.make_text_image("(!)")
            self.on.compile()
        self.image = self.off

        self.state = 0

        self.size = self.off.get_size()

        self.dispatch.bind("click", self._change_state)
        self.pack()

    def _change_state(self):
        """Change the state of the widget. Fire "change" event to dispatch."""
        self.state = abs(self.state-1)
        self.dispatch.fire("change", self.state)

    def render(self, offset):
        if self.state:
            self.image = self.on
        else:
            self.image = self.off
        Widget.render(self, offset)
    render.__doc__ = Widget.render.__doc__

class Radio(Frame):
    """A widget that contains several different checkboxes and labels, allowing you to pick one or another."""
    widget_name = "Radio"
    def __init__(self, app, pos=None, options=[],
                 background_image=tdef, option_background_image=tdef, option_check_image=tdef,
                 font_color=tdef, font_color_inactive=tdef, font=tdef,
                 font_underline=tdef, font_italic=tdef, font_bold=tdef,
                 image_border=tdef,
                 special_name=None):
        """Create the radio
           app must be the App/Frame/Window this widget is attached to
           pos must be None (to use app.packer) or the (x,y) pos of the widget
           options must be a list of string options for the widget
           background_image must be tdef or the filename of the image to use as the background
           option_background_image must be tdef or the filename of the image to use as the background for each option check
           option_check_image must be tdef or the filename of the image to use as the check_image for each option check
           font must be tdef/None or the (Font, MEFont) fonts to use
           font_color must be tdef or the (R,G,B,A)(0-1) color of the text
           font_color_inactive must be tdef or the (R,G,B,A)(0-1) color of the text when inactive
           font_underline must be tdef or True/False - whether text is underlined
           font_italic must be tdef or True/False - whether text is italic
           font_bold must be tdef or True/False - whether text is bold
           image_border must be tdef or the pixel size of the border tiles of teh background image (if any)
           special_name must be None or the name used to grab a value different from widget_name from the theme"""
        Frame.__init__(self, app, pos, tdef, None, font, image_border, special_name)
        self.packer.packtype = None

        if background_image == tdef:
            background_image = self.theme.get(self, "background-image")
        if option_background_image == tdef:
            option_background_image = self.theme.get(self, "option-background-image")
        if option_check_image == tdef:
            option_check_image = self.theme.get(self, "option-check-image")
        if font_color == tdef:
            font_color = self.theme.get(self, "font-color")
        if font_color_inactive == tdef:
            font_color_inactive = self.theme.get(self, "font-color-inactive")
        if font_underline == tdef:
            font_underline = self.theme.get(self, "font-underline")
        if font_italic == tdef:
            font_italic = self.theme.get(self, "font-italic")
        if font_bold == tdef:
            font_bold = self.theme.get(self, "font-bold")

        fu, fi, fb = font_underline, font_italic, font_bold

        self.options = []
        self.states = {}
        fc = font_color
        fc2 = font_color_inactive

        w = 0
        for i in options:
            c = Checkbox(self, background_image=option_background_image,
                         check_image=option_check_image)
            if not self.options:
                c.state = 1
            c.dispatch.bind("click", self.check_click)
            l = Label(self, i, background_image=None, font_color=fc, font_color_inactive=fc2, font_underline=fu,
                      font_italic=fi, font_bold=fb)
            l.dispatch.bind("click", self.check_click_label)
            NewLine(self)
            self.options.append([i, c, l, c.state])
            self.states[i] = c.state

            _w = l.pos[0]+l.size[0]
            if _w > w:
                w = _w
        h = max((c.pos[1]+c.size[1],
                 l.pos[1]+l.size[1]))

        self.size = (w, h)
        if background_image:
            self.background, self.size, self.tsize, self.tshift = self.load_background(background_image)
        self.pack()

    def check_click(self):
        """Checks which check/label was clicked, and sets that to active state, and fires "change" event to dispatch."""
        need_change = False
        for i in self.options:
            name, check, label, state = i
            if check.state != state: #changed!
                if check.state: #selected!
                    for x in self.options:
                        if not i == x:
                            x[1].state = 0
                            x[3] = 0
                            self.states[x[0]] = 0
                    state = 1
                else:
                    check.state = 1
                label.focus()
                check.focus()
                need_change = True
            i[0], i[1], i[2], i[3] = name, check, label, state
            self.states[name] = state
        if need_change:
            self.dispatch.fire("change", self.states)

    def check_click_label(self):
        """Checks whether a label was clicked - if so generates appropriate check click."""
        for i in self.options:
            name, check, label, state = i
            if label._mhover: #they were clicked ;)
                check.state = abs(check.state-1)
                self.check_click()

class MultiChoiceRadio(Radio):
    """A widget that contains several different checkboxes and labels, allowing you to pick several."""
    widget_name = "MultiChoiceRadio"
    def __init__(self, app, pos=None, options=[],
                 background_image=tdef, option_background_image=tdef, option_check_image=tdef,
                 font_color=tdef, font_color_inactive=tdef, font=tdef,
                 font_underline=tdef, font_italic=tdef, font_bold=tdef,
                 image_border=tdef,
                 special_name=None):
        Radio.__init__(self, app, pos, options, background_image,
                       option_background_image, option_check_image,
                       font_color, font_color_inactive, font,
                       font_underline, font_italic, font_bold,
                       image_border, special_name)
    __init__.__doc__ = Radio.__init__.__doc__

    def check_click(self):
        for i in self.options:
            name, check, label, state = i
            if check.state != state:
                label.focus()
            state = check.state
            i[0], i[1], i[2], i[3] = name, check, label, state
            self.states[name] = state
        self.dispatch.fire("change", self.states)
    check_click.__doc__ = Radio.check_click.__doc__

class Input(Widget):
    """A widget that allows you to capture keyboard input and convert it into text."""
    widget_name = "Input"
    def __init__(self, app, start_text=tdef, width=tdef, pos=None, background_image=tdef,
                 font_color=tdef, font_color_inactive=tdef, font=tdef,
                 font_underline=tdef, font_italic=tdef, font_bold=tdef,
                 image_border=tdef, callback=None,
                 special_name=None):
        """Create the input
           app must be the App/Frame/Window this widget is attached to
           start_text must be tdef or the text the input starts with
           width must be tdef or the max pixel width of the text input box
           pos must be None (to use app.packer) or the (x,y) pos of the widget
           background_image must be tdef or the filename of the image to use as the background
           font must be tdef/None or the (Font, MEFont) fonts to use
           font_color must be tdef or the (R,G,B,A)(0-1) color of the text
           font_color_inactive must be tdef or the (R,G,B,A)(0-1) color of the text when inactive
           font_underline must be tdef or True/False - whether text is underlined
           font_italic must be tdef or True/False - whether text is italic
           font_bold must be tdef or True/False - whether text is bold
           image_border must be tdef or the pixel size of the border tiles of teh background image (if any)
           special_name must be None or the name used to grab a value different from widget_name from the theme"""
        Widget.__init__(self, app, pos, font, image_border, special_name)

        if start_text == tdef:
            start_text = self.theme.get(self, "text")
        if width == tdef:
            width = self.theme.get(self, "width")
        if background_image == tdef:
            background_image = self.theme.get(self, "background-image")
        if font_color == tdef:
            font_color = self.theme.get(self, "font-color")
        if font_color_inactive == tdef:
            font_color_inactive = self.theme.get(self, "font-color-inactive")
        if font_underline == tdef:
            font_underline = self.theme.get(self, "font-underline")
        if font_italic == tdef:
            font_italic = self.theme.get(self, "font-italic")
        if font_bold == tdef:
            font_bold = self.theme.get(self, "font-bold")

        self.text = start_text
        self.image = self.mefont.make_text_image(self.text, font_color, None, font_underline, font_italic, font_bold)

        self.font_colors = (font_color, font_color_inactive)

        self.size = (width, self.mefont.pygame_font.get_height())

        self.cursor_pos = len(self.text)
        self.cursor_image = _image.Animation(((self.font.make_text_image("|",color=font_color), .5),
                                              (self.font.make_text_image("|",color=font_color_inactive), .5)))
        for i in self.cursor_image.frames:
            i[0].compile()
        self.cwidth = int(self.cursor_image.get_width()/2)
        self.xwidth = self.size[0] - self.cwidth*2
        if background_image:
            self.background, self.size, self.tsize, self.tshift = self.load_background(background_image)
        self.pack()

        if callback:
            self.dispatch.bind("submit", callback)

        self.calc_working_pos()

    def force_pos_update(self, pos):
        Widget.force_pos_update(self, pos)
        self.calc_working_pos()
    force_pos_update.__doc__ = Widget.force_pos_update.__doc__

    def can_handle_key(self, key, string):
        if string and string in self.mefont.acceptable:
            return True
        if key in (K_LEFT, K_RIGHT, K_END, K_HOME, K_DELETE,
                   K_BACKSPACE, K_RETURN):
            return True
        return False
    can_handle_key.__doc__ = Widget.can_handle_key.__doc__

    def submit_text(self):
        """Fire the "submit" event to dispatch, and clear the current text."""
        if self.text:
            self.dispatch.fire("submit", self.text)
        self.text = ""
        self.image.text = ""
        self.working = 0
        self.calc_working_pos()

    def move_cursor(self, x):
        """Move the text cursor position."""
        self.cursor_image.reset()
        self.cursor_pos += x
        if self.cursor_pos < 0:
            self.cursor_pos = 0
        if self.cursor_pos > len(self.text):
            self.cursor_pos = len(self.text)
        self.calc_working_pos()

    def handle_keydown(self, key, string):
        if self.can_handle_key(key, string):
            if self.key_active:
                if key == K_LEFT:
                    self.move_cursor(-1)
                elif key == K_RIGHT:
                    self.move_cursor(1)
                elif key == K_HOME:
                    self.cursor = 0
                elif key == K_END:
                    self.cursor = len(self.text)
                    self.calc_working_pos()
                elif key == K_DELETE:
                    self.text = self.text[0:self.cursor_pos]+self.text[self.cursor_pos+1::]
                    self.image.text = self.text
                    self.calc_working_pos()
                elif key == K_BACKSPACE:
                    if self.cursor_pos:
                        self.text = self.text[0:self.cursor_pos-1]+self.text[self.cursor_pos::]
                        self.move_cursor(-1)
                        self.image.text = self.text
                elif key == K_RETURN:
                    self.submit_text()
                else:
                    self.text = self.text[0:self.cursor_pos] + string + self.text[self.cursor_pos::]
                    self.image.text = self.text
                    self.move_cursor(1)
                return True
    handle_keydown.__doc__ = Widget.handle_keydown.__doc__

    def calc_working_pos(self):
        """Calculate the position of the text cursor - ie, where in the text are we typing... and the text offset."""
        tx, ty = self.pos
        if self.text and self.cursor_pos:
            g1 = self.image.glyphs[0:self.cursor_pos]
            g2 = self.image.glyphs[self.cursor_pos+1::]

            w1 = 0
            w2 = 0
            for i in g1:
                w1 += i.get_width()
            for i in g2:
                w2 += i.get_width()

            tp = tx + self.xwidth - w1 + self.tsize[0]
            if tp > self.pos[0]:
                tp = self.pos[0]

            cp = tp + w1

            self.wpos, self.tpos = (cp+self.cwidth+self.tsize[0], ty+self.tsize[1]),\
                                   (tp+self.cwidth*2+self.tsize[0], ty+self.tsize[1])
        else:
            self.wpos, self.tpos = (tx+self.tsize[0]-self.cwidth, ty+self.tsize[1]),\
                                   (tx+self.cwidth*2+self.tsize[0], ty+self.tsize[1])

    def focus(self):
        Widget.focus(self)
        self.cursor_image.reset()
    focus.__doc__ = Widget.focus.__doc__

    def render(self, offset=(0,0)):
        """Render the Input widget."""
        tpx, tpy = self.tpos
        tpx += offset[0]
        tpy += offset[1]
        if self.key_active:
            self.image.color = self.font_colors[0]
        else:
            self.image.color = self.font_colors[1]
        self.image.pos = (tpx, tpy)
        if self.background:
            bx, by = self.pos
            bx += offset[0]
            by += offset[1]
            self.background.pos = (bx, by)
            self.background.render()
            self.background.pos = self.pos
        view.screen.push_clip2d(*self.get_clip(offset))
        self.image.render()
        self.image.pos = self.tpos
        view.screen.pop_clip()
        if self.key_active:
            wpx, wpy = self.wpos
            wpx += offset[0]
            wpy += offset[1]
            self.cursor_image.pos = (wpx, wpy)
            self.cursor_image.render()
            self.cursor_image.pos = self.wpos
    render.__doc__ = Widget.render.__doc__

class MoveBar(Widget):
    """A widget that is basically like a label except it will move when clicked and held.
       Can have a child attached (any other widget, generally a Frame though) that moves with it."""
    widget_name = "MoveBar"
    def __init__(self, app, title=tdef, pos=(0,0), width=tdef, background_image=tdef,
                 font_color=tdef, font_color_inactive=tdef, font=tdef, child=None,
                 font_underline=tdef, font_italic=tdef, font_bold=tdef,
                 image_border=tdef,
                 special_name=None):
        """Create the movebar
           app must be the App/Frame/Window this widget is attached to
           title must be tdef or the text for the widget
           pos must be the (x,y) pos of the widget
           width must be tdef or the width of the widget
           background_image must be tdef or the filename of the image to use as the background
           font must be tdef/None or the (Font, MEFont) fonts to use
           font_color must be tdef or the (R,G,B,A)(0-1) color of the text
           font_color_inactive must be tdef or the (R,G,B,A)(0-1) color of the text when inactive
           font_underline must be tdef or True/False - whether text is underlined
           font_italic must be tdef or True/False - whether text is italic
           font_bold must be tdef or True/False - whether text is bold
           image_border must be tdef or the pixel size of the border tiles of teh background image (if any)
           special_name must be None or the name used to grab a value different from widget_name from the theme
           child must be None or the widget object that is bound to this movebar"""
        Widget.__init__(self, app, pos, font, image_border, special_name)
        self.override_pos = True #window is always overridden,sorry :P

        if title == tdef:
            title = self.theme.get(self, "title")
        if width == tdef:
            width = self.theme.get(self, "width")
        if background_image == tdef:
            background_image = self.theme.get(self, "background-image")
        if font_color == tdef:
            font_color = self.theme.get(self, "font-color")
        if font_color_inactive == tdef:
            font_color_inactive = self.theme.get(self, "font-color-inactive")
        if font_underline == tdef:
            font_underline = self.theme.get(self, "font-underline")
        if font_italic == tdef:
            font_italic = self.theme.get(self, "font-italic")
        if font_bold == tdef:
            font_bold = self.theme.get(self, "font-bold")

        self.font_color = font_color
        self.font_color_inactive = font_color_inactive

        self.title = title

        self.child = child
        if self.child:
            self.child.override_pos = True
            self.size = (self.child.size[0]-self.child.tsize[0]*2, self.mefont.pygame_font.get_height())
            self.child.pos = (self.pos[0], self.pos[1]+self.size[1])
        else:
            self.size = (width, self.mefont.pygame_font.get_height())

        if background_image:
            self.background, self.size, self.tsize, self.tshift = self.load_background(background_image)
        if self.child:
            x, y = self.child.pos
            y += self.tsize[1]*2-1
            self.child.pos = (x, y)

        i = self.font.make_text_image(title, font_color, font_underline, font_italic, font_bold)
        if i.get_width() > self.size[0] - self.tsize[0]*2:
            while title and self.font.make_text_image(title+"...", font_color, font_underline, font_italic, font_bold).get_width() >\
                  self.size[0] - self.tsize[0]*2:
                title = title[0:-1]
            i = self.font.make_text_image(title+"...", font_color, font_underline, font_italic, font_bold)
        self.image = i
        self.image.compile()
        self.pack()

    def handle_mousemotion(self, change):
        _retval = Widget.handle_mousemotion(self, change)
        if self._mhold:
            x, y = self.pos
            x += change[0]
            y += change[1]
            self.pos = (x, y)
            self._mhover = self._collidem()
            if self.child:
                x, y = self.child.pos
                x += change[0]
                y += change[1]
                self.child.pos = (x, y)
            return True
        return _retval
    handle_mousemotion.__doc__ = Widget.handle_mousemotion.__doc__

    def focus(self):
        self.image.colorize = self.font_color
        if self.child:
            self.child.focus()
        Widget.focus(self)
    focus.__doc__ = Widget.focus.__doc__

    def unfocus(self):
        self.image.colorize = self.font_color_inactive
        if not self.child == self.app.widgets[0]:
            Widget.unfocus(self)
    unfocus.__doc__ = Widget.unfocus.__doc__

class Window(MoveBar):
    """A widget that simply create a frame and a movebar to hold it, then keep strack of them."""
    widget_name = "Window"
    def __init__(self, app, title=tdef, pos=(0,0), size=tdef,
                 background_image=tdef, movebar_background_image=tdef,
                 font_color=tdef, font_color_inactive=tdef, font=tdef,
                 font_underline=tdef, font_italic=tdef, font_bold=tdef,
                 image_border=tdef,
                 special_name=None):
        """Create the movebar
           app must be the App/Frame/Window this widget is attached to
           title must be tdef or the text for the movebar
           size must be tdef or the (x,y) size of the frame (and x for the movebar)
           pos must be the (x,y) pos of the widget
           background_image must be tdef or the filename of the image to use as the background for the frame
           movebar_background_image must be tdef or the filename of the image to use as the background for the movebar
           font must be tdef/None or the (Font, MEFont) fonts to use
           font_color must be tdef or the (R,G,B,A)(0-1) color of the text
           font_color_inactive must be tdef or the (R,G,B,A)(0-1) color of the text when inactive
           font_underline must be tdef or True/False - whether text is underlined
           font_italic must be tdef or True/False - whether text is italic
           font_bold must be tdef or True/False - whether text is bold
           image_border must be tdef or the pixel size of the border tiles of teh background image (if any)
           special_name must be None or the name used to grab a value different from widget_name from the theme"""
        if title == tdef:
            title = app.theme.get(self, "title")
        if size == tdef:
            size = app.theme.get(self, "size")
        if background_image == tdef:
            background_image = app.theme.get(self, "background-image")
        if movebar_background_image == tdef:
            movebar_background_image = app.theme.get(self, "movebar-background-image")
        if font_color == tdef:
            font_color = app.theme.get(self, "font-color")
        if font_color_inactive == tdef:
            font_color_inactive = app.theme.get(self, "font-color-inactive")
        if font_underline == tdef:
            font_underline = app.theme.get(self, "font-underline")
        if font_italic == tdef:
            font_italic = app.theme.get(self, "font-italic")
        if font_bold == tdef:
            font_bold = app.theme.get(self, "font-bold")

        child = Frame(app, pos, size, background_image, font, image_border, special_name)
        MoveBar.__init__(self, app, title, pos, size[0], movebar_background_image,
                         font_color, font_color_inactive, font, child,
                         font_underline, font_italic, font_bold,
                         image_border, special_name)

        self.packer = self.child.packer
        self.fonts = self.app.fonts
        self.pack()

    def new_widget(self, widg):
        widg.app = self.child
        self.child.new_widget(widg)
    new_widget.__doc__ = Frame.new_widget.__doc__

    def update_fonts(self, fonts):
        self.fonts = fonts
    update_fonts.__doc__ = Frame.update_fonts.__doc__

    
class Menu(Button):
    """A widget that creates a menu, that allows submenus, and fires an event to the dispatch when one is selected."""
    widget_name = "Menu"
    def __init__(self, app, name=tdef, pos=None, options=[],
                 background_image=tdef, background_image_hover=tdef,
                 background_image_click=tdef, menu_background_image=tdef,
                 option_background_image=tdef, option_background_image_hover=tdef,
                 option_background_image_click=tdef,
                 sub_background_image=tdef, sub_background_image_hover=tdef,
                 sub_background_image_click=tdef,
                 font_color=tdef, font_color_hover=tdef, font_color_click=tdef,
                 option_font_color=tdef, option_font_color_hover=tdef, option_font_color_click=tdef,
                 sub_font_color=tdef, sub_font_color_hover=tdef, sub_font_color_click=tdef,
                 font=tdef,
                 font_underline=tdef, font_italic=tdef, font_bold=tdef,
                 font_underline_hover=tdef, font_italic_hover=tdef, font_bold_hover=tdef,
                 font_underline_click=tdef, font_italic_click=tdef, font_bold_click=tdef,
                 option_font_underline=tdef, option_font_italic=tdef, option_font_bold=tdef,
                 option_font_underline_hover=tdef, option_font_italic_hover=tdef, option_font_bold_hover=tdef,
                 option_font_underline_click=tdef, option_font_italic_click=tdef, option_font_bold_click=tdef,
                 sub_font_underline=tdef, sub_font_italic=tdef, sub_font_bold=tdef,
                 sub_font_underline_hover=tdef, sub_font_italic_hover=tdef, sub_font_bold_hover=tdef,
                 sub_font_underline_click=tdef, sub_font_italic_click=tdef, sub_font_bold_click=tdef,
                 sub_icon=tdef,
                 callback=None, image_border=tdef,
                 special_name=None):
        """Create the menu
           app must be the App/Frame/Window this widget is attached to
           name must be tdef or the name of the widget and the text for the menu button
           pos must be None (to use app.packer) or the (x,y) pos of the widget
           options must be a list of options in the (if an option is a list, the first value is considered the name of the sub-menu,
                                                     and the options for it - allows unlimited nesting)
           background_image must be tdef or the filename of the image to use as the background for the menu button
           background_image_hover must be tdef or the filename of the image to use as the background when mouse is hovering for the menu button
           background_image_click must be tdef or the filename of the image to use as the background when the button is clicked for the menu button
           option_background_image must be tdef or the filename of the image to use as the background for each menu option
           option_background_image_hover must be tdef or the filename of the image to use as the background when mouse is hovering for each menu option
           option_background_image_click must be tdef or the filename of the image to use as the background when the button is clicked for each menu option
           sub_background_image must be tdef or the filename of the image to use as the background for each sub-menu option
           sub_background_image_hover must be tdef or the filename of the image to use as the background when mouse is hovering for each sub-menu option
           sub_background_image_click must be tdef or the filename of the image to use as the background when the button is clicked for each sub-menu option
           font must be tdef/None or the (Font, MEFont) fonts to use
           font_color must be tdef or the (R,G,B,A)(0-1) color of the text for the menu button
           font_color_hover must be tdef or the (R,G,B,A)(0-1) color of the text when mouse is hovering for the menu button
           font_color_click must be tdef or the (R,G,B,A)(0-1) color of the text when the button is clicked for the menu button
           option_font_color must be tdef or the (R,G,B,A)(0-1) color of the text for the menu button
           option_font_color_hover must be tdef or the (R,G,B,A)(0-1) color of the text when mouse is hovering for each menu option
           option_font_color_click must be tdef or the (R,G,B,A)(0-1) color of the text when the button is clicked for each menu option
           sub_font_color must be tdef or the (R,G,B,A)(0-1) color of the text for each sub-menu option
           sub_font_color_hover must be tdef or the (R,G,B,A)(0-1) color of the text when mouse is hovering for each sub-menu option
           sub_color_click must be tdef or the (R,G,B,A)(0-1) color of the text when the button is clicked for each sub-menu option
           font_underline must be tdef or True/False - whether text is underlined for the menu button
           font_underline_hover must be tdef or True/False - whether text is underlined when mouse is hovering for the menu button
           font_underline_click must be tdef or True/False - whether text is underlined when the button is clicked for the menu button
           option_font_underline must be tdef or True/False - whether text is underlined for each menu option
           option_font_underline_hover must be tdef or True/False - whether text is underlined when mouse is hovering for each menu option
           option_font_underline_click must be tdef or True/False - whether text is underlined when the button is clicked for each menu option
           sub_font_underline must be tdef or True/False - whether text is underlined for each sub-menu option
           sub_font_underline_hover must be tdef or True/False - whether text is underlined when mouse is hovering for each sub-menu option
           sub_font_underline_click must be tdef or True/False - whether text is underlined when the button is clicked for each sub-menu option
           font_italic must be tdef or True/False - whether text is italic for the menu button
           font_italic_hover must be tdef or True/False - whether text is italic when mouse is hovering for the menu button
           font_italic_click must be tdef or True/False - whether text is italic when button is clicked for the menu button
           option_font_italic must be tdef or True/False - whether text is italic for each menu option
           option_font_italic_hover must tdef or be True/False - whether text is italic when mouse is hovering for each menu option
           option_font_italic_click must tdef or be True/False - whether text is italic when button is clicked for each menu option
           sub_font_italic must be tdef or True/False - whether text is italic for each sub-menu option
           sub_font_italic_hover must be tdef or True/False - whether text is italic when mouse is hovering for each sub-menu option
           sub_font_italic_click must be tdef or True/False - whether text is italic when button is clicked for each sub-menu option
           font_bold must be tdef or True/False - whether text is bold for the menu button
           font_bold_hover must be tdef or True/False - whether text is bold when mouse is hovering for the menu button
           font_bold_click must be tdef or True/False - whether text is bold when button is clicked for the menu button
           option_font_bold must be tdef or True/False - whether text is bold for each menu option
           option_font_bold_hover must be tdef or True/False - whether text is bold when mouse is hovering for each menu option
           option_font_bold_click must be tdef or True/False - whether text is bold when button is clicked for each menu option
           sub_font_bold must be tdef or True/False - whether text is bold for each sub-menu option
           sub_font_bold_hover must be tdef or True/False - whether text is bold when mouse is hovering for each sub-menu option
           sub_font_bold_click must be tdef or True/False - whether text is bold when button is clicked for each sub-menu option
           image_border must be tdef or the pixel size of the border tiles of teh background image (if any)
           special_name must be None or the name used to grab a value different from widget_name from the theme"""
        if name == tdef:
            name = app.theme.get(self, "name")
        if background_image == tdef:
            background_image = app.theme.get(self, "background-image")
        if background_image_hover == tdef:
            background_image_hover = app.theme.get(self, "background-image-hover")
        if background_image_click == tdef:
            background_image_click = app.theme.get(self, "background-image-click")
        if menu_background_image == tdef:
            menu_background_image = app.theme.get(self, "menu-background-image")
        if option_background_image == tdef:
            option_background_image = app.theme.get(self, "option-background-image")
        if option_background_image_hover == tdef:
            option_background_image_hover = app.theme.get(self, "option-background-image-hover")
        if option_background_image_click == tdef:
            option_background_image_click = app.theme.get(self, "option-background-image-click")
        if sub_background_image == tdef:
            sub_background_image = app.theme.get(self, "sub-background-image")
        if sub_background_image_hover == tdef:
            sub_background_image_hover = app.theme.get(self, "sub-background-image-hover")
        if sub_background_image_click == tdef:
            sub_background_image_click = app.theme.get(self, "sub-background-image-click")
        if font_color == tdef:
            font_color = app.theme.get(self, "font-color")
        if font_color_hover == tdef:
            font_color_hover = app.theme.get(self, "font-color-hover")
        if font_color_click == tdef:
            font_color_click = app.theme.get(self, "font-color-click")
        if option_font_color == tdef:
            option_font_color = app.theme.get(self, "option-font-color")
        if option_font_color_hover == tdef:
            option_font_color_hover = app.theme.get(self, "option-font-color-hover")
        if option_font_color_click == tdef:
            option_font_color_click = app.theme.get(self, "option-font-color-click")
        if sub_font_color == tdef:
            sub_font_color = app.theme.get(self, "sub-font-color")
        if sub_font_color_hover == tdef:
            sub_font_color_hover = app.theme.get(self, "sub-font-color-hover")
        if sub_font_color_click == tdef:
            sub_font_color_click = app.theme.get(self, "sub-font-color-click")
        if sub_icon == tdef:
            sub_icon = app.theme.get(self, "sub-icon")
            if sub_icon:
                sub_icon = app.theme.data(sub_icon)

        if font_underline == tdef:
            font_underline = app.theme.get(self, "font-underline")
        if font_italic == tdef:
            font_italic = app.theme.get(self, "font-italic")
        if font_bold == tdef:
            font_bold = app.theme.get(self, "font-bold")
        if font_underline_hover == tdef:
            font_underline_hover = app.theme.get(self, "font-underline-hover")
        if font_italic_hover == tdef:
            font_italic_hover = app.theme.get(self, "font-italic-hover")
        if font_bold_hover == tdef:
            font_bold_hover = app.theme.get(self, "font-bold-hover")
        if font_underline_click == tdef:
            font_underline_click = app.theme.get(self, "font-underline-click")
        if font_italic_click == tdef:
            font_italic_click = app.theme.get(self, "font-italic-click")
        if font_bold_click == tdef:
            font_bold_click = app.theme.get(self, "font-bold-click")

        if option_font_underline == tdef:
            option_font_underline = app.theme.get(self, "option-font-underline")
        if option_font_italic == tdef:
            option_font_italic = app.theme.get(self, "option-font-italic")
        if option_font_bold == tdef:
            option_font_bold = app.theme.get(self, "option-font-bold")
        if option_font_underline_hover == tdef:
            option_font_underline_hover = app.theme.get(self, "option-font-underline-hover")
        if option_font_italic_hover == tdef:
            option_font_italic_hover = app.theme.get(self, "option-font-italic-hover")
        if option_font_bold_hover == tdef:
            option_font_bold_hover = app.theme.get(self, "option-font-bold-hover")
        if option_font_underline_click == tdef:
            option_font_underline_click = app.theme.get(self, "option-font-underline-click")
        if option_font_italic_click == tdef:
            option_font_italic_click = app.theme.get(self, "option-font-italic-click")
        if option_font_bold_click == tdef:
            option_font_bold_click = app.theme.get(self, "option-font-bold-click")

        if sub_font_underline == tdef:
            sub_font_underline = app.theme.get(self, "sub-font-underline")
        if sub_font_italic == tdef:
            sub_font_italic = app.theme.get(self, "sub-font-italic")
        if sub_font_bold == tdef:
            sub_font_bold = app.theme.get(self, "sub-font-bold")
        if sub_font_underline_hover == tdef:
            sub_font_underline_hover = app.theme.get(self, "sub-font-underline-hover")
        if sub_font_italic_hover == tdef:
            sub_font_italic_hover = app.theme.get(self, "sub-font-italic-hover")
        if sub_font_bold_hover == tdef:
            sub_font_bold_hover = app.theme.get(self, "sub-font-bold-hover")
        if sub_font_underline_click == tdef:
            sub_font_underline_click = app.theme.get(self, "sub-font-underline-click")
        if sub_font_italic_click == tdef:
            sub_font_italic_click = app.theme.get(self, "sub-font-italic-click")
        if sub_font_bold_click == tdef:
            sub_font_bold_click = app.theme.get(self, "sub-font-bold-click")

        Button.__init__(self, app, name, pos, [],
                        background_image, background_image_hover, background_image_click,
                        font, font_color, font_color_hover, font_color_click,
                        font_underline, font_italic, font_bold,
                        font_underline_hover, font_italic_hover, font_bold_hover,
                        font_underline_click, font_italic_click, font_bold_click,
                        image_border, special_name)
        self.dispatch.bind("click", self.do_visible)

        self.frames = []
        self.cur_frame = 0

        self.sub_icon = sub_icon

        images = (menu_background_image,
                  option_background_image,
                  option_background_image_hover,
                  option_background_image_click)
        subimages = (sub_background_image,
                     sub_background_image_hover,
                     sub_background_image_click)
        font_colors = (option_font_color,
                       option_font_color_hover,
                       option_font_color_click)
        sub_font_colors = (sub_font_color,
                           sub_font_color_hover,
                           sub_font_color_click)
        font_states = (option_font_underline, option_font_italic, option_font_bold,
                       option_font_underline_hover, option_font_italic_hover, option_font_bold_hover,
                       option_font_underline_click, option_font_italic_click, option_font_bold_click)

        sub_font_states = (sub_font_underline, sub_font_italic, sub_font_bold,
                           sub_font_underline_hover, sub_font_italic_hover, sub_font_bold_hover,
                           sub_font_underline_click, sub_font_italic_click, sub_font_bold_click)

        self.add_frame("", options, images, font_colors, font, font_states, subimages, sub_font_colors, sub_font_states)

        if callback:
            self.dispatch.bind("menu-click", callback)

        self.skip_click = False

    def get_pos_for_frames(self, frame):
        x, y = self.pos
        y += self.size[1]
        app = self.app
        while hasattr(app, "app"):
            a, b = app.pos
            c, d = app.tsize
            x += a + c
            y += b + d
            app = app.app

        sx, sy = self.get_root_app().size

        right = x + frame.size[0]
        if right > sx:
            right = sx
        bottom = y + frame.size[1]
        if bottom > sy:
            bottom = sy
        return right-frame.size[0], bottom-frame.size[1]

    def add_frame(self, name, options, images, fc, ff, fs, ssi, sfc, sfs):
        """Build frames for each menu/sun-menu, and entries."""
        goback = int(self.cur_frame)
        frame = Frame(self.get_root_app(), (0,0), background_image=images[0], font=ff,
                      image_border=self.image_border)
        frame.packer.packtype = None
        frame.visible = False
        frame.dispatch.bind("unfocus", self.do_unfocus)
        self.frames.append(frame)

        fu, fi, fb, fuh, fih, fbh, fuc, fic, fbc = fs

        self.cur_frame = len(self.frames)-1

        bimages = images[1::]
        simages = ssi
        sfu, sfi, sfb, sfuh, sfih, sfbh, sfuc, sfic, sfbc = sfs
        need_space = False
        space_size = 0

        w = 0
        incw = 0
        if not frame == self.frames[0]:
            if self.sub_icon:
                ic = pygame.image.load(self.sub_icon).convert_alpha()
                ic = pygame.transform.flip(ic, True, False)
            else:
                ic = self.mefont.glyphs[""]["<"].copy()
            x = Icon(frame, image=ic)
            Spacer(frame, (1,0))
            need_space = True
            space_size = x.size[0]+1
            c = Button(frame, name.split(".")[-1], background_image=simages[0],
                       background_image_hover=simages[1],
                       background_image_click=simages[2],
                       font_color=sfc[0], font_color_hover=sfc[1],
                       font_color_click=sfc[2], font=ff,
                       font_underline=sfu, font_italic=sfi, font_bold=sfb,
                       font_underline_hover=sfuh, font_italic_hover=sfih, font_bold_hover=sfbh,
                       font_underline_click=sfuc, font_italic_click=sfic, font_bold_click=sfbc,
                       image_border=self.image_border)
            NewLine(frame)
            w = c.size[0]+space_size
            c.dispatch.bind("click", self.swap_frame(goback))

        for i in options:
            if type(i) is type(""):
                if need_space:
                    Spacer(frame, (space_size, 0))
                c = Button(frame, i, background_image=bimages[0],
                           background_image_hover=bimages[1],
                           background_image_click=bimages[2],
                           font_color=fc[0], font_color_hover=fc[1],
                           font_color_click=fc[2], font=ff,
                           font_underline=fu, font_italic=fi, font_bold=fb,
                           font_underline_hover=fuh, font_italic_hover=fih, font_bold_hover=fbh,
                           font_underline_click=fuc, font_italic_click=fic, font_bold_click=fbc,
                           image_border=self.image_border)
                NewLine(frame)
                if c.size[0]+space_size > w:
                    w = c.size[0]+space_size
                if name:
                    ni = name+"."+i
                else:
                    ni = i
                c.dispatch.bind("click", self.bind_to_event(ni))
                c.dispatch.bind("click", self.do_unfocus)
            else:
                if need_space:
                    Spacer(frame, (space_size, 0))
                c = Button(frame, i[0], background_image=simages[0],
                           background_image_hover=simages[1],
                           background_image_click=simages[2],
                           font_color=sfc[0], font_color_hover=sfc[1],
                           font_color_click=sfc[2], font=ff,
                           font_underline=sfu, font_italic=sfi, font_bold=sfb,
                           font_underline_hover=sfuh, font_italic_hover=sfih, font_bold_hover=sfbh,
                           font_underline_click=sfuc, font_italic_click=sfic, font_bold_click=sfbc,
                           image_border=self.image_border)
                if self.sub_icon:
                    ic = self.sub_icon
                else:
                    ic = self.mefont.glyphs[""][">"].copy()
                x = Icon(frame, image=ic)
                NewLine(frame)
                if c.size[0]+space_size > w:
                    w = c.size[0]+space_size
                if x.size[0] > incw:
                    incw = x.size[0]
                c.dispatch.bind("click", self.swap_frame(self.cur_frame+1))
                if name:
                    ni = name+"."+i[0]
                else:
                    ni = i[0]
                self.add_frame(ni, i[1::], images, fc, ff, fs, ssi, sfc, sfs)
        if options:
            h = c.pos[1]+c.size[1]
        else:
            h = 1
        for i in frame.widgets:
            if not (isinstance(i, NewLine) or\
                    isinstance(i, Icon) or\
                    isinstance(i, Spacer)):
                i.size = w-i.tsize[0]*2, i.size[1]-i.tsize[1]*2
                _size = 0
                if i.breg: i.breg, _size, a, i.tshift = i.load_background(bimages[0])
                if i.bhov: i.bhov, c, a, b = i.load_background(bimages[1])
                if i.bcli: i.bcli, c, a, b = i.load_background(bimages[2])
                if _size:
                    i.size = _size
        i.pack()
        for i in frame.widgets:
            if isinstance(i, Icon):
                _x, _y = i.pos
                _h = i.size[1]
                if need_space:
                    _b = frame.widgets[-3].size[1]
                else:
                    _b = frame.widgets[-1].size[1]
                _y += int((_b-_h)/2)
                i.pos = (_x+1, _y)

        frame.size = (w+incw+1+space_size, h)
        if images[0]:
            frame.background, frame.size, frame.tsize, frame.tshift = frame.load_background(images[0])

        x, y = frame.pos
        while x + frame.size[0] > frame.app.size[0]:
            x -= 1
        while y + frame.size[1] > frame.app.size[1]:
            y -= 1
        frame.pos = (x, y)
        self.cur_frame = goback

    def do_swap_frame(self, num):
        """Swap current frame to frame number num."""
        self.cur_frame = num
        for i in self.frames:
            i.visible = False
        self.frames[num].visible = True
        self.frames[num].focus()
        self.frames[num].pos = self.get_pos_for_frames(self.frames[num])

    def swap_frame(self, num):
        """Return a function that when called calls do_swap_frame(num)"""
        def do():
            self.do_swap_frame(num)
        return do

    def do_visible(self):
        """Set the top level frame to visible."""
        if self.skip_click:
            self.skip_click = False
            return
        self.do_swap_frame(0)

    def bind_to_event(self, name):
        """Return funtction that when called fire a "menu-click" even with arg name."""
        def send():
            self.dispatch.fire("menu-click", name)
        return send

    def do_unfocus(self):
        """Unfocus the widget, and all frames."""
        for i in self.frames:
            i.visible = False
            i.key_active=False
            i.key_hold_lengths = {}
            i._mhold=False
            i._mhover=False
        self.cur_frame = 0
        self.unfocus()
        if self._mhover:
            self.skip_click = True

    def unfocus(self):
        if not self.frames[self.cur_frame].visible:
            Widget.unfocus(self)
            self._mhover = self._collidem()
    unfocus.__doc__ = Button.unfocus.__doc__
