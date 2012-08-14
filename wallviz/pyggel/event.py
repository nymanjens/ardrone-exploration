"""
pyggle.event
This library (PYGGEL) is licensed under the LGPL by Matthew Roe and PYGGEL contributors.

The event module contains classes to grab and access events.
"""

from include import *
import view
import string
import time

class Keyboard(object):
    """A simple class to store keyboard events."""
    def __init__(self):
        """Create the holder.
           Attributes:
               active -> a list of all keys hit or held
               hit -> a list of all keys hit
               held -> a list of all keys held"""
        self.active = []
        self.hook = {}

        self.hit = []
        self.held = []

    def get_ident(self, event):
        try:
            return str(event.unicode)
        except:
            return event.unicode

    def do_active_hit(self, event):
        self.hook[event.key] = self.get_ident(event)
        if not event.key in self.active:
            self.active.append(event.key)
            self.active.append(self.get_ident(event))
            self.hit.append(event.key)
            self.hit.append(self.get_ident(event))

    def do_keyup(self, event):
        if event.key in self.active:
            self.active.remove(event.key)
            self.active.remove(self.hook[event.key])
            x = self.hook[event.key]
            del self.hook[event.key]
            return x

class Mouse(object):
    """A simple class to store mouse events."""
    all_names = {1:"left", 2:"middle", 3:"right", 4:"wheel-up", 5:"wheel-down"}
    def __init__(self):
        """Create the holder.
           Attributes:
               active -> a list of all mouse buttons that were clicked or held
               hit -> a list of all mouse buttons that were clicked
               held -> a list of all mouse buttons that were held"""
        self.active = []
        self.motion = [0,0]

        self.hit = []
        self.held = []

    def get_pos(self):
        """Return the mouse pos."""
        return view.screen.get_mouse_pos()

    def get_name(self, button):
        """Return the 'name' that matches the button, ie:
           1 -> left
           2 -> middle
           3 -> right
           4 -> wheel-up
           5 -> wheel-down"""
        if button in self.all_names:
            return self.all_names[button]
        return "extra-%s"%button

    def do_active_hit(self, event):
        """Add a hit event."""
        if not event.button in self.active:
            name = self.get_name(event.button)
            self.active.append(event.button)
            self.active.append(name)
            self.hit.append(event.button)
            self.hit.append(name)

    def do_buttonup(self, event):
        """Remove a button from active list."""
        if event.button in self.active:
            name = self.get_name(event.button)
            self.active.remove(event.button)
            self.active.remove(name)

class Dispatcher(object):
    """A simple dispatcher class, that allows you to bind functions to events, and execute them all with a single command."""
    def __init__(self):
        """Create the Dispatcher object."""
        self.name_bindings = {}

    def bind(self, name, function):
        """Bind 'function' to the event 'name'.
           name can be anything that works as a python dict key (string, number, etc.)
           function must be a python function or method"""
        if name in self.name_bindings:
            self.name_bindings[name].append(function)
        else:
            self.name_bindings[name] = [function]

    def fire(self, name, *args, **kwargs):
        """Execute command 'name', calls any functions bound to this event with args/kwargs.
           name can be anything that works as a python dict key (string, number, etc.)
           *args/**kwargs are the arguments to use on any function calls bound to this event"""
        if name in self.name_bindings:
            for func in self.name_bindings[name]:
                func(*args, **kwargs)

class Handler(object):
    """A simple event handler. This object catches and stores events, as well as fire off any callbacks attached to them.
       There should only ever be one Handler in use at once, as only one handler can get a specific event.
       If a gui is used, it will "take control" of teh event handler, ie,
           any events it catches will be suppressed here (they can still be accessed at gui_keyboard/gui_mouse)
           no callbacks will be fired, no values set - the only exceptions are:
               quit, update, mouseup, and keyup - these values will be set, but no callbacks will be fired."""
    def __init__(self):
        """Create the handler.
           Attributes:
               keyboard -> a Keyboard object storing keyboard events
               mouse -> a Mouse object storing mouse events
               quit -> bool - whether wuit signal has been sent
               dispatch -> Dispatcher object used for firing callbacks
               uncaught_events -> list of all events the Handler couldn't handle"""
        self.keyboard = Keyboard()
        self.mouse = Mouse()
        self.quit = False

        self.dispatch = Dispatcher()

        self.uncaught_events = []

        self.all_guis = []
        self.gui = None
        self.gui_keyboard = Keyboard()
        self.gui_mouse = Mouse()
        self.gui_uncaught_events = []

    def bind_to_event(self, event, function):
        """Bind a callback function to an event.
           event must be the name of an input event, event names are:
               keydown - when a key is pressed
               keyup - when a key is released
               keyhold - when a mouse key is held
               keyactive - when a mouse key is active
               mousedown - when a mouse button is pressed
               mouseup - when a mouse button is released
               mousehold - when a mouse button is held
               mouseactive - when a mouse button is active
               quit - when the QUIT event was fired (ie the X box on the window is hit)
               uncaught-event - when an unsupported event is fired
               update - called at end of grabbing events/firing callbacks.
           function must be a python function or method that accepts the proper args for each event,
           event args are:
               keydown, keyup, keyhold: key->Pygame event key, string->the Python str of the key, or the unicode of the key
                   string will be the key pressed, ie, the a key is "a" (or "A" with shift/caps)
               mousedown, mouseup, mousehold: button->Pygame event button, string-> the "name" of the button
                   string will be "left", "right", "middle", "wheel-up", "wheel-down", or "extra-N" where N is the Pygame event button
               uncaught-event: event->the Pygame event
               quit, update: None"""
        self.dispatch.bind(event, function)

    def replace_event(self, event, function):
        """This is the same as bind_to_event, except that it forces function to be the only one attached to the event,
           instead of allowing several."""
        self.dispatch.name_bindings[event] = []
        self.bind_to_event(event, function)

    def handle_event(self, event):
        """Handle an event, store in proper object, and fire callbacks."""
        if event.type == KEYDOWN:
            if self.gui and self.gui.handle_keydown(event.key, str(event.unicode)):
                self.gui_keyboard.do_active_hit(event)
                return None
            self.keyboard.do_active_hit(event)

            self.dispatch.fire("keydown", event.key,
                               self.keyboard.get_ident(event))

        elif event.type == KEYUP:
            x = self.keyboard.do_keyup(event)
            xb = self.gui_keyboard.do_keyup(event)
            if self.gui and self.gui.handle_keyup(event.key, xb):
                return None
            if x or xb:
                self.dispatch.fire("keyup", event.key, x)
            else:
                if self.gui and self.gui.handle_uncaught_event(event):
                    self.gui_uncaught_events.append(event)
                    return None
                self.uncaught_events.append(event)
                self.dispatch.fire("uncaught-event", event)

        elif event.type == MOUSEBUTTONDOWN:
            name = self.mouse.get_name(event.button)
            if self.gui and self.gui.handle_mousedown(event.button, name):
                self.gui_mouse.do_active_hit(event)
                return None
            self.mouse.do_active_hit(event)
            self.dispatch.fire("mousedown", event.button, name)

        elif event.type == MOUSEBUTTONUP:
            self.gui_mouse.do_buttonup(event)
            self.mouse.do_buttonup(event)
            name = self.mouse.get_name(event.button)
            if self.gui and self.gui.handle_mouseup(event.button, name):
                return None
            self.dispatch.fire("mouseup", event.button, name)

        elif event.type == MOUSEMOTION:
            self.gui_mouse.motion[0] += event.rel[0]
            self.gui_mouse.motion[1] += event.rel[1]
            if self.gui and self.gui.handle_mousemotion(event.rel):
                return None
            self.dispatch.fire("mousemotion", event.rel)
            self.mouse.motion[0] += event.rel[0]
            self.mouse.motion[1] += event.rel[1]
            
        elif event.type == QUIT:
            self.quit = True
            self.dispatch.fire("quit")

        else:
            if self.gui and self.gui.handle_uncaught_event(event):
                self.gui_uncaught_events.append(event)
                return None
            self.uncaught_events.append(event)
            self.dispatch.fire("uncaught-event", event)

    def update(self):
        """Grab all events, store in proper objects, and fire callbacks where necessary."""
        self.keyboard.hit = []
        self.mouse.hit = []
        self.mouse.motion = [0,0]
        self.gui_mouse.motion = [0,0]
        self.uncaught_events = []
        self.gui_uncaught_events = []
        self.gui_keyboard.hit = []
        self.gui_mouse.hit = []
        self.keyboard.held = []
        self.gui_keyboard.held = []
        self.mouse.held = []
        self.gui_mouse.held = []
        for event in pygame.event.get():
            self.handle_event(event)

        for i in self.keyboard.active:
            if not i in self.keyboard.hit:
                self.keyboard.held.append(i) #regardless of type now!
                if i in self.keyboard.hook: #make sure these aren't the string names! Or else we would double fire, potentially
                    eventkey = i
                    name = self.keyboard.hook[eventkey]
                    self.dispatch.fire("keyhold", eventkey, name)
                    self.dispatch.fire("keyactive", eventkey, name)
        for i in self.mouse.active:
            if not i in self.mouse.hit:
                self.mouse.held.append(i)
                if type(i) is type(1): #same thing as keys, only slightly different test!
                    self.dispatch.fire("mousehold", i, self.mouse.get_name(i))
                    self.dispatch.fire("mouseactive", i, self.mouse.get_name(i))

        for i in self.gui_keyboard.active:
            if not i in self.gui_keyboard.hit:
                self.gui_keyboard.held.append(i) #regardless of type now!
                if i in self.gui_keyboard.hook: #make sure these aren't the string names! Or else we would double fire, potentially
                    eventkey = i
                    name = self.gui_keyboard.hook[eventkey]
                    if self.gui:
                        self.gui.handle_keyhold(eventkey, name)
        for i in self.gui_mouse.active:
            if not i in self.gui_mouse.hit:
                self.gui_mouse.held.append(i)
                if type(i) is type(1): #same thing as keys, only slightly different test!
                    if self.gui:
                        self.gui.handle_mousehold(i, self.gui_mouse.get_name(i))
        self.dispatch.fire("update")
