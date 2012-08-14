#!/usr/bin/python
""" imports """
# std imports
import random, sys, argparse
from math import *
import numpy as np
from numpy import array
from numpy.linalg import norm
# ROS imports
import roslib; roslib.load_manifest('wallviz'); import rospy
from std_msgs.msg import String
# custom imports
import pyggel
from pyggel import *
from pyggelcustomshapes.wall import Wall
from pyggelcustomshapes.floor import Floor
import mapmaker
from mapmaker.mapmaker import MapMaker
from utility import MaxOneProcess

""" settings """
WALL_SCALE_PARAM = 1
WALL_HEIGHT = MapMaker.WALL_HEIGHT * WALL_SCALE_PARAM
WALL_WIDTH_UNSCALED = MapMaker.WALL_HEIGHT / 15.
WALL_WIDTH = WALL_WIDTH_UNSCALED * WALL_SCALE_PARAM

""" constants """
LEFTMOUSEBUTTON = 1
MIDDLEMOUSEBUTTON = 2
RIGHTMOUSEBUTTON = 3
MOUSEWHEELUP = 4
MOUSEWHEELDOWN = 5
MAX_MOUSE_DIFF = 290

""" globals """
game = None

class World:
    refresh_rooms = False
    
    def __init__(self, rooms):
        self.initialise()
        self.setupValues()
        #self.addFixedModels()
        self.addRooms(rooms)
    
    def refresh(self, rooms):
        self.removeRooms()
        self.addRooms(rooms)
    
    def initialise(self):
        # initialize pygel screen
        pyggel.init(screen_size=(800,600))
        # create pygel scene
        self.scene = pyggel.scene.Scene()
        # Set window title
        pyggel.view.set_title("wallviz")
        # create a pygel light
        self.light3 = pyggel.light.Light((0,10,0),
                          (0.8,0.8,0.8,1),# ambient color
                          (1,1,1,1),# diffuse color
                          (1,1,1,1),# specular
                          (0,0,0),# spot position
                          True) # directional, not a spot light
        self.scene.add_light(self.light3)
        
        # create cameras
        self.camera3rdP = pyggel.camera.LookAtCamera([0,0,0], distance=6)
        self.camera3rdP.rotx = -45
        self.camera3rdP.roty = 45
        self.cameraFPS = pyggel.camera.LookFromCamera([0,WALL_SCALE_PARAM,0])
        self.camera = self.cameraFPS
        # setup pygel event handler
        self.event_handler = pyggel.event.Handler()

    def setupValues(self):
        self.scene.pick = True
        self.events = pyggel.event.Handler()
        self.clock = pygame.time.Clock()
        self.grid = []

    def addToScene(self, x):
        self.scene.add_3d(x)
        self.room_objects.append(x)
    
    def addRooms(self, rooms):
        # load walls
        self.room_objects = []
        for room_num, (walls, floorF) in enumerate(rooms):
            for i, (E0, E1) in enumerate(walls):
                fname = MapMaker.wallPathFromIndex(room_num, i)
                dE = E1 - E0
                dE *= 1e-3
                wall = Wall(E0+dE, E1-dE, fname, height=WALL_HEIGHT, width=WALL_WIDTH)
                self.addToScene(wall)
            floor = Floor(*floorF, texture=MapMaker.floorPath(room_num))
            self.addToScene(floor)
    
    def removeRooms(self):
        for obj in self.room_objects:
            self.scene.remove_3d(obj)

    def rotateCamera(self,rotation):
        """rotate camera based on mouse movement"""
        if self.camera is self.cameraFPS:
            rotation = -.6*array(rotation)
        x,y,z = rotation
        self.camera.rotx += x
        self.camera.roty += y
        self.camera.rotz += z

    def moveCamera(self, dP):
        """ move camera based on mouse movement"""
        cam = self.camera
        if cam is self.camera3rdP:
            dP = array(dP) * sqrt(self.camera.distance) / 90.
            theta = -cam.roty * pi / 180
            R = np.matrix([
                [cos(theta), -sin(theta)],
                [-sin(theta), -cos(theta)],
            ])
        else:
            dP = array(dP) * sqrt(cam.posy) / 90.
            theta = cam.roty * pi / 180
            R = np.matrix([
                [cos(theta), -sin(theta)],
                [-sin(theta), -cos(theta)],
            ])
        dP = R * array([dP.tolist()]).T
        dx, dy = array(dP).squeeze()
        cam.posx += dx
        cam.posz += dy
            

    def changeCameraDistance(self, diff):
        cam = self.camera
        if cam is self.camera3rdP:
            dist = cam.distance
            dist *= exp(diff/10.)
            dist = dist if dist > 1 else 1
            cam.distance = dist
        else:
            h = cam.posy
            h *= exp(-diff/10.)
            h = h if h > 1 else 1
            cam.posy = h

    def switchCamera(self):
        current = self.camera
        next = self.camera3rdP if current is self.cameraFPS else self.cameraFPS
        self.camera = next
    
    def updateMouse(self):
        fps = self.camera is self.cameraFPS
        pygame.mouse.set_visible(not fps)
        if fps and norm(array(pygame.mouse.get_pos()) - array([400,300])) > MAX_MOUSE_DIFF:
            pygame.mouse.set_pos([400,300])
    
    def processInput(self):
        """Get user input"""
        change = False
        keyboard = self.events.keyboard
        mousemoveX=None; mouseMoveY = None
        mouseX, mouseY = pygame.mouse.get_pos()
        mouseMoveX, mouseMoveY = pygame.mouse.get_rel()
        self.events.update()
        # check max mouseMove
        if norm(array([mouseMoveX, mouseMoveY])) > MAX_MOUSE_DIFF-150:
            mouseMoveX = mouseMoveY = 0
        # chck for quit
        if self.events.quit or K_ESCAPE in keyboard.hit:
            pyggel.view.clear_screen()
            pyggel.view.refresh_screen()
            pyggel.quit()
            sys.exit(0)
        # if 1 mouse button is held down
        if len(self.events.mouse.held)==2:
            # if right mouse button is held down
            mousebutton = self.events.mouse.held[0]
            mousedown = True
        else:
            mousebutton = None
            mousedown = False
        # if 2 mouse buttons are held down
        if len(self.events.mouse.held)==4:
            twomousebuttons = True
        else:
            twomousebuttons = False
        # mousehit
        mousehit = self.events.mouse.hit[0] if len(self.events.mouse.hit) else 0
        # tab
        tabhit = K_TAB in keyboard.hit
        # arrow keys
        movex = movey = 0
        if K_LEFT in keyboard.active or K_q in keyboard.active:
            movex = -1
        if K_RIGHT in keyboard.active or K_d in keyboard.active:
            movex = 1
        if K_UP in keyboard.active or K_z in keyboard.active:
            movey = 1
        if K_DOWN in keyboard.active or K_s in keyboard.active:
            movey = -1
        
        """ Process user input """
        # rotate third person camera
        rotx=0; roty=0; rotz=0
        if (mousebutton == LEFTMOUSEBUTTON and mousedown) or (not mousedown and self.camera is self.cameraFPS):
            if mouseMoveX:
                roty = -0.5*mouseMoveX
                change = True
            if mouseMoveY:
                rotx = -0.5*mouseMoveY
                change = True
        self.rotateCamera((rotx,roty,rotz))
        # move third person camera
        dx = dy = 0
        if mousebutton == MIDDLEMOUSEBUTTON and mousedown and mouseMoveX:
            dx = -0.5*mouseMoveX
            change = True
        if mousebutton == MIDDLEMOUSEBUTTON and mousedown and mouseMoveY:
            dy = -0.5*mouseMoveY
            change = True
        self.moveCamera((dx,dy))
        # move camera distance
        if mousebutton == RIGHTMOUSEBUTTON and mouseMoveY:
            self.changeCameraDistance(0.05*mouseMoveY)
            change = True
        # move camera distance
        if mousehit == MOUSEWHEELUP:
            self.changeCameraDistance(-3)
            change = True
        if mousehit == MOUSEWHEELDOWN:
            self.changeCameraDistance(3)
            change = True
        # switch camera view
        if tabhit:
            self.switchCamera()
            change = True
        # move around
        if movex or movey:
            change = True
        self.moveCamera(WALL_SCALE_PARAM * 4 * array([movex, -movey]))
        
        return change

    def run(self):
        """game main loop"""
        # render scene
        self.scene.render(self.camera)
        # flip the display buffer
        pyggel.view.refresh_screen()
        while True:
            # limit frames per second
            self.clock.tick(50)
            # get user input
            change = self.processInput()
            # update mouse
            self.updateMouse()
            # check for ROS updates
            if self.refresh_rooms:
                rooms = self.refresh_rooms
                self.refresh_rooms = False
                self.refresh(rooms)
                change = True
            if change:
                # clear screen for new drawing
                pyggel.view.clear_screen()
                # render scene
                self.scene.render(self.camera)
                # flip the display buffer
                pyggel.view.refresh_screen()

def update_map(_):
    print "\n[*] Update signal received"
    
    ### get map ###
    rooms = MapMaker().makeMap(WALL_WIDTH_UNSCALED, build_texture=True)
    rooms = [ [WALL_SCALE_PARAM * x for x in room] for room in rooms] # apply scale param
    
    ### refresh visualization ###
    global game
    game.refresh_rooms = rooms

if __name__ == '__main__':
    ### optional psyco optimization ###
    try: import psyco; psyco.full()
    except: pass
    
    ### parse args ###
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--remake-textrue', action='store_true', dest='remake_texture')
    parser.add_argument('-d', '--nodebugplot', action='store_true')
    args = parser.parse_args()
    mapmaker.mapmaker.projectionlib.DEBUGPLOT = not args.nodebugplot
    
    ### get map ###
    rooms = MapMaker().makeMap(WALL_WIDTH_UNSCALED, build_texture=args.remake_texture)
    # apply scale param
    rooms = [ [WALL_SCALE_PARAM * x for x in room] for room in rooms]
    
    ### subscribe to ROS topic ###
    rospy.init_node('wallviz')
    rospy.Subscriber("/wallviz/update", String, MaxOneProcess(update_map).startProcess)
    
    ### build and run 3D visualization ###
    game = World(rooms)
    game.run()




