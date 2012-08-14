from threading import Thread
import pygame, os, time, traceback
import libardrone

class ARCommanderGUI(Thread):
    draw_vid = None
    running = True
    drone = None
    spacebar_pressed = False
    allow_ros_control = None
    
    def __init__(self, drone, draw_vid=True):
        # init vars
        self.draw_vid = draw_vid
        self.drone = drone
        self.allow_ros_control = True
        # start process
        super(self.__class__,self).__init__()
        self.start()
        
    def stop(self):
        self.running = False
    
    @staticmethod
    def echo(s):
        print "[ARCommanderGUI] {0}".format(s)
    
    @staticmethod
    def warning(s):
        print "[[ARCommanderGUI]] Warning: {0}".format(s)
    
    def run(self):
        """ creates GUI window which enables user to command drone """
        # settings
        W, H = 320, 240
        ZOOM = 2
        # surfaces from pictures
        ardrone_pic_surface = pygame.image.load(os.path.dirname(__file__) + '/ardrone.jpg')
        no_ardrone_surface = pygame.image.load(os.path.dirname(__file__) + '/no_ardrone.jpg')
        ardrone_pic_surface = pygame.transform.scale(ardrone_pic_surface, (ZOOM*W, ZOOM*H))
        no_ardrone_surface = pygame.transform.scale(no_ardrone_surface, (ZOOM*W, ZOOM*H))
        # init
        pygame.init()
        screen = pygame.display.set_mode((ZOOM*W, ZOOM*H))
        pygame.display.set_caption("AR.Drone commander")
        clock = pygame.time.Clock()
        
        surface = no_ardrone_surface
        screen.blit(surface, (0, 0))
        
        while self.running:
            """ parse events """
            for event in pygame.event.get():
                self.parseEvent(event)
            
            """ print video/picture as background on screen """
            oldsurface = surface
            if self.drone.image:
                if self.draw_vid:
                    surface = pygame.image.fromstring(self.drone.image, (W, H), 'RGB')
                    surface = pygame.transform.scale(surface, (ZOOM*W, ZOOM*H))
                else:
                    surface = ardrone_pic_surface
            else:
                surface = no_ardrone_surface
            if not oldsurface is surface:
                screen.blit(surface, (0, 0))
            
            """ print info on screen """
            LH = ZOOM*11 # lineheight
            hud_color = (0, 0, 0)
            f = pygame.font.Font(pygame.font.match_font('verdana'), ZOOM*7)
            battery = self.drone.getFromNavdata('battery')
            info = [
                ['battery: {0}%'.format(battery)],
                ['vx', self.drone.getFromNavdata('vx')],
                ['vy', self.drone.getFromNavdata('vy')],
                ['altitude', self.drone.getFromNavdata('altitude')],
                ['theta', self.drone.getFromNavdata('theta')],
                ['phi', self.drone.getFromNavdata('phi')],
                ['psi', self.drone.getFromNavdata('psi')],
            ]
            # print frame for info
            surface2 = pygame.surface.Surface((ZOOM*100, ZOOM*3+LH*len(info)))
            surface2.set_alpha(100 if self.drone.image and self.draw_vid else 255)
            surface2.fill((150, 255, 150))
            screen.blit(surface2, (ZOOM*3, ZOOM*3))
            # print info
            for i, line in enumerate(info):
                line = ': '.join(str(x) for x in line)
                hud = f.render(line, True, hud_color)
                screen.blit(hud, (ZOOM*6, ZOOM*6 + i*LH))
            
            # refresh screen
            pygame.display.flip()
            # wait
            clock.tick(20) #Hz
        
        # thread end --> close display
        pygame.display.quit()
    
    def getSpacebarPressed(self):
        if self.spacebar_pressed:
            self.spacebar_pressed=False
            return True
        return False
    
    def parseEvent(self, event):
        if event.type == pygame.QUIT:
            self.echo("stop")
            self.stop()
        elif event.type == pygame.KEYUP:
            self.echo("hover")
            self.drone.hover()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                self.echo("stop")
                self.drone.reset()
                self.stop()
            # takeoff / land
            elif event.key == pygame.K_RETURN:
                self.echo("takeoff")
                self.drone.takeoff()
            elif event.key == pygame.K_BACKSPACE:
                self.echo("land")
                self.drone.land()
            # emergency
            elif event.key == pygame.K_r:
                self.echo("reset")
                self.drone.reset()
            # spacebar_pressed (for SLAM calibration)
            elif event.key == pygame.K_SPACE:
                self.echo("spacebar_pressed")
                self.spacebar_pressed = True
            # forward / backward
            elif event.key == pygame.K_z:
                self.echo("forward")
                self.drone.move_forward()
            elif event.key == pygame.K_s:
                self.echo("backward")
                self.drone.move_backward()
            # left / right
            elif event.key == pygame.K_q:
                self.echo("left")
                self.drone.move_left()
            elif event.key == pygame.K_d:
                self.echo("right")
                self.drone.move_right()
            # up / down
            elif event.key == pygame.K_UP:
                self.echo("up")
                self.drone.move_up()
            elif event.key == pygame.K_DOWN:
                self.echo("down")
                self.drone.move_down()
            # turn left / turn right
            elif event.key == pygame.K_LEFT:
                self.echo("turn left")
                self.drone.turn_left()
            elif event.key == pygame.K_RIGHT:
                self.echo("turn right")
                self.drone.turn_right()
            # speed
            elif event.key == pygame.K_1:
                self.drone.speed = 0.1
            elif event.key == pygame.K_2:
                self.drone.speed = 0.2
            elif event.key == pygame.K_3:
                self.drone.speed = 0.3
            elif event.key == pygame.K_4:
                self.drone.speed = 0.4
            elif event.key == pygame.K_5:
                self.drone.speed = 0.5
            elif event.key == pygame.K_6:
                self.drone.speed = 0.6
            elif event.key == pygame.K_7:
                self.drone.speed = 0.7
            elif event.key == pygame.K_8:
                self.drone.speed = 0.8
            elif event.key == pygame.K_9:
                self.drone.speed = 0.9
            elif event.key == pygame.K_0:
                self.drone.speed = 1.0
            elif event.key == pygame.K_DELETE:
                self.echo('disallow ros control')
                self.allow_ros_control = False
            elif event.key == pygame.K_INSERT:
                self.echo('reallow ros control')
                self.allow_ros_control = True


