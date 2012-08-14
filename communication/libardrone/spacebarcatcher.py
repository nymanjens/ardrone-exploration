from threading import Thread
import pygame, os, time, traceback
import rospy

class SpacebarCatcher(Thread):
    spacebar_pressed = False
    
    def __init__(self):
        super(self.__class__,self).__init__()
        self.start()
        
    def stop(self):
        if not rospy.is_shutdown():
            rospy.signal_shutdown("window closed")
    
    @staticmethod
    def echo(s):
        print "[SpacebarCatcher] {0}".format(s)
    
    @staticmethod
    def warning(s):
        print "[[SpacebarCatcher]] Warning: {0}".format(s)
    
    def run(self):
        """ creates GUI window which captures the spacebar """
        # settings
        W, H = 320, 240
        # surfaces from pictures
        ardrone_pic_surface = pygame.image.load(os.path.dirname(__file__) + '/ardrone.jpg')
        ardrone_pic_surface = pygame.transform.scale(ardrone_pic_surface, (W, H))
        # init
        pygame.init()
        screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Spacebar catcher")
        screen.blit(ardrone_pic_surface, (0, 0))
        pygame.display.flip()
        # main loop
        while not rospy.is_shutdown():
            """ parse events """
            for event in pygame.event.get():
                self.parseEvent(event)
            # wait 50ms
            time.sleep(.050)
        # thread end --> close display
        pygame.display.quit()
    
    def getSpacebarPressed(self):
        if self.spacebar_pressed:
            self.spacebar_pressed=False
            return True
        return False
    
    def parseEvent(self, event):
        if event.type == pygame.QUIT:
            self.stop()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                self.stop()
            # spacebar_pressed (for SLAM calibration)
            elif event.key == pygame.K_SPACE:
                self.echo("spacebar_pressed")
                self.spacebar_pressed = True



