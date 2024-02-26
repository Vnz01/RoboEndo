#!/usr/bin/python3

#import struct
#
#file = open( "/dev/input/mice", "rb" );
#
#def getMouseEvent():
#  buf = file.read(3);
#  button = ord( \buf[0] );
#  bLeft = button & 0x1;
#  bMiddle = ( button & 0x4 ) > 0;
#  bRight = ( button & 0x2 ) > 0;
#  x,y = struct.unpack( "bb", buf[1:] );
#  print ("L:%d, M: %d, R: %d, x: %d, y: %d\n" % (bLeft,bMiddle,bRight, x, y) );
#  # return stuffs
#
#while( 1 ):
#  getMouseEvent();
#file.close();



from pynput import mouse
import time
import numpy as np


class MouseClass():

    def __init__(self):
        self.offset_x = 1980
        self.offset_y = 1080
        # Create a mover object that keep mouse centered on screen
        # so that it never reaches a wall
        self.mover = mouse.Controller()

        # main
        self.mover.position = (self.offset_x,self.offset_y) #center the mouse

        #create mouse callback
        self.listener = mouse.Listener(on_move=self.on_move,on_click=self.on_click,on_scroll=self.on_scroll)

        self.listener.start()
        print("initialize mouse tracker")


    def on_move(self,x, y):
        Dx = x-self.offset_x
        Dy = y-self.offset_y
        Dist = np.sqrt(Dx**2 + Dy**2)
        if Dist > 1000: #move beyond a unit circle of 1000 pixels
            x = Dx/Dist*990 + self.offset_x
            y = Dy/Dist*990 + self.offset_y
            self.mover.position = (x,y)


        print('Pointer moved to {0}'.format((x-self.offset_x, y-self.offset_y)))    

    def on_click(self,x, y, button, pressed):
        print('{0} at {1}'.format('Pressed' if pressed else 'Released',(x, y)))
        if not pressed:
            # Stop listener
            print('UnPressed')
            return True #change to false to stop listening

    def on_scroll(self, x, y, dx, dy):
        print('Scrolled {0} at {1}'.format('down' if dy < 0 else 'up', (x, y)))




mouse_object = MouseClass()
while 1:
    time.sleep(1)
    