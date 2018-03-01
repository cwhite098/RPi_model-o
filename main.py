import os
import io
import ivport_v2.ivport as ivport
import cv2
import RPi.GPIO as gp
from functools import reduce
import time
from threading import Thread
import numpy as np

# Defining vars for the 2 cameras
camera1 = 1
camera2 = 2


class TacTip():
    def __init__(self, width, height, fps, iv, display=False):

        # Initialise the class with params for the cameras
        self.width = width
        self.height = height
        self.fps = fps
        self.display = display
        
        self.iv = iv

        # Do some setup for the RPi
        gp.setwarnings(False)
        gp.setmode(gp.BOARD)

        JUMPER_SETTING = 'A'

        # Jumper Pin assignment
        IVJP = {'A': (11, 12), 'C': (21, 22), 'B': (15, 16), 'D': (23, 24)}
        pins = list(reduce(lambda x,y: x+y, IVJP.values()))
        pins.sort()
        DIVJP = {i+1 : x for i,x in enumerate(pins)}
        self.f1Pin, self.f2Pin = IVJP[JUMPER_SETTING]
        self.ePin = 7

        gp.setup(self.ePin, gp.OUT)
        gp.setup(self.f1Pin, gp.OUT)
        gp.setup(self.f2Pin, gp.OUT)

        gp.output(self.ePin, False)
        gp.output(self.f1Pin, False)
        gp.output(self.f2Pin, True)

        ###################################

        self.cam = 1
        self.end = False
        self.count = 1
        # self.img = np.empty((self._frame_height, self._frame_width, 3), dtype=np.uint8)

        if self.display == True:
            cv2.namedWindow('sensor1', cv2.WINDOW_NORMAL)            
            #cv2.namedWindow('sensor2', cv2.WINDOW_NORMAL)


    def start_cam(self): 
        self.iv.camera_open(camera_v2=True, resolution=(self.width,self.height), framerate=self.fps)
        self.iv.camera_change(1) # choose camera 1
        time.sleep(2)    # Camera Initialize
        
        self.get_frame()

    def start_thread(self):
        # Starts thread running the get_frame function
        t = Thread(target=self.get_frame, args=())
        t.daemon = True
        print('TacTip thread starting...')
        t.start()
        

    def get_frame(self):
        
        print('getting frame')
        while not self.end:
            print(self.count)
            cap = self.iv.camera_capture('test')
            img = cv2.cvtColor(cap, cv2.COLOR_RGB2GRAY)

            cv2.imshow('sensor1',img)
            self.count +=1

            if self.count>600:
                self.end=True



def main():
    iv = ivport.IVPort(ivport.TYPE_DUAL2)
    sensor1 = TacTip(320,240,60, iv, True)
    sensor1.start_cam()

    return 0





if __name__ == '__main__':
    main()




