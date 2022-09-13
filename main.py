import os
import io
import cv2
import RPi.GPIO as gp
import time
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera


class TacTip:
    def __init__(self, width, height, fps, name):
        # Init class vars
        self.camera = PiCamera()
        self.width = width
        self.height = height
        self.fps = fps
        self.name = name
        
        # Set up raw capture
        self.raw_capture = PiRGBArray(self.camera, size=(self.width, self.height))
        
        # Let camera warm up
        time.sleep(0.5)
        
        
    def stream(self):
        
        # Capture frames from camera and loop over them
        for frame in self.camera.capture_continuous(self.raw_capture, format='bgr', use_video_port=True):
            # grab the np array of the image
            image = frame.array
            
            # Resize to be smaller and more managebale
            image = cv2.resize(image, (480, 360))
            
            # clear the stream before the next capture
            self.raw_capture.truncate()
            self.raw_capture.seek(0)
            
            # show the frame
            cv2.imshow(self.name, image)
            key = cv2.waitKey(1)
            

            
            # Break on press of q
            if key == ord('q'):
                break
        '''
        self.camera.capture(self.raw_capture, format='bgr')
        image = self.raw_capture.array
        cv2.imshow('test', image)
        cv2.waitKey(0)
        '''
        
        
        
def main():
    tactip = TacTip(1920,1088,32,'Tactip1')
    tactip.stream()

    return 0



if __name__ == '__main__':
    main()




