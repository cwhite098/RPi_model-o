import os
import io
import cv2
import RPi.GPIO as gp
import time
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread


class TacTip:
    def __init__(self, width, height, fps, name, thresh_width, thresh_offset, crop, display=True):
        # Init class vars
        self.width = width
        self.height = height
        self.fps = fps
        self.name = name
        self.crop = crop
        self.display = display
    
        # params for Gaussian thresholding
        self.thresh_width = thresh_width
        self.thresh_offset = thresh_offset
        
        # init camera
        self.camera = PiCamera()
        self.camera.resolution = (self.width, self.height)
        
        # Set up raw capture - has to be this res
        self.raw_capture = PiRGBArray(self.camera, size=(self.width, self.height))
        self.vid_stream = self.camera.capture_continuous(self.raw_capture, format='bgr', use_video_port=True)
        
        self.frame = None
        self.stopped = False
        
        # Let camera warm up
        time.sleep(0.5)
        
        
    def stream(self):
        
        # Capture frames from camera and loop over them
        for frame in self.vid_stream:
            # grab the np array of the image
            self.frame = frame.array
            
            # Clear the frame buffer in camera
            self.raw_capture.truncate()
            self.raw_capture.seek(0)
            
            if self.stopped:
                self.camera.close()
                self.raw_capture.close()
                self.vid_stream.close()
                break
                return
            
            
    def process(self):
        
        while not self.stopped:
            image = self.read()
            image = self.process_frame(image)
            
            # show the frame
            if self.display:
                cv2.imshow(self.name, image)
            key = cv2.waitKey(1)
            
            print(1)
                        
            # Break on press of q
            if key == ord('q'):
                self.stopped=True

        
    
    def start_cap(self):
        Thread(target=self.stream,args=()).start()
    
    def read(self):
        return self.frame
    
    def stop(self):
        self.stopped = True
    
    
    def process_frame(self, frame):
        
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)# convert to grayscale
        # gaussian thresholding
        frame = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, self.thresh_width, self.thresh_offset)
        # Crop the frame to only show the pins in tactip
        x0,y0,x1,y1 = self.crop
        frame = frame[y0:y1,x0:x1]
        
        return frame
        
        
        
def main():
    tactip = TacTip(640,480,32,'Tactip1', 121, -8, [10,80,590,420], True)
    print('Starting Camera Stream...')
    tactip.start_cap()
    time.sleep(0.5)
    print('Starting Frame Processing...')
    tactip.process()

    return 0



if __name__ == '__main__':
    main()




