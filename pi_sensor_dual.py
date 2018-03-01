import sys,os 
import time
import picamera
import io
import numpy as np
import cv2
from threading import Thread
from lake_of_sirannon import WatcherInTheWater as Watcher

import RPi.GPIO as gp

class TactileSensor():
    def __init__(self,
                 frame_width=320,
                 frame_height=240,
                 frame_rate = 60,                 
                 minThreshold = [80,90],
                 maxThreshold = [263,265],
                 maxArea = [180,200],
                 minArea = [33,21],
                 minCircularity = [0.37,0.45],
                 minConvexity = [0.37,0.51],
                 minInertiaRatio = [0.37,0.41],
                 camera_disp=False):

        ###################################

        self._frame_width = frame_width
        self._frame_height = frame_height
        self._frame_rate = frame_rate

        ###################################

        # Set blob detection parameters
        _params_dist = cv2.SimpleBlobDetector_Params()
        # Change thresholds
        _params_dist.minThreshold = minThreshold[0]
        _params_dist.maxThreshold = maxThreshold[0]
        # Filter by Area.
        _params_dist.filterByArea = True
        _params_dist.maxArea = maxArea[0]
        _params_dist.minArea = minArea[0]
        # Filter by Circularity
        _params_dist.filterByCircularity = True
        _params_dist.minCircularity = minCircularity[0]
        # Filter by Convexity
        _params_dist.filterByConvexity = True
        _params_dist.minConvexity = minConvexity[0]
        # Filter by Inertia
        _params_dist.filterByInertia = True
        _params_dist.minInertiaRatio = minInertiaRatio[0]

        _params_prox = cv2.SimpleBlobDetector_Params()
        # Change thresholds
        _params_prox.minThreshold = minThreshold[1]
        _params_prox.maxThreshold = maxThreshold[1]
        # Filter by Area.
        _params_prox.filterByArea = True
        _params_prox.maxArea = maxArea[1]
        _params_prox.minArea = minArea[1]
        # Filter by Circularity
        _params_prox.filterByCircularity = True
        _params_prox.minCircularity = minCircularity[1]
        # Filter by Convexity
        _params_prox.filterByConvexity = True
        _params_prox.minConvexity = minConvexity[1]
        # Filter by Inertia
        _params_prox.filterByInertia = True
        _params_prox.minInertiaRatio = minInertiaRatio[1]

        self.detectors = [cv2.SimpleBlobDetector_create(_params_dist),
            cv2.SimpleBlobDetector_create(_params_prox)]

        ####################################

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

        self.camera_disp = camera_disp
        if self.camera_disp == True:
            cv2.namedWindow('sensor1', cv2.WINDOW_NORMAL)            
            cv2.namedWindow('sensor2', cv2.WINDOW_NORMAL)

    def start_cam(self): 
        self.camera = picamera.PiCamera()
        self.is_opened = True
        
        self.camera.resolution = (self._frame_width, self._frame_height)
        self.camera.shutter_speed = 200000
        self.camera.framerate = self._frame_rate
        self.camera.exposure_mode='auto'
        self.camera.awb_mode='auto'
        # self.camera.awb_gains=1 # between 0 and 8 typical 0.9-1.9

        time.sleep(2)    # Camera Initialize
        
        self.start_thread()

    def start_thread(self):
        t = Thread(target=self.capt, args=())
        t.daemon = True
        t.start()
        print 't started'

    def capt(self):
        self.camera.capture_sequence(self.output(), use_video_port=True)
        self.close()

        # self.iv.camera_sequence(outputs = self.output(), use_video_port=True)
        # self.iv.close()

    def cam_change(self, **kwargs):
        if 'camera_ID' in kwargs:
            self.cam = kwargs.get('camera_ID')  

        gp.setmode(gp.BOARD)
        if self.cam == 1:
            # CAM 1 for A Jumper Setting
            gp.output(self.ePin, False)
            gp.output(self.f1Pin, False)
            gp.output(self.f2Pin, True)

        elif self.cam == 2:
            # CAM 2 for A Jumper Setting
            gp.output(self.ePin, False)
            gp.output(self.f1Pin, True)
            gp.output(self.f2Pin, False)

        self.cam += 1
        if self.cam > 2:
            self.cam = 1

    def output(self):
        frame = 0
        stream = io.BytesIO()
        self.cam_change(camera_ID = 2)
        self.camera.start_preview()
        time.sleep(3)
        self.cam_change(camera_ID = 1)
        time.sleep(3)
        self.camera.stop_preview()
        while self.end == False:
            yield stream
            stream.seek(0)
            # convert image into numpy array
            data = np.fromstring(stream.getvalue(), dtype=np.uint8)
            # turn the array into a cv2 image
            img_jpg = cv2.imdecode(data, 1)
            img = cv2.cvtColor(img_jpg, cv2.COLOR_BGR2GRAY)
            stream.seek(0)
            stream.truncate()

            if self.cam == 1:
                self.dist = img
            if self.cam == 2:
                self.prox = img

            frame += 1

            self.cam = (frame%2)+1
            time.sleep(0.022)   # SD Card Bandwidth Correction Delay (0.007)
            self.cam_change()        # Switching Camera
            # self.iv.camera_change(self.cam)
            time.sleep(0.022)   # SD Card Bandwidth Correction Delay

    def start_calibrate_detect_pins(self,n_frames):

        # from tactile_sensor_optimizer import start_opt
        print 'Sampling'

        frames_1 = []
        frames_2 = []

        for frame in range(n_frames*2):
            time.sleep(0.05)
            self.count += 1
            camera_ID = (self.count%2+1)
            if camera_ID == 1:
                frames_1.append(self.dist)
            elif camera_ID == 2:
                frames_2.append(self.prox)

            sys.stdout.write('\r')
            progress_bar = int(np.ceil((float(frame)/float(n_frames*2)*20.0)))
            progress_percent = (float(frame)/float(n_frames*2))*100
            sys.stdout.write("[%-20s] %d%%" % ('='*progress_bar, progress_percent))
            sys.stdout.flush()

        sys.stdout.write('\n')

        print 'saving'

        frames_1 = np.array(frames_1)
        np.save('sample_frames_1D', frames_1)
        frames_2 = np.array(frames_2)
        np.save('sample_frames_1P', frames_2)

    def start_detect_pins(self):

        while self.end == False:

            self.count += 1
            camera_ID = (self.count%2+1)
            pins = self.detect_pins(camera_ID)

    def detect_pins(self,camera_ID):      

            if camera_ID == 1:
                frame = self.dist
            elif camera_ID == 2:
                frame = self.prox      

            frame_pins = frame
            keypoints = self.detectors[camera_ID-1].detect(cv2.bitwise_not(frame))
            im_with_keypoints = cv2.drawKeypoints(frame_pins, keypoints, np.array([]), (0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            pins = [a.pt[:] for a in keypoints]
            pins = np.asarray(pins)

            if self.camera_disp == True:
                if camera_ID == 1:
                    cv2.imshow('sensor1',im_with_keypoints)
                    key = cv2.waitKey(1) & 0xFF
                elif camera_ID == 2:
                    cv2.imshow('sensor2',im_with_keypoints)
                    key = cv2.waitKey(1) & 0xFF   

            return pins            

    def close(self):
        self.camera_change()
        if self.is_opened:
            self.camera.close()


    def stop_all(self):
        self.end = True
        # self.iv.close()
        self.close()
        print 'ending all processes...'

if __name__ == "__main__":

    w = Watcher()    
    TS = TactileSensor()
    TS.start_cam()
    time.sleep(7)
    #TS.start_detect_pins()
    TS.start_calibrate_detect_pins(200)
    time.sleep(10)


