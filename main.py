import os
import ivport_v2.ivport as ivport
import picamera
import cv2
import RPi.GPIO as gp

# Defining vars for the 2 cameras
camera1 = 1
camera2 = 2


class TacTip():
    def __init__(self, width, height, fps, display=False):
        # Initialise the class with params for the cameras
        self.width = width
        self.height = height
        self.fps = fps
        self.display = display

        # Do some setup for the RPi
        gp.setwarnings(False)
        gp.setmode(gp.BOARD)

        JUMPER_SETTINGS = 'A'

        




def main():
    iv = ivport.IVPort(ivport.TYPE_DUAL2)

    return 0





if __name__ == '__main__':
    main()




