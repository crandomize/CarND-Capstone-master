from styx_msgs.msg import TrafficLight

import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass


    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #return self.detect_color(image, TrafficLight.RED)

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        image = cv2.GaussianBlur(image, (9,9), 0)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        #TODO implement light color prediction
        min_red = np.array([0, 50, 50])
        max_red = np.array([10, 255, 255])
        mask_red = cv2.inRange(image, min_red, max_red)
        
        min_yellow = np.array([20, 128, 128])
        max_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(image, min_yellow, max_yellow)

        mask = mask_red + mask_yellow
       
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html

        mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask_open = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel)
        _, contours, _ = cv2.findContours(mask_open, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours)>0:
            return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN


        

