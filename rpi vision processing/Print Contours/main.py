# Import the camera server
from cscore import CameraServer

# Import OpenCV and NumPy
import cv2
import numpy as np

# Import networktables
from networktables import NetworkTablesInstance

ntinst = NetworkTablesInstance.getDefault()
ntinst.startDSClient()
lowh = 0
lows = 0
lowv = 0

highh = 15
highs = 255
highv = 255

def drawRectangle(image,rect,color):
    cv2.rectangle(image,(int(rect[0]),int(rect[1])),(int(rect[0]+rect[2]),int(rect[1]+rect[3])),color,2)
def main():
    print(dir(ntinst))
    w = 640
    h = 360

    cs = CameraServer.getInstance()
    cs.startAutomaticCapture()
    cs.enableLogging()

    # Capture from the first USB Camera on the system
    camera = cs.startAutomaticCapture()
    camera.setResolution(w,h)

    # Get a CvSink. This will capture images from the camera
    cvSink = cs.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = cs.putVideo("Name", w,h)
    thresholdStream = cs.putVideo("Thresholded image", w,h)

    # Allocating new images is very expensive, always try to preallocate
    image = np.zeros(shape=(w, h, 3), dtype=np.uint8)

    while True:
        #print("Running")
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, image = cvSink.grabFrame(image)
        #blue
        #img_threshold = cv2.inRange(img_hsv, (161, 56, 203), (172, 156, 255))
        #_, contours, _ = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #contours = sorted(contours, key=cv2.contourArea, reverse=True)
        #for contour in contours[:5]:
        #    drawRectangle(image, cv2.boundingRect(contour),(255,255,255))

        outputStream.putFrame(image)
        #thresholdStream.putFrame(img_threshold)
main()