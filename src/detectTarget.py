import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np

class detectTarget:

    def __init__(self):

        # convert ROS Image to OpenCV Image
        self.cvBridge = CvBridge()

        self.pub_ImageLocation = rospy.Publisher('imageLocation', Point, queue_size=10)
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.image_callback)
        rospy.init_node('detect_target', anonymous=True)

    """ This function is a callback function for each frame of the captured image from camera. """
    def image_callback(self, CompressedImage):

        global updateTargetPosition, firstImageDetected

        # Transforming CompressedImage to a color image in BGR space
        self.colorBGRImage = self.cvBridge.compressed_imgmsg_to_cv2(CompressedImage, "bgr8")

        # height and width of the image to pass along to the PID controller as the reference point.
        height, width = self.colorBGRImage.shape[:2]

        # Image used to show object boundary
        self.trackerImage = self.colorBGRImage.copy()
        
        # Blur the image to reduce edges caused by noise or that are useless to us.
        blurredImage = cv2.GaussianBlur(self.colorBGRImage, (blurKernelSize, blurKernelSize), 0)

        # Transform BGR to HSV to avoid lighting issues.
        self.hsvImage = cv2.cvtColor(blurredImage, cv2.COLOR_BGR2HSV)	
        
        # Threshold the image using the selected lower and upper bounds of the color of the object.
        self.maskedImage = cv2.inRange(self.hsvImage, lowerHSVThreshold, upperHSVThreshold)

        # To get rid of noise and fill in gaps in our object use open and close.
        morphedImage = self.performMorphedOperations(self.maskedImage, morphOpKernelSize)

        centersOfTarget = self.findTargets(morphedImage)

        # Not always, the houghCircles function finds circle, so a None inspection is made
        if not centersOfTarget: 
            #If no object was found, sends bogus numbers.
            self.targetCoordinate = Point()

            self.targetCoordinate.x = 999
            self.targetCoordinate.y = 999
            self.targetCoordinate.z = 999
            updateTargetPosition = True

        elif centersOfTarget is not []:
            for cot in centersOfTarget:
                # The x position of the center of the object, the width of the object, and the width of the image.
                p = Point(cot[0], cot[1], width)

                self.targetCoordinate = Point()
                self.targetCoordinate.x = p.x
                self.targetCoordinate.y = p.y
                self.targetCoordinate.z = p.z
                updateTargetPosition = True
            
        firstImageDetected = True

    def performMorphedOperations(self, maskedImage, kernelSize):
        # Morphological operations (open and close) used to reduce noise in the acquired image.
        kernel = np.ones((kernelSize,kernelSize), np.uint8)
        # Fills in the holes
        morphedImage = cv2.morphologyEx(maskedImage, cv2.MORPH_CLOSE, kernel)
        # Get rid of noise
        morphedImage = cv2.morphologyEx(morphedImage, cv2.MORPH_OPEN, kernel)
        
        return morphedImage

    def findTargets(self, binaryMatrix):
        #Finds the location of the desired object in the image.
        output = []

        # Contours the image to find blobs of the same color
        contours, _ = cv2.findContours(binaryMatrix, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Sorts the blobs by size (Largest to smallest) 
        maxContour = sorted(contours, key = cv2.contourArea, reverse = True)[:maxNumberOfObjects]

        # Find the center of mass of the blob if there are any
        if len(maxContour) > 0:
            for i in range (0, len(maxContour)):
                M = cv2.moments(maxContour[i])
                # Check if the total area of the contour is large enough to care about!
                if M['m00'] > minObjectArea:
                    rect = cv2.minAreaRect(maxContour[0])
                    w = int(rect[1][0])
                    x = int(M['m10']/M['m00'])
                    y = int(M['m01']/M['m00'])
                    if(showImages):
                        # Draws the contour.
                        cv2.drawContours(self.trackerImage, maxContour[i], -1, (255, 0, 0), 3)
                        self.drawOverRegion(self.trackerImage, x, y, objectName)
                    if output == []:
                        output = [[x,w]]
                    else:
                        output.append[[x,w]]
        return output

    def drawOverRegion(self, frame, x, y, objectName):
        cv2.circle(frame, (x,y), 5, (0,255,0))
        cv2.putText(frame, objectName, (x - 30, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)


objectName = "Target"
# lower threshold bound of the HSV image
lowerHSVThreshold = np.array([60, 100, 50], np.uint8) 
# upper threshold bound of the HSV image
upperHSVThreshold = np.array([60, 255, 255], np.uint8)
# error widths to create the upper and lower threshold bounds above.
diffHSVThreshold = np.array([15, 100, 100], np.uint8)

trackerImageWindow = "Object Tracker"
originalImageWindow = "Original Image"
maskedImageWindow = "Masked Image"
# Whether to show images
showImages = True

# Width of the image, this is sent in our point message as the z-component to know the zero point in the frame.
width = 360
# Blur Kernel Size
blurKernelSize = 9
# Closing and Opening Kernel Size
morphOpKernelSize = 5   

# Max number of object to detect.
maxNumberOfObjects = 1
# Min number of pixels for an object to be recognized. 
minObjectArea = 150

# Set to true when first image is acquired and will firstImageDetected the program.
firstImageDetected = False

# True - When a new point has been found and can be published. False - Otherwise.
updateTargetPosition = False

def mouseCallbackEvent(event, x, y, flags, param):
    # Whenever left mouse button is pressed, this callback method is invoked.
    # This event is tracked on the "Original Image" window.
    # Using the HSV value of the clicked pixel, lower and upper thresholds are computed.

    global lowerHSVThreshold, upperHSVThreshold, diffHSVThreshold
    
    if event == cv2.EVENT_LBUTTONDOWN:
        lowerHSVThreshold = cv2.subtract(detectTarget.hsvImage[y,x,:], diffHSVThreshold)
        upperHSVThreshold = cv2.add(detectTarget.hsvImage[y,x,:], diffHSVThreshold)


if __name__ == '__main__':
    try:
        detectTarget = detectTarget()
    except rospy.ROSInterruptException:
        pass

rate = rospy.Rate(10)

if showImages:
    cv2.namedWindow(trackerImageWindow, cv2.WINDOW_AUTOSIZE )
    cv2.moveWindow(trackerImageWindow, 620, 50)

    cv2.namedWindow(maskedImageWindow, cv2.WINDOW_AUTOSIZE )
    cv2.moveWindow(maskedImageWindow, 1240, 50)

    cv2.namedWindow(originalImageWindow, cv2.WINDOW_AUTOSIZE )
    cv2.moveWindow(originalImageWindow, 50, 50)
    cv2.setMouseCallback(originalImageWindow, mouseCallbackEvent)

while not rospy.is_shutdown():

    if firstImageDetected:

        if showImages:
            cv2.imshow(originalImageWindow, detectTarget.colorBGRImage)
            cv2.imshow(maskedImageWindow, detectTarget.maskedImage)
            cv2.imshow(trackerImageWindow, detectTarget.trackerImage)

        if updateTargetPosition:
            detectTarget.pub_ImageLocation.publish(detectTarget.targetCoordinate)
            updateTargetPosition = False

        rate.sleep()

        # User's options to interact with the software
        key = cv2.waitKey(10)

        # Decreasing the Hue value of the threshold difference
        # When '1' number key is pressed
        if key == 49:
            diffHSVThreshold[0] = max(diffHSVThreshold[0] - 1, 0)
            key = 0
            rospy.loginfo("Hue Difference: %d", diffHSVThreshold[0])

        # Increasing the Hue value of the threshold difference
        # When '2' number key is pressed
        elif key == 50:
            diffHSVThreshold[0] = min(diffHSVThreshold[0] + 1, 50)
            key = 0
            rospy.loginfo("Hue Difference: %d", diffHSVThreshold[0])

        # Decreasing the morphOp kernel size
        # When '3' number key is pressed
        elif key == 51:
            morphOpKernelSize = max(morphOpKernelSize - 2, 1)
            key = 0
            rospy.loginfo("Kernel size for close and open: %d", morphOpKernelSize)

        # Increasing the morphOp kernel size
        # When '4' number key is pressed
        elif key == 52: #number 4
            morphOpKernelSize = morphOpKernelSize + 2
            key = 0
            rospy.loginfo("Kernel size for close and open: %d", morphOpKernelSize)

        # Decreasing the blur kernel size
        # When '5' number key is pressed
        elif key == 53:
            morphOpKernelSize = max(blurKernelSize - 2, 1)
            key = 0
            rospy.loginfo("Blur kernel size: %d", blurKernelSize)

        # Increasing the blur kernel size
        # When '6' number key is pressed
        elif key == 54:
            blurKernelSize = blurKernelSize + 2
            key = 0
            rospy.loginfo("Blur kernel size: %d", blurKernelSize)

        # Decreasing the min object area to detect
        # When '7' number key is pressed
        elif key == 55:
            minObjectArea = max(minObjectArea - 1, 1)
            key = 0
            rospy.loginfo("Min object pixel area: %d", minObjectArea)

        # Increasing the min object area to detect
        # When '8' number key is pressed
        elif key == 56:
            minObjectArea = minObjectArea + 1
            key = 0
            rospy.loginfo("Min object pixel area: %d", minObjectArea)
