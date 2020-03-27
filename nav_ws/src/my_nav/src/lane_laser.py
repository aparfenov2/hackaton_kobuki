import roslib
import sys
import rospy
import cv2
import cv2 as cv
import message_filters
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
import lane_detection_module as ld


def do_canny(frame):
    # Converts frame to grayscale because we only need the luminance channel for detecting edges - less computationally expensive
    gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    # Applies a 5x5 gaussian blur with deviation of 0 to frame - not mandatory since Canny will do this for us
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    # Applies Canny edge detector with minVal of 50 and maxVal of 150
    canny = cv.Canny(blur, 50, 150)
    return canny

def do_segment(frame):
    # Since an image is a multi-directional array containing the relative intensities of each pixel in the image, we can use frame.shape to return a tuple: [number of rows, number of columns, number of channels] of the dimensions of the frame
    # frame.shape[0] give us the number of rows of pixels the frame has. Since height begins from 0 at the top, the y-coordinate of the bottom of the frame is its height
    height = frame.shape[0]
    # Creates a triangular polygon for the mask defined by three (x, y) coordinates
    polygons = np.array([
                            [(0, height), (800, height), (380, 290)]
                        ])
    # Creates an image filled with zero intensities with the same dimensions as the frame
    mask = np.zeros_like(frame)
    # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
    cv.fillPoly(mask, polygons, 255)
    # A bitwise and operation between the mask and frame keeps only the triangular area of the frame
    segment = cv.bitwise_and(frame, mask)
    return segment

def calculate_lines(frame, lines):
    # Empty arrays to store the coordinates of the left and right lines
    left = []
    right = []
    # Loops through every detected line
    if lines is None or len(lines) < 1:
        return []

    for line in lines[0]:
        # Reshapes line from 2D array to 1D array
        x1, y1, x2, y2 = line.reshape(4)
        # Fits a linear polynomial to the x and y coordinates and returns a vector of coefficients which describe the slope and y-intercept
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_intercept = parameters[1]
        # If slope is negative, the line is to the left of the lane, and otherwise, the line is to the right of the lane
        if slope < 0:
            left.append((slope, y_intercept))
        else:
            right.append((slope, y_intercept))
    ret = []
    if len(left) > 0:
        # Averages out all the values for left and right into a single slope and y-intercept value for each line
        left_avg = np.average(left, axis = 0)
        # Calculates the x1, y1, x2, y2 coordinates for the left and right lines
        left_line = calculate_coordinates(frame, left_avg)
        ret.append(left_line)

    if len(right) > 0:
        right_avg = np.average(right, axis = 0)
        right_line = calculate_coordinates(frame, right_avg)
        ret.append(right_line)

    return np.array(ret)

def calculate_coordinates(frame, parameters):
    slope, intercept = parameters
    # Sets initial y-coordinate as height from top down (bottom of the frame)
    y1 = frame.shape[0]
    # Sets final y-coordinate as 150 above the bottom of the frame
    y2 = int(y1 - 150)
    # Sets initial x-coordinate as (y1 - b) / m since y1 = mx1 + b
    x1 = int((y1 - intercept) / slope)
    # Sets final x-coordinate as (y2 - b) / m since y2 = mx2 + b
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])

def visualize_lines(frame, lines):
    # Creates an image filled with zero intensities with the same dimensions as the frame
    lines_visualize = np.zeros_like(frame)
    # Checks if any lines are detected
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            print(x1, y1, x2, y2)
            # Draws lines between two coordinates with green color and 5 thickness
            cv.line(lines_visualize, (x1, y1), (x2, y2), (0, 255, 0), 5)
    return lines_visualize

class Filter:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.cvt_image) 
        # self.image_sub = rospy.Subscriber("/movie",Image,self.cvt_image) 
        self.laserscan_pub = rospy.Publisher('/lines/scan', LaserScan, queue_size=5)
        self.image_pub = rospy.Publisher("/lines/combined",Image, queue_size=10)
        self.latestImage = None

        self.kernel_size = 11
        self.low_threshold = 40
        self.high_threshold = 50
        self.rho = 1
        self.theta = np.pi/180
        self.threshold = 100
        self.min_line_len = 10 #60
        self.max_line_gap = 10 #80
        self.lines = (0, 0, 0, 0)
        
        # self.corners = np.float32([[44,560], [378,450],[902,450],[1215,560]]) #Gazebo Conde track

    def cvt_image(self,data):  
        try:
            self.latestImage = self.bridge.imgmsg_to_cv2(data, "bgr8")  
        except CvBridgeError as e:
            print(e)

    def publish(self, image,  bridge,  publisher):
        try:
            #Determine Encoding
            if np.size(image.shape) == 3: 
                imgmsg = bridge.cv2_to_imgmsg(image, "bgr8") 
            else:
                imgmsg = bridge.cv2_to_imgmsg(image, "mono8") 
            publisher.publish(imgmsg)  
        except CvBridgeError as e:
            print(e)

    def run(self):     
        while not rospy.is_shutdown():
            if self.latestImage is None:
                rospy.sleep(0.1)
                continue

            cv2.imshow("debug", self.latestImage)
            cv2.waitKey(1)
            continue

            # frame = self.latestImage
            # canny = do_canny(frame)

            # Only run loop if we have an image
            self.blurImage = ld.gaussian_blur(self.latestImage, self.kernel_size)             
            self.edgeImage = ld.canny(self.blurImage, self.low_threshold, self.high_threshold)
             
            #Define region of interest for cropping
            height = self.latestImage.shape[0]
            width = self.latestImage.shape[1]

            # combined, abs_bin, mag_bin, dir_bin, hls_bin = combined_thresh(self.latestImage)

            # segment = do_segment(canny)
            # segment = canny
            # hough = cv.HoughLinesP(segment, 2, np.pi / 180, 100, np.array([]), minLineLength = 100, maxLineGap = 50)
            # Averages multiple detected lines from hough into one line for left border of lane and one line for right border of lane
            # lines = calculate_lines(frame, hough)
            # Visualizes the lines
            # lines_visualize = visualize_lines(frame, lines)
            # cv.imshow("hough", lines_visualize)
            # Overlays lines on frame by taking their weighted sums and adding an arbitrary scalar value of 1 as the gamma argument
            # output = cv.addWeighted(frame, 0.9, lines_visualize, 1, 1)
            # Opens a new window and displays the output frame
            # cv.imshow("output", output)

            vertices = np.array( [[
                    [4*width/4, 3*height/5],
                    [0*width/4, 3*height/5],
                    [10, height],
                    [width-10, height]
                ]], dtype=np.int32 )
            self.maskedImage = ld.region_of_interest(self.edgeImage, vertices)

            # self.corners = np.float32([[0,height], [208,310],[270,310],[width, height]])
            # self.warpedImage,  _,  _ = ld.perspective_transform(self.latestImage, self.corners)

            lines = ld.hough_lines(self.maskedImage, self.rho, self.theta, self.threshold, self.min_line_len, self.max_line_gap)
            # # print(self.lines)
            # lines = hough
            for x1, y1, x2, y2 in lines[0]:
                cv2.line(self.latestImage, (x1,y1),(x2,y2),(0,0,255),2)
            # self.lineMarkedImage, self.intersectionPoint  = ld.draw_lane_lines(self.latestImage, self.lines)

            # # Publish Processed Image
            # self.outputImage = self.lineMarkedImage
            # self.publish(self.outputImage, self.bridge,  self.image_pub)

            # cv2.imshow("src", canny)
            cv2.imshow("debug", self.latestImage)
            # cv2.imshow("mixed", self.lineMarkedImage)
            cv2.waitKey(1)
            self.latestImage = None

def main(args):
    fp = Filter()
    rospy.init_node('lane_filter', anonymous=True)
    try:
        fp.run()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
