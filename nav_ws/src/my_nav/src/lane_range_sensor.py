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



class Filter:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.cvt_image) 
        self.latestImage = None

        self.kernel_size = 3
        self.low_threshold = 20
        self.high_threshold = 70
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

    def run(self):     
        while not rospy.is_shutdown():
            if self.latestImage is None:
                rospy.sleep(0.1)
                continue

            # cv2.imshow("debug", self.latestImage)
            # cv2.waitKey(1)
            # continue

            # frame = self.latestImage
            # canny = do_canny(frame)

            # Only run loop if we have an image
            blurImage = ld.gaussian_blur(self.latestImage, self.kernel_size)             
            # self.edgeImage = ld.canny(self.blurImage, self.low_threshold, self.high_threshold)
             
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

            maskedImage = ld.region_of_interest(blurImage, vertices)


            # im = self.latestImage
            # bg = im[:,:,0] == im[:,:,1] # B == G
            # gr = im[:,:,1] == im[:,:,2] # G == R
            # more_128 = im[:,:,1] > 128
            # slices = np.bitwise_and(bg, gr)
            # slices = np.bitwise_and(slices, more_128)
            # slices = slices.astype(np.uint8) * 255

            boundaries = [([0, 0, 168], [172, 111, 255])] #Gazebo Conde track
            binaryImage = ld.binary_thresh(maskedImage,  boundaries,  'HSV')     #RGB or HSV
            edgeImage = ld.canny(binaryImage, self.low_threshold, self.high_threshold)
            # cv2.imshow("debug", edgeImage)
            # cv2.waitKey(1)
            # continue

            # self.corners = np.float32([[0,height], [208,310],[270,310],[width, height]])
            # self.warpedImage,  _,  _ = ld.perspective_transform(self.latestImage, self.corners)

            lines = ld.hough_lines(edgeImage, self.rho, self.theta, self.threshold, self.min_line_len, self.max_line_gap)
            # # print(self.lines)
            # lines = hough
            for x1, y1, x2, y2 in lines[0]:
                cv2.line(self.latestImage, (x1,y1),(x2,y2),(0,0,255),2)
            # self.lineMarkedImage, self.intersectionPoint  = ld.draw_lane_lines(self.latestImage, lines)

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
