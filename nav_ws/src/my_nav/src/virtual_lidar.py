import roslib #; roslib.load_manifest('rfh_follow_me')
import sys
import rospy
import cv2
import message_filters
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
import tf
# https://answers.ros.org/question/215152/combining-the-depth-and-color-data/

def read_transparent_png(filename):
    image_4channel = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    alpha_channel = image_4channel[:,:,3]
    rgb_channels = image_4channel[:,:,:3]

    # White Background Image
    white_background_image = np.ones_like(rgb_channels, dtype=np.uint8) * 255

    # Alpha factor
    alpha_factor = alpha_channel[:,:,np.newaxis].astype(np.float32) / 255.0
    alpha_factor = np.concatenate((alpha_factor,alpha_factor,alpha_factor), axis=2)

    # Transparent Image Rendered on White Background
    base = rgb_channels.astype(np.float32) * alpha_factor
    white = white_background_image.astype(np.float32) * (1 - alpha_factor)
    final_image = base + white
    final_image = final_image.astype(np.uint8)
    final_image = cv2.cvtColor(final_image, cv2.COLOR_BGR2GRAY)
    final_image = 255 - final_image
    return final_image

class Filter:
    def __init__(self):
        self.tf_listener = tf.TransformListener()        
        self.laserscan_pub = rospy.Publisher('/lines/scan', LaserScan, queue_size=5)
        self.line_map = read_transparent_png('../map/virt_lines_cropped.png')
        # print(self.line_map.shape)
        # print(self.line_map[self.line_map > 0])
        # cv2.imshow("", self.line_map)
        # cv2.waitKey(0)
        # exit(0)
        self.initial_rot = (0,0,0)
        self.initial_trans = (0,0,0)

    def run(self):
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/base_footprint', '/lines_map', rospy.Time(0))
            except Exception as e:
                # print(e)
                rospy.sleep(0.1)
                continue

            print("tf", trans, rot)
            img = np.copy(self.line_map)
            cx, cy, _ = trans

            cx, cy = int(cx), int(cy)
            cv2.circle(img, (cx, cy), 10, (255), thickness=2)

            # ls = get_scan(img, trans, rot)

            cv2.imshow("dbg", img)
            cv2.waitKey(1)

    # function for line generation  
    def trace_line(self, img, x1,y1, theta):
        r = 0
        max_r = 100
        while r < max_r:
            r += 0.1
            x = x1 + r*np.cos(theta)
            y = y1 + r*np.sin(theta)
            if y < 0 or y >= img.shape[0]:
                return None
            if x < 0 or x >= img.shape[1]:
                return None
            if img[y,x] > 0:
                return x, y
            return None
          
    def get_scan(self, img, trans, rot):
        thetas = np.linspace(0, np.pi, 1000)
        x1, y1 = trans
        ranges = np.full(thetas.shape, np.inf)

        for i, theta in enumerate(thetas):
            ret = self.trace_line(img, x1, y1, theta)
            if ret is not None:
                x2, y2 = ret
                ranges[i] = np.linalg.norm(np.array([x1-x2,y1-y2]))
                
        ls = LaserScan()
        return ls

def main(args):
    rospy.init_node('lane_filter', anonymous=True)
    fp = Filter()
    try:
        fp.run()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
