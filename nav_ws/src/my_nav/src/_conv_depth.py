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
from rospy.numpy_msg import numpy_msg

# https://answers.ros.org/question/215152/combining-the-depth-and-color-data/

class Filter:
    def __init__(self):

        # self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image)
        # self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.callback2, queue_size=1)
        self.bridge = CvBridge()
        self.image = None

        image_sub = message_filters.Subscriber("camera/repub/rgb/image_raw", numpy_msg(Image))
        depth_sub = message_filters.Subscriber("camera/repub/depth/image_raw", Image)
        sync = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=5, slop=0.1)
        sync.registerCallback(self.callback)

        self.laserscan_pub = rospy.Publisher('/lines/scan', LaserScan, queue_size=5)

    def callback(self, rgb_data, depth_data):
        try:
            self.image = np.frombuffer(rgb_data.data, dtype=np.uint8).reshape(rgb_data.height, rgb_data.width, -1)
            # self.depth_image = np.frombuffer(depth_data.data, dtype='f8').reshape(depth_data.height, depth_data.width, -1)
            self.depth_image = self.conv_depth(depth_data)
        except Exception, e:
            print e

    def conv_depth(self,msg_depth): # TODO still too noisy!
        try:
            # The depth image is a single-channel float32 image
            # the values is the distance in mm in z axis
            cv_image = self.bridge.imgmsg_to_cv2(msg_depth, "32FC1")
            # Convert the depth image to a Numpy array since most cv2 functions
            # require Numpy arrays.
            cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
            # Normalize the depth image to fall between 0 (black) and 1 (white)
            # http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html lines 95-125
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
            # Resize to the desired size
            # cv_image_resized = cv2.resize(cv_image_norm, self.desired_shape, interpolation = cv2.INTER_CUBIC)
            return cv_image_norm
        except CvBridgeError as e:
            print(e)
            return None

    def run(self):
        while not rospy.is_shutdown():
            if self.image is None:
                rospy.sleep(0.1)
                continue
            # print(self.depth_image)
            cv2.imshow("dbg", self.depth_image)
            if cv2.waitKey(1) & 0xff == ord('q'):
                break
            self.image = None

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
