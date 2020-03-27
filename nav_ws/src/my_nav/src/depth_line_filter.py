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

        self.image = None

        image_sub = message_filters.Subscriber("camera/repub/rgb/image_raw", numpy_msg(Image))
        depth_sub = message_filters.Subscriber("camera/repub/depth/image_raw", numpy_msg(Image))
        sync = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=5, slop=0.1)
        sync.registerCallback(self.callback)

        self.laserscan_pub = rospy.Publisher('/lines/scan', LaserScan, queue_size=5)

    def callback(self, rgb_data, depth_data):
        try:
            self.image = np.frombuffer(rgb_data.data, dtype=np.uint8).reshape(rgb_data.height, rgb_data.width, -1)
            self.depth_image = np.frombuffer(depth_data.data, dtype=np.uint8).reshape(depth_data.height, depth_data.width, -1).astype(np.float32)
            # print(np.unique(self.depth_image))
            # print(self.image.shape, self.depth_image.shape)
        except Exception, e:
            print e

    def run(self):
        while not rospy.is_shutdown():
            if cv2.waitKey(1) & 0xff == ord('q'):
                break
            print('.')
            if self.image is None:
                rospy.sleep(0.1)
                continue
            # print(self.depth_image)
            cv2.imshow("dbg", self.depth_image)
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
