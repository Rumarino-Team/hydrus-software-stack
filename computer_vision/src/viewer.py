

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class WebcamViewer:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('webcam_viewer', anonymous=True)

        # Subscribe to the camera image topic
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        
        # OpenCV window to show the images
        cv2.namedWindow("Webcam Feed", cv2.WINDOW_NORMAL)
        rospy.loginfo("Waiting for image data...")

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Display the image
            cv2.imshow("Webcam Feed", frame)

            # Wait for 1 ms and check if the user wants to quit by pressing 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User exit")
                cv2.destroyAllWindows()
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: {0}".format(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        viewer = WebcamViewer()
        viewer.run()
    except rospy.ROSInterruptException:
        pass