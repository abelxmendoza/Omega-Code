import unittest
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32

class TestFullSystemIntegration(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_full_system_integration', anonymous=True)
        self.camera_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
        self.ultrasonic_pub = rospy.Publisher('/ultrasonic', Range, queue_size=10)
        self.line_tracking_pub = rospy.Publisher('/line_tracking/data', Float32, queue_size=10)
        self.path_pub = rospy.Publisher('/d_star_lite_path', PoseStamped, queue_size=10)
        rospy.Subscriber('/fused_sensor_data', Image, self.fused_data_callback)
        self.fused_data_received = False

    def fused_data_callback(self, data):
        self.fused_data_received = True

    def test_full_system_integration(self):
        # Publish test data
        self.camera_pub.publish(Image())
        self.ultrasonic_pub.publish(Range())
        self.line_tracking_pub.publish(Float32(data=0.5))
        self.path_pub.publish(PoseStamped())

        rospy.sleep(2)  # Give some time for the nodes to process

        self.assertTrue(self.fused_data_received)

if __name__ == '__main__':
    unittest.main()
