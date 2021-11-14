#!/usr/bin/python2.7
"""
TODO: Description
"""
import rospy
import struct
import socket
import cv2
import numpy as np
import ros_numpy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import pdb

__license__ = "MIT"
__author__ = "Aldo Teran Espinoza"
__author_email__ = "aldot@kth.se"
__status__ = "Development"

class ImageConverter:
    """
    Class to handle Sonar image.
    """

    def __init__(self):
        """
        Not done yet.
        """
        # TODO: get this as a rosparam
        self.publish_edges = True
        self.publish_pointcloud = False

        self.udp_socket = socket.socket(socket.AF_INET,
                                        socket.SOCK_DGRAM)

        # TODO: Which approach is better?
        self.sonar_result = {
            "image_msg" : None,
            "ping_result" : None,
            "image_array" : None

        }

        # Image processing parameters
        self.image_threshold = 100

        # Init bearing arrays
        self.low_freq_brgs = None
        self.high_freq_brgs = None
        self._init_bearings()

        self.bridge = CvBridge()

        self.sonar_image_topic = "/sonar_image"
        self.sonar_ping_topic = "/simple_ping_result"
        # self.sonar_image_topic = rospy.get_param("sonar_image_topic")
        # self.sonar_ping_topic = rospy.get_param("sonar_ping_topic")
        self.point_pub = rospy.Publisher("/sonar_points", PointCloud2, queue_size=1)
        self.image_pub = rospy.Publisher("/processed_image", Image, queue_size=1)

        # Init Subscribers
        rospy.Subscriber(self.sonar_image_topic,
                        Image,
                        self._sonar_image_callback)
        rospy.Subscriber(self.sonar_ping_topic,
                        Float64MultiArray,
                        self._sonar_ping_callback)

    def _init_bearings(self):
        """
        Initialize the bearing vectors (since constant)
        to avoid redundant computations.
        """
        # Calculate for low frequency
        iterable = (-65+x*0.5078125 for x in range(256))
        bearings = np.fromiter(iterable, float)
        self.low_freq_brgs = np.expand_dims(bearings, 0)

        # Calculate for high frequency
        iterable = (-35+x*0.2734375 for x in range(256))
        bearings = np.fromiter(iterable, float)
        self.high_freq_brgs = np.expand_dims(bearings, 0)

    def _sonar_image_callback(self, image):
        # Get data, save in dictionary both message and image array
        self.sonar_result["image_msg"] = image
        self.sonar_result["image_array"] = self.bridge.imgmsg_to_cv2(image, "mono8")

    def _sonar_ping_callback(self, data):
        self.sonar_result["ping_result"] = data

    def proc_and_pub_pointcloud(self):
        """
        Build pointcloud and publish.
        """
        image = self.sonar_result["image_array"]

        # Compute range and bearing maps using the ping result
        ping_result = self.sonar_result["ping_result"].data
        resolution = ping_result[8]
        [rows, cols] = np.shape(image)
        range_max = rows*resolution
        ranges = np.linspace(0,range_max, rows)

        # Check image frequency
        high_freq = (ping_result[2] > 1000000)
        if high_freq:
            # bearings = np.tile(self.high_freq_brgs, (rows, 1))
            bearing_mesh, range_mesh = np.meshgrid(self.high_freq_brgs, ranges)
        else:
            # bearings = np.tile(self.low_freq_brgs, (rows, 1))
            bearing_mesh, range_mesh = np.meshgrid(self.high_freq_brgs, ranges)

        # TODO: Turn this into a gate (like the MBES)
        # Threshold image
        ret, image = cv2.threshold(image, self.image_threshold, 255, cv2.THRESH_TOZERO)
        # TODO: Maybe can tune parameters better, good results with current values.
        # Detect edges with second Laplacian and processes out of the image
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(5,5))
        image = clahe.apply(image)
        image = cv2.Canny(image, 200, 255, L2gradient=True)
        kernel = np.ones((5,5), np.uint8)
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
        edges = np.argmax(image, axis=0)
        cols = np.arange(0,len(edges),1)
        image_out = np.zeros(image.shape, np.uint8)
        image_out[edges,cols] = 255

        # TODO: send the closest point's distance via UDP to Dune
        min_distance = ranges[np.min(edges)]
        self.udp_socket.sendto(min_distance, ("", 7777))

        if self.publish_edges:
            # Publish processed edges as an image
            image_msg = self.bridge.cv2_to_imgmsg(image_out, encoding="passthrough")
            self.image_pub.publish(image_msg)

        if self.publish_pointcloud:
            # Publish processed edges as a 2D pointcloud
            pointcloud_msg = self._build_pcl2_msg(image_out)
            self.point_pub.publish(pointcloud_msg)


    def _build_pcl2_msg(self, image):
        [x_coords, y_coords] = self._polar_to_cartesian(image)
        # Get rid of points in empty water
        image = image.ravel()
        edges = (image>0)
        filtered_image = image[edges]
        x_coords = x_coords.ravel()[edges]
        y_coords = y_coords.ravel()[edges]

        # Create cloud record array
        cloud_recarray = np.rec.array([(x_coords),
                                       (y_coords),
                                       (np.zeros(len(x_coords))),
                                       (filtered_image)],
                                      dtype=[('x', 'f4'),
                                             ('y', 'f4'),
                                             ('z', 'f4'),
                                             ('intensity', 'uint8')])

        # Build pontcloud2 message with record array
        pcl2_msg = ros_numpy.point_cloud2.array_to_pointcloud2(cloud_recarray,
                                                               rospy.Time.now(),
                                                               "/sonar")

        return pcl2_msg

    def _polar_to_cartesian(self, image):
        ping_result = self.sonar_result["ping_result"].data
        resolution = ping_result[4]
        [rows, cols] = np.shape(image)

        # Check if image was taken in high frequency
        high_freq = (ping_result[2] > 1000000)
        if high_freq:
            bearings = np.tile(self.high_freq_brgs, (rows, 1))
        else:
            bearings = np.tile(self.low_freq_brgs, (rows, 1))

        # Build array with ranges
        iterable = (i*resolution for i in range(rows))
        ranges = np.fromiter(iterable, float)
        ranges = np.expand_dims(ranges, 1)
        ranges = np.tile(ranges, (1, cols))

        # Convert to to cartesian
        [x, y] = cv2.polarToCart(ranges, bearings, angleInDegrees=True)

        return [x, y]


def main():
    """
    Main method for the ROS node.
    """
    rospy.init_node('image_to_pointcloud')
    rospy.loginfo("Starting sonar image to pointcloud node...")
    converter = ImageConverter()

    rate = rospy.Rate(10)
    rospy.sleep(3.0)
    while not rospy.is_shutdown():
        converter.proc_and_pub_pointcloud()
        rate.sleep()

if __name__ == "__main__":
    main()