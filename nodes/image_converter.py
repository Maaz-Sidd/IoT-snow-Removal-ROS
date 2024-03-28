#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2

def pgm_to_jpeg_publisher():
    # Initialize ROS node
    rospy.init_node('pgm_to_jpeg_publisher', anonymous=True)
    
    # Define publisher for compressed image
    image_pub = rospy.Publisher('/compressed_map', CompressedImage, queue_size=10)

    # Path to the PGM file
    pgm_file_path = '/home/maaz/catkin_ws/src/snow_removal/maps/map.pgm'

    # Read the PGM file
    pgm_image = cv2.imread(pgm_file_path, cv2.IMREAD_GRAYSCALE)
    if pgm_image is None:
        rospy.logerr("Failed to read PGM file")
        return

    # Convert the PGM image to JPEG
    success, jpeg_data = cv2.imencode('.jpg', pgm_image)
    if not success:
        rospy.logerr("Failed to encode image as JPEG")
        return

    # Create a ROS CompressedImage message
    compressed_image_msg = CompressedImage()
    compressed_image_msg.format = "jpeg"
    compressed_image_msg.data = jpeg_data.tostring()

    # Publish the compressed image
    while not rospy.is_shutdown():
        image_pub.publish(compressed_image_msg)
        rospy.sleep(0.1)  # Publish at 10 Hz

if __name__ == '__main__':
    try:
        pgm_to_jpeg_publisher()
    except rospy.ROSInterruptException:
        pass