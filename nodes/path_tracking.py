#!/usr/bin/env python3
import rospy
import rospkg
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv

class PoseSaver:
    def __init__(self):
        rospy.init_node('pose_saver', anonymous=True)
        self.pose_sub = rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.pose_data = []

    def pose_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        print(str(msg.pose.pose.position)+"\n")
        _x = position.x
        _y = position.y
        plt.scatter(_x,_y)
        plt.pause(0.05)
        self.pose_data.append([position.x, position.y, position.z])

    def save_pose_to_csv(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('snow_removal')  # Replace 'your_package_name' with your actual package name
        csv_file = package_path + '/pose_data.csv'

        with open(csv_file, mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(['Position_X', 'Position_Y', 'Theta'])
            writer.writerows(self.pose_data)

        rospy.loginfo("Pose data saved to {}".format(csv_file))

if __name__ == '__main__':
    try:
        pose_saver = PoseSaver()
        rospy.sleep(2)  # Wait for some time to collect enough pose data
        rospy.on_shutdown(pose_saver.save_pose_to_csv)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass