#!/usr/bin/env python3

import rospy, roslaunch
from enum import Enum
import os
import time
from std_msgs.msg import UInt8, Float64, String

class CoreNodeController():
    def __init__(self):
        self.ros_package_path = os.path.dirname(os.path.realpath(__file__))
        self.ros_package_path = self.ros_package_path.replace('snow_removal/nodes', '')
        self.sub_mode_control = rospy.Subscriber('/chatter', String, self.cbReceiveMode, queue_size=1)
  
        self.Launcher = Enum('Launcher', 'idle teleop_cam mapping save_map path_planning done_path auto done_auto' )


        self.current_mode = 'idle'

        
        self.launch_camera = False
        self.launch_mapping = False
        self.save_map = False  
        self.set_path = False     
        self.auto = False

        self.is_triggered = False
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)



        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.is_triggered == True:
                self.fnControlNode()
            
            loop_rate.sleep()

    def cbReceiveMode(self, mode_msg):
        rospy.loginfo("starts the progress with %s", mode_msg.data)
        
        self.current_mode = mode_msg.data
        self.is_triggered = True

  
    def fnControlNode(self): 
        if self.current_mode == 'teleop':
            rospy.loginfo("New trigger for camera")

            self.fnLaunch(self.Launcher.teleop_cam.value, True)

        elif self.current_mode == 'home':
            self.fnLaunch(self.Launcher.teleop_cam.value, False)
        
        elif self.current_mode == 'mapping':
            self.fnLaunch(self.Launcher.teleop_cam.value, False)
            time.sleep(2)
            self.fnLaunch(self.Launcher.mapping.value, True)
        
        elif self.current_mode == 'done_mapping':
            self.fnLaunch(self.Launcher.save_map.value, True)
            time.sleep(2)
            self.fnLaunch(self.Launcher.mapping.value, False)
            self.fnLaunch(self.Launcher.teleop_cam.value, True)
        
        elif self.current_mode == 'path_planning':
            self.fnLaunch(self.Launcher.teleop_cam.value, False)
            time.sleep(2)
            self.fnLaunch(self.Launcher.path_planning.value, True)

        elif self.current_mode == 'done_path':
            self.fnLaunch(self.Launcher.path_planning.value, False)
            time.sleep(2)
            self.fnLaunch(self.Launcher.teleop_cam.value, True)
        
        elif self.current_mode == 'auto':
            self.fnLaunch(self.Launcher.auto.value, True)
        
        elif self.current_mode == 'done_auto':
            self.fnLaunch(self.Launcher.auto.value, False)

        


            rospy.sleep(2)

   
    def fnLaunch(self, launch_num, is_start):
        if launch_num == self.Launcher.teleop_cam.value:
            if is_start == True:
                if self.launch_camera == False:
                    self.launch_camera_ = roslaunch.scriptapi.ROSLaunch()
                    self.launch_camera_ = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "snow_removal/launch/teleop_cam.launch"])
                    self.launch_camera = True
                    self.launch_camera_.start()
                else:
                    pass
            else:
                if self.launch_camera == True:
                    self.launch_camera = False
                    self.launch_camera_.shutdown()
                else:
                    pass   

        if launch_num == self.Launcher.mapping.value:
            if is_start == True:
                if self.launch_mapping == False:
                    self.launch_mapping_ = roslaunch.scriptapi.ROSLaunch()
                    self.launch_mapping_ = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "snow_removal/launch/mapping.launch"])
                    self.launch_mapping = True
                    self.launch_mapping_.start()
                else:
                    pass
            else:
                if self.launch_mapping == True:
                    self.launch_mapping = False
                    self.launch_mapping_.shutdown()
                else:
                    pass   
        if launch_num == self.Launcher.save_map.value:
            if is_start == True:
                if self.save_map == False:
                    self.save_map_ = roslaunch.scriptapi.ROSLaunch()
                    self.save_map_ = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "snow_removal/launch/save_map.launch"])
                    self.save_map = True
                    self.save_map_.start()
                else:
                    pass
            else:
                if self.save_map == True:
                    self.save_map = False
                    self.save_map_.shutdown()
                else:
                    pass   
        if launch_num == self.Launcher.set_path.value:
            if is_start == True:
                if self.set_path == False:
                    self.set_path_ = roslaunch.scriptapi.ROSLaunch()
                    self.set_path_ = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "snow_removal/launch/path_planning.launch"])
                    self.set_path = True
                    self.set_path_.start()
                else:
                    pass
            else:
                if self.set_path == True:
                    self.set_path = False
                    self.set_path_.shutdown()
                else:
                    pass   
        if launch_num == self.Launcher.auto.value:
            if is_start == True:
                if self.auto == False:
                    self.auto_ = roslaunch.scriptapi.ROSLaunch()
                    self.auto_ = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "snow_removal/launch/path_planning.launch"])
                    self.auto = True
                    self.auto_.start()
                else:
                    pass
            else:
                if self.auto == True:
                    self.auto = False
                    self.auto_.shutdown()
                else:
                    pass   
                           
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('core_node_controller')
    node = CoreNodeController()
    node.main()
