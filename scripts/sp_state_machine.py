#!/usr/bin/env python
# BSD 3-Clause License
#
# Copyright (c) 2019, Matheus Nascimento, Luciana Reys
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Professor: Wouter Caarls
# Students: Matheus do Nascimento Santos 1920858  (@matheusns)
#           Luciana Reys 1920856 (@lsnreys)

# OpenCV modules
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Ros modules
import rospy
from strawberry_detector.srv import *
from dobot.srv import *
from geometry_msgs.msg import Pose2D, TransformStamped
from sensor_msgs.msg import Image, JointState, CameraInfo
from std_msgs.msg import Float64
from image_geometry import PinholeCameraModel
from tf2_geometry_msgs import PointStamped, Vector3Stamped
import tf2_ros

# Other modules
import sys


class SPStateMachine:
    def __init__(self):
        rospy.init_node('sp_state_machine')
        self.init_members_variables()
        self.init_ros_channels()
        self.execute()

    def init_members_variables(self):
        self.debug = False
        self.bridge = CvBridge()
        self.cv_image = None
        self.at_home = False
        self.is_gripper_open = False
        self.STATES = {0: "SYSTEM IS NOT IN OPERATION", 1: "READY TO START", 2: "AT HOME POSITION",
                       3: "ACQUIRING IMAGE", 4: "PROCESSING_IMAGE", 5: "MOVING TO THE PICK POSITION", 6: "GRIPPING",
                       7: "MOVING TO THE PLACE POSITION", 8: "RELEASING", 9: "ERROR"}
        self.current_state = 0
        self.HOME_X = 202
        self.HOME_Y = 0
        self.HOME_Z = 160.49
        self.HOME_R = 0.0
        self.FAKE_POSES = [(236,36), (255, 45),(275, 52)]

    def init_ros_channels(self):
        self.image_sub = rospy.Subscriber("/img_acquistion/camera/image", Image, self.callback, queue_size=1)

    def callback(self, msg):
        rospy.logdebug("Received an image!")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if self.debug:
                print "Here"
                cv2.namedWindow(self.ADJUSTMENT_WINDOW, cv2.WINDOW_NORMAL)
                cv2.imshow(self.ADJUSTMENT_WINDOW, self.cv_image)
                cv2.waitKey(1)

        except CvBridgeError, e:
            print(e)

    def system_check(self):
        # todo check if the manipulator nodes are up
        # todo check if the camera nodes are up
        # If is everything ok, the system is ready to start
        if self.clear_all_alarms():
            self.current_state = 1
            self.print_system_state()
        else:
            self.current_state = 9
            self.print_system_state()

    def clear_all_alarms(self):
        rospy.wait_for_service('/DobotServer/ClearAllAlarmsState')
        try:
            rospy.logdebug("[SM] STW_PICKER: Clearing alarms")
            alarms = rospy.ServiceProxy('/DobotServer/ClearAllAlarmsState', ClearAllAlarmsState)
            result = alarms()
            return True
        except rospy.ServiceException, e:
            rospy.logfatal("[SM] STW_PICKER: Failed during clear alarms calling")
            self.current_state = 9
            self.print_system_state()
            return False

    def set_home_params(self):
        rospy.wait_for_service('/DobotServer/SetHOMEParams')
        try:
            set_home_params = rospy.ServiceProxy('/DobotServer/SetHOMEParams', SetHOMEParams)
            result = set_home_params(self.HOME_X, self.HOME_Y, self.HOME_Z, self.HOME_R, False)
            return True
        except rospy.ServiceException, e:
            rospy.logfatal("[SM] STW_PICKER: Failed during Home position setting")
            self.current_state = 9
            self.print_system_state()
            return False

    def print_system_state(self, state=None):
        if state is not None:
            rospy.loginfo("[SM] STW_PICKER: " + self.STATES[state])
        else:
            rospy.loginfo("[SM] STW_PICKER: " + self.STATES[self.current_state])

    def execute(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.system_check() #Changes the current state to 1
            # Move to home position
            if not self.at_home:
                self.move_manipulator_2_home()
            if self.is_gripper_open:
                self.cmdGripper(compressor=True, open_gripper=False)

            if self.current_state == 2:  # Ensure it is at home position
                self.current_state = 3
                self.print_system_state()
                rospy.wait_for_service('/stw_detector/img_proc')
                try:
                    strawberry_pose = rospy.ServiceProxy('/stw_detector/img_proc', StwPose)

                    next_stw_pose = strawberry_pose()
                    self.current_state = 4
                    self.print_system_state()
                    rospy.sleep(3)
                    rospy.loginfo("[SM] STW_PICKER: Detected Strawberry at: " + str((next_stw_pose.y/4) + 160) + " , " + str((next_stw_pose.x/4) - 48))
                    rospy.sleep(2)
                    if (next_stw_pose.x != 0 and next_stw_pose.y != 0):
                        # Open Gripper
                        self.cmdGripper(compressor=True, open_gripper=True)
                        # Goal to pick position
                        self.go_2_pick_pose(next_stw_pose.x, next_stw_pose.y)
                        # Close Gripper
                        self.cmdGripper(compressor=True, open_gripper=False, state=6)
                        # Goal to place position
                        self.go_2_place_pose()
                        # Open Gripper
                        self.cmdGripper(compressor=True, open_gripper=True, state=8)
                        rospy.sleep(3)
                        self.go_2_pose([137, -215, 50, 0.0])

                except rospy.ServiceException, e:
                    rospy.logfatal("[SM] STW_PICKER: Failed during image processing")

                rospy.sleep(2)

            rate.sleep()

    def move_manipulator_2_home(self):
        rospy.wait_for_service('/DobotServer/SetHOMECmd')
        try:
            set_home_cmd = rospy.ServiceProxy('/DobotServer/SetHOMECmd', SetHOMECmd)
            next_stw_pose = set_home_cmd()
            self.current_state = 2
            rospy.sleep(15)
            self.at_home = True
            self.print_system_state()
        except rospy.ServiceException, e:
            rospy.logfatal("[SM] STW_PICKER: Failed while getting home position")

    def go_2_pick_pose(self, xi, yi):
        xr = (yi/4) + 160
        if (xi > 0 and xi <= 320):
            yr = (xi/4)*(-1) + 75 #Condition to match robots reference
        else:
            yr = (xi / 4) - 75

        print "xr = " + str(xr) + " , yr = " + str(yr)
        self.clear_all_alarms()
        self.current_state = 5
        self.print_system_state()
        self.go_2_pose([xr, yr, -30, 0.0])
        rospy.sleep(3)
        self.go_2_pose([xr, yr, 50, 0.0])

    def go_2_place_pose(self):
        self.clear_all_alarms()
        self.current_state = 7
        self.print_system_state()
        self.go_2_pose([137, -215, 50, 0.0])
        rospy.sleep(2)
        self.go_2_pose([137, -215, -25, 0.0])
        rospy.sleep(3)

    def go_2_pose(self, place_pose=[137, -215, -25, 0.0]):
        rospy.wait_for_service('/DobotServer/SetPTPCmd')
        try:
            set_home_cmd = rospy.ServiceProxy('/DobotServer/SetPTPCmd', SetPTPCmd)
            next_goal_pose = set_home_cmd(1, place_pose[0], place_pose[1], place_pose[2], place_pose[3])
            self.at_home = False
        except rospy.ServiceException, e:
            rospy.logfatal("[SM] STW_PICKER: Failed while getting pick position")

    def cmdGripper(self, compressor, open_gripper, state=None):
        #print "CLosing gripper..."
        rospy.wait_for_service('/DobotServer/SetEndEffectorGripper')
        try:
            set_home_cmd = rospy.ServiceProxy('/DobotServer/SetEndEffectorGripper', SetEndEffectorGripper)
            result = set_home_cmd(compressor, not open_gripper, False)
            rospy.sleep(2)
            self.is_gripper_open = open_gripper
            if state is not None:
                self.current_state = state
                self.print_system_state()


            if  compressor:
                self.cmdGripper(False, open_gripper)
            return

        except rospy.ServiceException, e:
            rospy.logfatal("[SM] STW_PICKER: Failed while getting pick position")



def main(args):
    main_app = SPStateMachine()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
