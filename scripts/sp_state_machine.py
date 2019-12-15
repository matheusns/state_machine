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
        self.STATES = {0: "SYSTEM IS NOT IN OPERATION", 1: "READY TO START", 2: "AT HOME POSITION", 3: "ACQUIRING IMAGE",
                       4: "PROCESSING_IMAGE",
                       5: "MOVING TO THE PICK POSITION", 6: "MOVING TO THE PLACE POSITION"}
        self.current_state = 0

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
        self.current_state = 1
        self.print_system_state()

    def print_system_state(self, state=None):
        if state is not None:
            rospy.loginfo("[SM] STW_PICKER: " + self.STATES[state])
        else:
            rospy.loginfo("[SM] STW_PICKER: " + self.STATES[self.current_state])

    def execute(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.system_check()
            # todo go2HomePose
            if self.current_state == 1: #Ensure it is at home position
                self.current_state = 3
                self.print_system_state()
                rospy.wait_for_service('/stw_detector/img_proc')
                try:
                    strawberry_pose = rospy.ServiceProxy('/stw_detector/img_proc', StwPose)

                    self.current_state = 4
                    self.print_system_state()

                    next_stw_pose = strawberry_pose()
                    rospy.loginfo("Detected Strawberry at: " + str(next_stw_pose.x) + " , " + str(next_stw_pose.y))
                except rospy.ServiceException, e:
                    rospy.logfatal("[SM] STW_PICKER: Failed during image processing")
                #todo Pick(x,y)
                #todo Place(x,Y)
            rate.sleep()


def main(args):
    main_app = SPStateMachine()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
