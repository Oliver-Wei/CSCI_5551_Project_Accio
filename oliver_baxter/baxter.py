from __future__ import print_function
import serial
import rospy
import roslib

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy
import math
import os
import sys
import string
import time
import random
import tf
import serial

from std_msgs.msg import String
from sensor_msgs.msg import Image

import baxter_interface

#from moveit_commander import conversions
import conversions

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

from baxter_interface import CameraController

from baxter_core_msgs.msg import (
    CameraControl,
    CameraSettings,
)
from baxter_core_msgs.srv import (
    CloseCamera,
    ListCameras,
    OpenCamera,
)

# load the package manifest
#roslib.load_manifest("activerobots")

# initialise ros node
rospy.init_node("test", anonymous = True)

# directory used to save analysis images
image_directory = os.getenv("HOME") + "/accio/"

task = 0

class locator:
    """
    Interface class for controlling camera settings on the Baxter robot.
    """

    # Valid resolutions
    MODES = [
             (1280, 800),
             (960, 600),
             (640, 400),
             (480, 300),
             (384, 240),
             (320, 200),
             ]

    # Used to represent when the camera is using automatic controls.
    # Valid for exposure, gain and white balance.
    CONTROL_AUTO = -1       
    def __init__(self,arm,distance):
        global image_directory

        baxter_interface.RobotEnable().enable()

        # arm ("left" or "right")
        self.limb           = arm
        self.distance       = distance
        self.limb_interface = baxter_interface.Limb(self.limb)

        
        self.gripper = baxter_interface.Gripper(arm)
        self.gripper.calibrate()

        self.other_limb_interface = baxter_interface.Limb("left")
        self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))
        self.limb_interface.set_joint_position_speed(0.1)
        self.other_limb_interface.set_joint_position_speed(0.1)
        
        #self.open_camera(arm)
        self.bridge = CvBridge()

        # image directory
        self.image_dir = image_directory

        # flag to control saving of analysis images
        self.save_images = True

        # required position accuracy in metres
        # self.ball_tolerance = 0.005
        # self.tray_tolerance = 0.05

        # number of balls found
        # self.balls_found = 0

        # start positions
        self.origin_x = 0.6                        # x     = front back   0.50    0.70(good)
        self.origin_y = -0.1                        # y     = left right   0.30    -0.1(good)
        self.origin_z = 0	                        # z     = up down    0.15
        self.roll        = -1.0 * math.pi              # roll  = horizontal
        self.pitch       = 0.0 * math.pi               # pitch = vertical
        self.yaw         = 0.0 * math.pi               # yaw   = rotation

        self.pose = [self.origin_x, self.origin_y, self.origin_z,     \
                     self.roll, self.pitch, self.yaw]


    def open_camera(self,arm):
        
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=10)
        
        self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)   
        self.subscribe_to_camera(arm)

        # reset cameras

        """
        Constructor.
        @param name: camera identifier.  You can get a list of valid
                     identifiers by calling the ROS service /cameras/list.
                     Expected names are right_hand_camera, left_hand_camera
                     and head_camera.  However if the cameras are not
                     identified via the parameter server, they are simply
                     indexed starting at 0.
        """
        if (arm == "left"):
            self._id = "left_hand_camera"
        elif (arm == "right"):
            self._id = "right_hand_camera"
        else:
            self._id = "head_camera"

        # camera parameters (NB. other parameters in open_camera)
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = 0.045                      # camera gripper offset
        self.cam_y_offset = -0.01

        list_svc = rospy.ServiceProxy('/cameras/list', ListCameras)
        rospy.wait_for_service('/cameras/list', timeout=10)
        if not self._id in list_svc().cameras:
            raise AttributeError(
                ("Cannot locate a service for camera name '{0}'. "
                "Close a different camera first and try again.".format(self._id)))

        self._settings = CameraSettings()
        self.width = self._settings.width = 960
        self.height = self._settings.height = 600
        self._settings.fps = 20
        self._open = False 

        self._open_svc = rospy.ServiceProxy('/cameras/open', OpenCamera)
        self._close_svc = rospy.ServiceProxy('/cameras/close', CloseCamera)
        self.image_num = 0


    def left_camera_callback(self, data):
        #self.camera_callback(data, "Left Hand Camera")
        pass

    # right camera call back function
    def right_camera_callback(self, data):
        #self.camera_callback(data, "Right Hand Camera")
        self.callback(data)
        #rospy.loginfo("I heard you")

    # head camera call back function
    def head_camera_callback(self, data):
        #self.camera_callback(data, "Head Camera")
        pass

    def subscribe_to_camera(self, camera):
        if camera == "left":
            callback = self.left_camera_callback
            camera_str = "/cameras/left_hand_camera/image"
        elif camera == "right":
            callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        elif camera == "head":
            callback = self.head_camera_callback
            camera_str = "/cameras/head_camera/image"
        else:
            sys.exit("ERROR - subscribe_to_camera - Invalid camera")

        camera_sub = rospy.Subscriber(camera_str, Image, callback)    

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.imshow("Image window", cv_image)
            if (self.image_num <= 10):
                cv2.imwrite(self.image_dir+"test_img_%d.png"%self.image_num,cv_image)  #save images for recognition
            cv2.waitKey(3)    #**
            self.image_num += 1

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    # convert Baxter point to image pixel
    def baxter_to_pixel(self, pt, dist):
        x = (self.width / 2)                                                         \
          + int((pt[1] - (self.pose[1] + self.cam_y_offset)) / (self.cam_calib * dist))
        y = (self.height / 2)                                                        \
          + int((pt[0] - (self.pose[0] + self.cam_x_offset)) / (self.cam_calib * dist))

        return (x, y)

    # convert image pixel to Baxter point
    def pixel_to_baxter(self, px, dist):
        x = ((px[1] - (self.height / 2)) * self.cam_calib * dist)                \
          + self.pose[0] + self.cam_x_offset
        y = ((px[0] - (self.width / 2)) * self.cam_calib * dist)                 \
          + self.pose[1] + self.cam_y_offset

        return (x, y)


    def baxter_ik_move(self, limb, rpy_pose):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if self.limb == limb:
                self.limb_interface.move_to_joint_positions(limb_joints)
            else:
                self.other_limb_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display
            #self.splash_screen("Invalid", "move")
            # little point in continuing so exit with error message
            print("requested move =", rpy_pose)
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

        if self.limb == limb:               # if working arm
            quaternion_pose = self.limb_interface.endpoint_pose()
            position        = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved
            self.pose = [position[0], position[1],                                \
                         self.pose[2], self.pose[3], self.pose[4], self.pose[5]]

    # find distance of limb from nearest line of sight object
    def get_distance(self, limb):
        if limb == "left":
            dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
        elif limb == "right":
            dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
        else:
            sys.exit("ERROR - get_distance - Invalid limb")

        # convert mm to m and return distance
        return float(dist / 1000.0)

    # update pose in x and y direction
    def update_pose(self, dx, dy):
        x = self.pose[0] + dx
        y = self.pose[1] + dy
        pose = [x, y, self.pose[2], self.roll, self.pitch, self.yaw]
        self.baxter_ik_move(self.limb, pose)
    
    def pick_and_place(self,object_x,object_y):
        # move towards the above of the object
        self.pose[0] = object_x
        self.pose[1] = object_y
   
        self.baxter_ik_move(self.limb, self.pose)

        # catch the object
        self.pose[2] = self.pose[2] - self.distance + 0.122
        self.baxter_ik_move(self.limb, self.pose)
        self.gripper.close()


        # move back
        
        self.pose[2] = self.origin_z
        self.baxter_ik_move(self.limb, self.pose)
        
        self.pose = (self.origin_x,
                self.origin_y,
                self.origin_z,
                self.roll,
                self.pitch,
                self.yaw)
        self.baxter_ik_move(self.limb, self.pose)

        # place the item over turtlebot
        self.pose = (self.origin_x-0.05,
                self.origin_y-0.65,
                self.origin_z,
                self.roll,
                self.pitch,
                self.yaw)
        self.baxter_ik_move(self.limb, self.pose)

        # place the item on turtlebot
        self.pose = (self.origin_x-0.05,
                self.origin_y-0.65,
                self.origin_z-0.4,
                self.roll,
                self.pitch,
                self.yaw)
        self.baxter_ik_move(self.limb, self.pose)
        self.gripper.open()

        # move back
        self.pose = (self.origin_x,
                self.origin_y,
                self.origin_z,
                self.roll,
                self.pitch,
                self.yaw)
        self.baxter_ik_move(self.limb, self.pose)
    

# read the setup parameters from setup.dat

def get_setup():
    global image_directory
    file_name = image_directory + "setup.dat"

    try:
        f = open(file_name, "r")
    except ValueError:
        sys.exit("ERROR: golf_setup must be run before golf")

    # find limb
    s = string.split(f.readline())
    if len(s) >= 3:
        if s[2] == "left" or s[2] == "right":
            limb = s[2]
        else:
            sys.exit("ERROR: invalid limb in %s" % file_name)
    else:
        sys.exit("ERROR: missing limb in %s" % file_name)

    # find distance to table
    s = string.split(f.readline())
    if len(s) >= 3:
        try:
            distance = float(s[2])
        except ValueError:
            sys.exit("ERROR: invalid distance in %s" % file_name)
    else:
        sys.exit("ERROR: missing distance in %s" % file_name)

    return limb, distance

def cv_position(): # to be used
    px = [0,0]
    while True:
        try:
            file_name = image_directory + "cv_position.dat"
            f = open(file_name, "r")
            s = string.split(f.readline())
            if len(s) >= 3:
                x = float(s[2])   #/992*960
            s = string.split(f.readline())
            if len(s) >= 3:
                y = float(s[2])   #/558*600
            px = [x,y]
        except:
            pass
        if (px != [0,0]):
            break
    return px   
    

def read_and_save_task(ser):
    # s = ""
    # while (s == ""):
    s = ser.read(2)
    print(s)
    f = open(image_directory+"task_num.dat", "w")
    task = "task_num = %s\n" % s
    f.write(task)         
    f.close()
    #return int(s)

def main(args):
    # setup the baxter
    limb, distance = "right", 0.383
    lc = locator(limb,distance)
    lc.pose = (lc.origin_x,
                lc.origin_y,
                lc.origin_z,
               lc.roll,
                lc.pitch,
                lc.yaw)
    lc.baxter_ik_move(lc.limb, lc.pose)
    print("Open camera")
    lc.open_camera(lc.limb)
    ser = serial.Serial("/dev/ttyUSB0",57600)
    print("Serial opened")
    image_num = 0
    read_and_save_task(ser)
    print("Task number readed")
    px = cv_position()    #read the position in image
    (object_x, object_y) = lc.pixel_to_baxter(px,distance)
    print("Start picking")
    lc.pick_and_place(object_x,object_y)
    ser.write("finish")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main(sys.argv)
