import rclpy
import cv2
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from matplotlib import pyplot as plt

# import time
from sensor_msgs.msg import Image, PointCloud2, CompressedImage, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
# from vision_msgs.msg import BoundingBox2D, BoundingBox3D, ObjectHypothesisWithPose

## create node
class RealsenseImageProcesing(Node):

    ## constructor
    def __init__(self):
        super().__init__('realsense_image_processing')
        self.get_logger().info('Realsense Image Processing Node Initialized')

        ## create qos profile
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.bridge = CvBridge()
        self.latest_depth = None
        self.camera_intrinsics = None

        # create subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )

        self.image_publisher = self.create_publisher(
            Image,
            'rs_node/cv_image',
            qos_profile
        )

        self.features_rgb = self.create_publisher(
            Float32MultiArray,
            'rs_node/rgb/features',
            qos_profile 
        )

        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            qos_profile
        )


    ## create callbacks
    def camera_info_callback(self, msg):
        self.camera_intrinsics = msg

    def image_callback(self, msg):
        self.latest_depth = msg

        # Returns a NumPy array
        cv_image = self.bridge.imgmsg_to_cv2(img_msg=self.latest_depth ,desired_encoding="passthrough")
        
        
        self.get_logger().info(f"Shape of np.array is {cv_image.shape}, {cv_image.dtype}")

        # TO DO: ROS2 param for ORB or SIFT
        algo = None
        # algo = 'sift'

        if algo == 'sift':
            img2 = self.sift_func(cv_img=cv_image)
        else: # default to ORB
            img2 = self.orb_func(cv_image=cv_image)
        
        

        output_img = self.bridge.cv2_to_imgmsg(img2)
        self.image_publisher.publish(output_img)

        # Verify a feature is observed:
        # return keypoints as u, v coordinates
        # kp = keypoints[0]
        # u, v = (kp.pt[0], kp.pt[1])
        # self.get_logger().info(f"features are U: {u}, V: {v}")

    # SIFT function
    def sift_func(self, cv_img):
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB) 
        sift = cv2.SIFT.create()
        keypoints = sift.detect(gray, None)

        output_img = cv2.drawKeypoints(gray, keypoints, cv_img)        
        return output_img

    # ORB function
    def orb_func(self, cv_image):
        if cv_image.dtype != np.uint8:
            cv_image = (255 * (cv_image / cv_image.max())).astype(np.uint8)
        # Keypoints from ORB
        orb = cv2.ORB.create()
        keypoints = orb.detect(cv_image, None)

        ## NO KEYPOINTS FOUND
        if not keypoints:
            self.get_logger().info("No keypoints found")
            return
        kp, des = orb.compute(cv_image, keypoints=keypoints)
        
        output_img = cv2.drawKeypoints(cv_image, kp, None, color=(0,255,0), flags=0)
        return output_img



def main(args=None):
    rclpy.init(args=args)
    node = RealsenseImageProcesing()
    rclpy.spin(node)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    
    node.destroy_node()
    rclpy.shutdown()