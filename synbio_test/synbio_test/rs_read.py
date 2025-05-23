import rclpy
import cv2
from rclpy.node import Node
import numpy as np
# import time
from sensor_msgs.msg import Image, PointCloud2, CompressedImage #, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
# from realsense2_camera_msgs.msg import RGBDs
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

        ## create subscribers
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_raw/compressed',
            self.image_callback,
            qos_profile
        )

        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            qos_profile
        )

        self.image_publisher = self.create_publisher(
            Image,
            'rs_node/cv_image',
            qos_profile
        )

        # self.camera_info_subscriber = self.create_subscription(
        #     CameraInfo,
        #     '/camera/camera/color/camera_info',
        #     self.camera_info_callback,
        #     qos_profile
        # )

    ## create callbacks
    def image_callback(self, msg):
        # self.get_logger().info('Received Image')
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(msg)
        cv_image = cv2.resize(cv_image, (640, 480))
        # self.get_logger().info('Converting Image to OpenCV Format')
        ## extract green objects from the image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])

        # mask = (hsv_image[:, :, 0] >= lower_green[0]) & (hsv_image[:, :, 0] <= upper_green[0]) & \
                # (hsv_image[:, :, 1] >= lower_green[1]) & (hsv_image[:, :, 1] <= upper_green[1]) & \
            #   (hsv_image[:, :, 2] >= lower_green[2]) & (hsv_image[:, :, 2] <= upper_green[2])
        
        # mask = m  ask.astype(np.uint8) * 255

        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        found_contour = False
        for contour in contours:
            if cv2.contourArea(contour) > 100:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, 'Green Object', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                found_contour = True

        if found_contour:
            self.get_logger().info('Green Object Found')
            cv2.imshow('Green Object Detection', cv_image)
            cv2.waitKey(1)
            self.publish_cv_image(cv_image)
        else:
            # self.get_logger().info('No Green Object Found')
            self.publish_cv_image(cv_image)
        # self.get_logger().info('Added boxes')

        ## keep window open as long as the node is running, update with new images as they come in

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received Point Cloud')

        # do something with the point cloud data


    # def camera_info_callback(self, msg):
    #     self.get_logger().info('Received Camera Info')

        ## process camera info here
    def process_camera_info(self, msg):
        # Extract camera parameters from the CameraInfo message
        width = msg.width
        height = msg.height
        K = np.array(msg.k).reshape(3, 3)
        D = np.array(msg.d)
        R = np.array(msg.r).reshape(3, 3)
        P = np.array(msg.p).reshape(3, 4)

        # Print or use the camera parameters as needed
        self.get_logger().info(f'Camera Info: {width}x{height}, K: {K}, D: {D}, R: {R}, P: {P}')

    def publish_cv_image(self, cv_image):
        image_header = Image()
        image_header.header.stamp = self.get_clock().now().to_msg()
        image_header.header.frame_id = 'camera_frame'
        # Convert OpenCV image to ROS message
        brigde = CvBridge()
        # self.get_logger().info('Publishing Image')
        ros_image = brigde.cv2_to_imgmsg(cv_image, encoding='bgr8', header=image_header.header)
        # self.get_logger().info('s Image to ROS Format')
        # publish the image
        self.image_publisher.publish(ros_image)

        # ros_image.height = cv_image.shape[0]
        # ros_image.width = cv_image.shape[1]
        # ros_image.step = cv_image.shape[1] * 3
        # ros_image.data = np.array(cv_image).tobytes()
        # ros_image.is_bigendian = False


        


def main(args=None):
    rclpy.init(args=args)
    rs_read = RealsenseImageProcesing()
    # executor = MultiThreadedExecutor()

    # executor.add_node(rs_read)
    # executor.spin()
    rclpy.spin(rs_read)

    rs_read.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()