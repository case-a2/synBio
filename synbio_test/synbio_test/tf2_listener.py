import math
import numpy as np

from geometry_msgs.msg import TransformStamped, Point, Pose
import rclpy.time
from visualization_msgs.msg import Marker, MarkerArray

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.logging import get_logger


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class FrameListener(Node):

    def __init__(self):
        super().__init__('tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
            name='target_frame',
            value='camera_link'
            ).get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.point_to = self.point_from = Point()
        self.pose_to = self.pose_from = Pose()

        # self.subscriber = self.create_subscription(TFMessage, '/tf', 1)

        # Create marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_markers', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        self.from_frame_rel = self.get_parameter('target_frame')
        self.to_frame_rel = 'ur3e1_tool0'
        # arrow = to_frame = from_frame = TransformStamped()
        # pose_to = pose_from = Pose()

        # If ~MSG~ is not None
        if self.tf_listener.buffer is not None :
            # Look up for the transformation between target_frame and TOOL0 frames
            # and send marker displays between TOOL0 to reach target_frame
            try:
                marker_array = MarkerArray()

                ## TRANSFORMS ARE NOT CHANGING
                arrow = self.tf_buffer.lookup_transform(
                    self.to_frame_rel,
                    self.from_frame_rel,
                    rclpy.time.Time())
                # self.get_logger().info("Arrow tf")
                to_frame = self.tf_buffer.lookup_transform(
                    self.to_frame_rel,
                    'world',
                    rclpy.time.Time())
                
                ee_tf = to_frame.transform.translation
                arrow_frame = arrow.transform.translation
                self.point_to.x = arrow_frame.x
                self.point_to.y = arrow_frame.y
                self.point_to.z = arrow_frame.z

                
                self.get_logger().info("EE-tf")
                # print(np.array([self.point_to.x, self.point_to.y, self.point_to.z]))
                print(np.array([arrow_frame.x, arrow_frame.y, arrow_frame.z]))
                print("\n")


                # arrow starts at camera
                from_frame = self.tf_buffer.lookup_transform(
                    'world',
                    self.from_frame_rel,
                    rclpy.time.Time())
                
                camera_tf = from_frame.transform.translation

                self.point_from.x = camera_tf.x
                self.point_from.y = camera_tf.y
                self.point_from.z = camera_tf.z
                self.get_logger().info("Camera-tf")
                # print(np.array([self.point_from.x, self.point_from.y, self.point_from.z]))
                print(np.array([camera_tf.x, camera_tf.y, camera_tf.z]))

                # ARROW POINTS
                # camera_link -> world
                
                

                
                # self.pose_from.orientation = from_frame.transform.rotation


                # self.pose_to.position.x = to_frame.transform.translation.x
                # self.pose_to.position.y = to_frame.transform.translation.y
                # self.pose_to.position.z = to_frame.transform.translation.z
                # self.pose_to.orientation = to_frame.transform.rotation
                

                # arrow_length = np.array([arrow_tf.x, arrow_tf.y, arrow_tf.z])

                # ARROW MESSAGE
                # points = [Point(x=0.0, y=0.0, z=0.0),Point(x=0.0,y=1.0,z=3.0)]
                points = [self.point_from, self.point_to]
                arrow_msg = Marker()
                arrow_msg.action = Marker.ADD
                arrow_msg.ns = "labels"
                arrow_msg.id = 0
                arrow_msg.header.frame_id = "world"
                arrow_msg.header.stamp = arrow.header.stamp
                arrow_msg.scale.x = 0.05
                arrow_msg.scale.y = 0.20
                arrow_msg.scale.z = 0.20 # np.linalg.norm(arrow_length)
                arrow_msg.type = Marker.ARROW # arrow
                arrow_msg.points =  points
                arrow_msg.color.a = 1.0
                arrow_msg.color.g = 1.0

                marker_array.markers.append(arrow_msg)

                # LABEL MESSAGE #1
                label_msg_1 = Marker()
                label_msg_1.ns = "labels"
                label_msg_1.header.frame_id = "ur3e1_tool0"
                label_msg_1.id = 1
                label_msg_1.type = Marker.TEXT_VIEW_FACING # view-oriented text
                label_msg_1.scale.z = 0.10
                label_msg_1.action = Marker.ADD
                label_msg_1.text = self.to_frame_rel
                # label_msg_1.pose = self.pose_to
                label_msg_1.color.a = 1.0


                marker_array.markers.append(label_msg_1)

                # LABEL MESSAGE #2
                label_msg_2 = Marker()
                label_msg_2.ns = "labels"
                label_msg_2.header.frame_id = "camera_link"
                label_msg_2.id = 2
                label_msg_2.type = Marker.TEXT_VIEW_FACING # view-oriented text
                label_msg_2.scale.z = 0.10
                label_msg_2.action = Marker.ADD
                label_msg_2.text = self.from_frame_rel
                # label_msg_2.pose = self.pose_from
                label_msg_2.color.a = 1.0


                marker_array.markers.append(label_msg_2)
                # self.get_logger().info("Appended label 2")


                # arrow_label_msg = Marker()
                # arrow_label_msg.ns = "labels"
                # arrow_label_msg.header.frame_id = "table"
                # arrow_label_msg.id = 3
                # arrow_label_msg.type = Marker.TEXT_VIEW_FACING # view-oriented text
                # arrow_label_msg.scale.z = 1.0
                # arrow_label_msg.action = Marker.ADD
                # arrow_label_msg.text = str(arrow.transform)
                # arrow_label_msg.pose = arrow_msg.pose
                # arrow_label_msg.color.a = 1.0


                # marker_array.markers.append(arrow_label_msg)
                self.marker_pub.publish(marker_array)


            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}')
                return

def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    rclpy.spin(node)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    
    node.destroy_node()
    rclpy.shutdown()