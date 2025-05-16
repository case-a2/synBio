import rclpy
from rclpy.node import Node
import time

from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates


class GripperGpio(Node):
    
        def __init__(self):
            super().__init__('gripper_gpio')
            
            # SetIO Client
            self.client = self.create_client(SetIO, '/io_and_status_controller/set_io')
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

            # IOStates subscriber
            self.subscriber = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.callback, 10)
            
        def send_gpio(self):
            self.future = self.client.call_async(self.req)
            self.get_logger().info('sending GPIO signal...')

        def close_gripper(self):
            self.req = SetIO.Request()
            self.req.fun = 1
            self.req.pin = 8
            self.req.state = 1.0
            self.future = None

        def open_gripper(self):
            self.req = SetIO.Request()
            self.req.fun = 1
            self.req.pin = 8
            self.req.state = 0.0
            self.future = None


        # Set the gripper to open for 2 seconds then close
        def callback(self, msg):
            self._logger.info('Received IOStates message')
            if msg.digital_out_states[8].state == 0.0:
                self.close_gripper()
                self.send_gpio()
                time.sleep(2)
            else:
                self.open_gripper()
                self.send_gpio()
                time.sleep(2)

def main(args=None):
    rclpy.init(args=args)

    gpio = GripperGpio()

    rclpy.spin(gpio)

    gpio.destroy_node()
    rclpy.shutdown()