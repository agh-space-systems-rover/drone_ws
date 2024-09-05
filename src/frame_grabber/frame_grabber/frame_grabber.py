import rclpy
from rclpy.node import Node
from example_interfaces.msg import Empty
import os

class FrameGrabber(Node):
    def __init__(self):
        super().__init__("frame_grabber")

        self.subscription = self.create_subscription( Empty, '/trigger_shot', self.listener_callback, 10)

    def listener_callback(self, data: Empty):
        self.get_logger().info('Receiving video frame grab request')
        os.system("rpicam-still -n --camera 0 -o ~/Pictures/$(date +%H_%M_%S__%d_%m_%Y).jpg")



def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    image_subscriber = FrameGrabber()
    
    rclpy.spin(image_subscriber)
    
    # Destroy the node explicitly
    image_subscriber.destroy_node()
    
    rclpy.shutdown()
        

if __name__ == '__main__':
    main()
