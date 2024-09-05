import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOLLocal


class MavlinkTest(Node):

    def __init__(self):
        super().__init__('test_mavlink')
        self._state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)
        
        self.is_guided = False
        
        self._arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._takeoff_client = self.create_client(CommandTOLLocal, '/mavros/cmd/takeoff_local')
        self._land_client = self.create_client(CommandTOLLocal, '/mavros/cmd/land_local')
        

    
    def arm_drone(self):
        while not self._arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
            
        armReq = CommandBool.Request()
        armReq.value = True
        self.future = self._arming_client.call_async(armReq)
        self.get_logger().info("sent arm request")
        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()
    
    def takeoff_drone(self, height: float):
        while not self._takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
            
        takeoffReq = CommandTOLLocal.Request()
        takeoffReq.rate = 0.2
        takeoffReq.position.z = height
        self.future_takeoff = self._takeoff_client.call_async(takeoffReq)
        self.get_logger().info("sent takeoff request")
        rclpy.spin_until_future_complete(self, self.future_takeoff)
        return self.future_takeoff.result()
    
    
    def land_drone(self):
        while not self._land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
            
        landReq = CommandTOLLocal.Request()
        landReq.rate = 0.1
        # takeoffReq.position.z = height
        self.future_land = self._land_client.call_async(landReq)
        rclpy.spin_until_future_complete(self, self.future_land)
        return self.future_land.result()
        
        
        
        
    def state_callback(self, msg: State):
        if msg.guided and not self.is_guided:
            self.is_guided = True
            self.get_logger().info('Drone is in guided')
            
            self.armed = self.arm_drone()
            # if self.armed.success:
            self.get_logger().info('Armed successfully')
            # else:
            #     self.get_logger().error('Armed unsuccessfully')
            #     return
            time.sleep(5)
            self.takeoff = self.takeoff_drone(1.0)
            if self.takeoff.success:
                self.get_logger().info('Takeoff successfully')
            else:
                self.get_logger().error('Takeoff unsuccessfully')
                return
            
            self.land = self.land_drone()
            if self.land.success:
                self.get_logger().info('Landed successfully')
            else:
                self.get_logger().error('Landed unsuccessfully')
                return
            
        elif not msg.guided:
            self.is_guided = False
    

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MavlinkTest()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
