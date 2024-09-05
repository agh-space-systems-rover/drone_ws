import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String
from mavros_msgs.msg import State, CommandCode, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOLLocal, CommandLong

from enum import Enum

class FlightState(Enum):
    WAITING = 0
    ARMING = 1
    TAKEOFF = 2
    GO_TO_A = 3
    TAKE_PHOTO = 4
    GO_TO_B = 5
    LANDING_ON_B = 6
    LANDED_ON_B = 7
    TAKEOFF_FROM_B = 8
    GO_TO_C = 9
    TAKE_PHOTO_C = 10
    GO_TO_START = 11
    LANDING_ON_START = 12


class MavlinkTest(Node):

    def __init__(self):
        super().__init__('test_mavlink')
        self._state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)
        
        self._last_mode = State.MODE_APM_COPTER_STABILIZE
        self._last_armed = False
        
        self._flight_state = FlightState.WAITING
        
        self.is_guided = False
        
        self._hb_counter = 0
        
        self._arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._takeoff_client = self.create_client(CommandTOLLocal, '/mavros/cmd/takeoff_local')
        self._land_client = self.create_client(CommandTOLLocal, '/mavros/cmd/land_local')
        self._cmd_long_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        self._setpoint_raw_local_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
    
    def arm_drone(self):
        while not self._arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
            
        armReq = CommandBool.Request()
        armReq.value = True
        self.future = self._arming_client.call_async(armReq)
        self.get_logger().info("sent arm request")
    
    def takeoff_drone(self, height: float):
        while not self._cmd_long_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('takeoff service not available, waiting again...')
            
        cmdLong = CommandLong.Request()
        cmdLong.command = CommandCode.NAV_TAKEOFF
        cmdLong.param7 = height
        
        self.future_takeoff = self._cmd_long_client.call_async(cmdLong)
        self.get_logger().info("sent takeoff request")
        
    def go_to_offset(self, x: float, y: float, z: float):
        pos_target = PositionTarget()
        pos_target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        
        pos_target.position.x = x
        pos_target.position.y = y
        pos_target.position.z = z   
        
        self._setpoint_raw_local_pub.publish(pos_target)
    
    def land_drone(self):
        while not self._cmd_long_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('takeoff service not available, waiting again...')
            
        cmdLong = CommandLong.Request()
        cmdLong.command = CommandCode.NAV_LAND
        
        self.future_takeoff = self._cmd_long_client.call_async(cmdLong)
        self.get_logger().info("sent takeoff request")
        
        
    def state_callback(self, msg: State):
        self._hb_counter += 1
        if msg.mode != State.MODE_APM_COPTER_GUIDED or self._hb_counter < 10:
            return
        self._hb_counter = 0
        if self._last_mode != State.MODE_APM_COPTER_GUIDED and msg.mode == State.MODE_APM_COPTER_GUIDED and self._flight_state == FlightState.WAITING:
            self.get_logger().info('Drone is in guided!')
            self._flight_state = FlightState.ARMING
            self.arm_drone()
        elif self._flight_state == FlightState.ARMING and msg.armed == True and self._last_armed == False:
            self.get_logger().info('Drone is armed!')
            self._flight_state = FlightState.TAKEOFF
            self.takeoff_drone(1.0)
        elif self._flight_state == FlightState.TAKEOFF:
            self.get_logger().info(f'Drone is taking off! {msg.mode}')
            self.go_to_offset(5.0, 5.0, 0.0)
            self._flight_state = FlightState.GO_TO_A
        elif self._flight_state == FlightState.GO_TO_A:
            self.get_logger().info(f'Drone is going to A! {msg.mode}')
            self._flight_state = FlightState.TAKE_PHOTO
        elif self._flight_state == FlightState.TAKE_PHOTO:
            self.get_logger().info(f'Drone is taking photo! {msg.mode}')
            self._flight_state = FlightState.GO_TO_B
            self.go_to_offset(-5.0, 0.0, 0.0)
        elif self._flight_state == FlightState.GO_TO_B:
            self.get_logger().info(f'Drone is going to B! {msg.mode}')
            self._flight_state = FlightState.LANDING_ON_B
            self.land_drone()
            self.get_logger().info(f'Drone is landing on B! {msg.mode}')
        elif self._flight_state == FlightState.LANDING_ON_B and msg.armed == False and self._last_armed == True:
            self._flight_state = FlightState.LANDED_ON_B
            self.get_logger().info(f'Drone has landed on B! {msg.mode}')
        elif self._flight_state == FlightState.LANDED_ON_B:
            self.arm_drone()
            self._flight_state = FlightState.TAKEOFF_FROM_B
        elif self._flight_state == FlightState.TAKEOFF_FROM_B and msg.armed == True and self._last_armed == False:
            self.takeoff_drone(1.0)
            self._flight_state = FlightState.GO_TO_C
            self.get_logger().info(f'Drone is taking off from B! {msg.mode}')
        elif self._flight_state == FlightState.GO_TO_C:
            self.get_logger().info(f'Drone is going to C! {msg.mode}')
            self._flight_state = FlightState.TAKE_PHOTO_C
            self.go_to_offset(5.0, -5.0, 0.0)
        elif self._flight_state == FlightState.TAKE_PHOTO_C:
            self.get_logger().info(f'Drone is taking photo C! {msg.mode}')
            self._flight_state = FlightState.GO_TO_START
            self.go_to_offset(-5.0, 0.0, 0.0)
        elif self._flight_state == FlightState.GO_TO_START:
            self.get_logger().info(f'Drone is going to start! {msg.mode}')
            self._flight_state = FlightState.LANDING_ON_START
        elif self._flight_state == FlightState.LANDING_ON_START:
            self.land_drone()
            self.get_logger().info(f'Drone is landing on start! {msg.mode}')  
            
        self._last_mode = msg.mode
        self._last_armed = msg.armed          


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
