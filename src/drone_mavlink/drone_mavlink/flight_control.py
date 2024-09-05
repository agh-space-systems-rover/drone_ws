import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String
from mavros_msgs.msg import State, CommandCode, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOLLocal, CommandLong
from drone_interfaces.msg import MarkerCords
from example_interfaces.msg import Empty

from enum import Enum
from collections import namedtuple as NamedTuple

class FlightState(Enum):
    WAIT = 0
    ARM = 1
    WAIT_FOR_SECS = 2
    TAKEOFF = 3
    OFFSET_WAYPOINT = 4
    LOOK_FOR_TAG = 5
    CENTER_ON_TAG = 6
    TAKE_PHOTO = 7
    LAND = 8
    
Waypoint = NamedTuple('Waypoint', ['x', 'y', 'z'])
FlightTarget = NamedTuple('FlightTarget', ['waypoint', 'tag', "next"])

TARGETS = [
    FlightTarget(Waypoint(6.5, 6.5, 0), "A", FlightState.TAKE_PHOTO),
    FlightTarget(Waypoint(0, -6.5, 0), "B", FlightState.LAND),
    FlightTarget(Waypoint(-6.5, 6.5, 0), "C", FlightState.TAKE_PHOTO),
    FlightTarget(Waypoint(0, -6.5, 0), "D", FlightState.LAND),
]

CENTERED_THRESHOLD = 0.4
CENTERED_COUNT = 3

PHOTOS_TO_TAKE = 5

class FlightControl(Node):

    def __init__(self):
        super().__init__('flight_control')
        self._state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)
        
        self._last_mode = State.MODE_APM_COPTER_STABILIZE
        self._last_armed = False
        self._previous_flight_state = FlightState.WAIT
        
        self._waiting_done = False
        self._wait_count = 0
        self._wait_secs = 0
        
        self._offset_waypoint = Waypoint(0, 0, 0)
        
        self._searched_tag = ""
        
        self._target_id = 0
        
        self._entry_executed = False
        self._after_wait_state = FlightState.TAKEOFF
        
        self._flight_state = FlightState.WAIT
        
        self._just_guided = None
        
        self._hb_counter = 0
        
        self._tag_msg = None
        self._tag_found = False
        self._tag_center_count = 0
        self._tag_centered = False
        
        self._taken_photos = 0
        self._photo_taken = False
        
        self._arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._cmd_long_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        self._tags_sub = self.create_subscription(MarkerCords, '/marker_cords', self.tags_callback, 10)
        
        self._setpoint_raw_local_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        
        self._take_photo_pub = self.create_publisher(Empty, '/trigger_shot', 10)
        
    def tags_callback(self, msg: MarkerCords):
        self._tag_msg = msg
        if self._searched_tag == msg.marker_id:
            self._tag_found = True
        
    
    def arm_drone(self):
        while not self._arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
            
        armReq = CommandBool.Request()
        armReq.value = True
        self.future = self._arming_client.call_async(armReq)
        self.get_logger().info("sent arm request")
        
    def wait_secs(self):
        self.get_logger().info(f"waiting {self._wait_count}/{self._wait_secs}")
        if self._wait_count < self._wait_secs:
            self._wait_count += 1
            return
        self._waiting_done = True
        
    
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
        self.get_logger().info(f"sent go to offset request {x}, {y}, {z}")
    
    def land_drone(self):
        while not self._cmd_long_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('land service not available, waiting again...')
            
        cmdLong = CommandLong.Request()
        cmdLong.command = CommandCode.NAV_LAND
        
        self.future_land = self._cmd_long_client.call_async(cmdLong)
        self.get_logger().info("sent land request")
        
    def center_tag(self):
        if self._tag_msg is None:
            return
        if self._tag_found:
            if abs(marker.x_pos) < CENTERED_THRESHOLD and abs(marker.y_pos) < CENTERED_THRESHOLD:
                self._tag_center_count += 1
            else:
                self._tag_center_count = 0
            if self._tag_center_count >= CENTERED_COUNT:
                self._tag_centered = True
            self.get_logger().info(f"centering tag {marker.x_pos}, {marker.y_pos}")
            self.go_to_offset(marker.x_pos, marker.y_pos, 0)
            self._tag_found = False
            
    def take_photo(self):
        self._take_photo_pub.publish(Empty())
        taken_photos += 1
        self.get_logger().info("sent take photo request")        
        
    def state_callback(self, msg: State):
        if msg.mode != State.MODE_APM_COPTER_GUIDED:
            return
        check_conditions(msg)
        change_state()
        execute_state()
        
    def change_flight_state_to(self, state: FlightState):
        self._previous_flight_state = self._flight_state
        self._flight_state = state
        self._entry_executed = False
        self.get_logger().info(f"changed state to {state}")
        
    def setup_wait(self, secs: int, after_wait_state: FlightState):
        self._wait_secs = secs
        self._after_wait_state = after_wait_state
        self._wait_count = 0
        self._waiting_done = False        
        
    def check_conditions(self, msg: State):
        self._just_guided = self._last_mode != State.MODE_APM_COPTER_GUIDED and msg.mode == State.MODE_APM_COPTER_GUIDED
        self._armed = msg.armed
        self._landed = not msg.armed and self._last_armed
        
        self._last_armed = msg.armed
        self._last_mode = msg.mode
        
        self._photo_taken = self._taken_photos >= PHOTOS_TO_TAKE and self._flight_state == FlightState.TAKE_PHOTO
    
    def change_state(self):
        if self._flight_state == FlightState.WAIT and self._just_guided:
            self.change_flight_state_to(FlightState.ARM)
        elif self._flight_state == FlightState.ARM and self._armed:
            self.setup_wait(5, FlightState.TAKEOFF)            
            self.change_flight_state_to(FlightState.WAIT_FOR_SECS)
        elif self._flight_state == FlightState.WAIT_FOR_SECS and self._waiting_done:
            self.change_flight_state_to(self._after_wait_state)
        elif self._flight_state == FlightState.TAKEOFF:
            self.setup_wait(3, FlightState.OFFSET_WAYPOINT)
            self._offset_waypoint = TARGETS[self._target_id].waypoint
            self.change_flight_state_to(FlightState.WAIT_FOR_SECS)
        elif self._flight_state == FlightState.OFFSET_WAYPOINT:
            self._searched_tag = TARGETS[self._target_id].tag
            self._tag_found = False
            self.change_flight_state_to(FlightState.LOOK_FOR_TAG)
        elif self._flight_state == FlightState.LOOK_FOR_TAG and self._tag_found:
            self._tag_found = False
            self.change_flight_state_to(FlightState.CENTER_ON_TAG)
        elif self._flight_state == FlightState.CENTER_ON_TAG and self._tag_centered:
            self.change_flight_state_to(TARGETS[self._target_id].next)
            self._target_id += 1
        elif self._flight_state == FlightState.TAKE_PHOTO and self._photo_taken:
            self.change_flight_state_to(FlightState.OFFSET_WAYPOINT)
        elif self._flight_state == FlightState.LAND and self._landed:
            self.setup_wait(5, FlightState.ARM)
            self.change_flight_state_to(FlightState.WAIT_FOR_SECS)
        
        
    def execute_state(self):
        if self._flight_state == FlightState.WAIT:
            pass
        elif self._flight_state == FlightState.ARM:
            if not self._entry_executed:
                self.arm_drone()
                self._entry_executed = True
        elif self._flight_state == FlightState.WAIT_FOR_SECS:
            self.wait_secs()
        elif self._flight_state == FlightState.TAKEOFF:
            if not self._entry_executed:
                self.takeoff_drone(2)
                self._entry_executed = True
        elif self._flight_state == FlightState.OFFSET_WAYPOINT:
            if not self._entry_executed:
                self.go_to_offset(_offset_waypoint)
                self._entry_executed = True
        elif self._flight_state == FlightState.LOOK_FOR_TAG:
            pass
        elif self._flight_state == FlightState.CENTER_ON_TAG:
            self.center_tag()
        elif self._flight_state == FlightState.TAKE_PHOTO:
            if not self._entry_executed:
                self._photo_taken = False
                self._taken_photos = 0
                self._entry_executed = True
            self.take_photo()
        elif self._flight_state == FlightState.LAND:
            if not self._entry_executed:
                self.land_drone()
                self._entry_executed = True
        
def main(args=None):
    rclpy.init(args=args)

    flight_control = FlightControl()

    rclpy.spin(flight_control)
    
    flight_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
