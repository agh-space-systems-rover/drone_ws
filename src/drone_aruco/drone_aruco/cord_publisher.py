from launch_ros.actions import Node
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.time
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
from tf_transformations import euler_from_quaternion, quaternion_matrix
from drone_interfaces.msg import MarkerCords


TAG_DICT = {
    101: "A",
    102: "B",
    103: "C",
    104: "D",
}

class cord_publisher(Node):
    def __init__(self):
        super().__init__("cord_publisher")

        self.marker_subscription = self.create_subscription(ArucoDetection, "/aruco_detections", self.marker_pose_callback, 10)
        self.marker_publisher = self.create_publisher(MarkerCords, "/marker_cords",10)


    def marker_pose_callback(self, msg: ArucoDetection):
        marker_cords = MarkerCords()
        markers:  list[MarkerPose] = msg.markers

        for marker in markers:
            if marker.marker_id not in TAG_DICT:
                continue
            marker_cords.marker_id = TAG_DICT[marker.marker_id]
            
            marker_cords.x_pos = marker.pose.position.x
            marker_cords.y_pos = marker.pose.position.y

            self.marker_publisher.publish(marker_cords)
            

def main():
    try:
        rclpy.init()
        node = cord_publisher()
        rclpy.spin(node)
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
          
        

    
