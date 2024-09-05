from launch_ros.actions import Node
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.time
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
from tf_transformations import euler_from_quaternion


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
        self.marker_publisher = self.create_publisher(MarkerPose, "marker_cords",10)


    def marker_pose_callback(self, msg: ArucoDetection):
        marker_cords = MarkerPose()
        markers:  list[MarkerPose] = msg.markers

        for marker in markers:
            marker_cords.marker_id = TAG_DICT[marker.marker_id]
            marker_cords.pose.position.x = marker.pose.position.x
            marker_cords.pose.position.y = marker.pose.position.y

            self.marker_publisher.publish(marker_cords)

    def filter_markers(self, msg: MarkerPose):
        orientation = [
            msg.pose.orientation.x, 
            msg.pose.orientation.y, 
            msg.pose.orientation.z, 
            msg.pose.orientation.w
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation)

        self.pose_roll = roll
        self.pose_pitch = pitch
        self.pose_yaw = yaw

        print(roll, pitch, yaw)
        
        


            

def main():
    try:
        rclpy.init()
        node = cord_publisher()
        rclpy.spin(node)
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
          
        

    
