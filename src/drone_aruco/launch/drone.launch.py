from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_path


def generate_launch_description():
    
    parameters = [
        str(get_package_share_path("drone_aruco") / "config" / f"aruco_tracker.yaml")
    ]

    aruco_tracker = Node(
                package="aruco_opencv",
                executable="aruco_tracker_autostart",
                name="aruco_tracker",
                parameters=parameters,
            )
    camera_node = Node(
                package="camera_ros",
                executable="camera_node",
                name="CAMERA_NODE",
                parameters=[
                    {"camera": 0},
                    {"format": "YUYV"},
                    {"height": 864},
                    {"width": 1536},
                    {"ExposureTime": 1000},
                ]
                
    )
    
            
    return LaunchDescription([
        camera_node,
        aruco_tracker
    ]
    )
