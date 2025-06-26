from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, \
                           GroupAction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace

from gesture_pointer.constants import MARKERS, CAMERA_TF_FRAME, \
                                      CORNER_MARKER_SIZE

def get_marker_node(marker_id, marker_size, camera_reference_frame): 
    """
    Get marker GroupcAtion for LaunchDescription 

    Args:
        marker_id (str): ArUco marker ID 
        marker_size (str): The size of the marker (m)
        camera_reference_frame (str): Name of the RGB stream reference frame

    Returns:
        launch.actions.GroupAction: Wrapper for a single marker detection node, 
                                    with a dedicated namespace 
    """
    aruco_single_params = {
        'marker_id' : marker_id,
        'marker_size' : marker_size,
        'reference_frame': camera_reference_frame,
        'marker_frame' : 'aruco_' + str(marker_id)
    }
    launch_dir = PathJoinSubstitution([FindPackageShare('gesture_pointer'),
                                       'launch'])

    return GroupAction(
                actions=[
                    PushRosNamespace('aruco_' + str(marker_id)),
                    IncludeLaunchDescription(
                        PathJoinSubstitution([
                            launch_dir,
                            'single_marker_launch.py'
                        ]),
                        launch_arguments=aruco_single_params.items()
                    )        
                ]
            )    

def generate_launch_description():
    """
    Launch description for detecting multiple markers and collecting the 
    poses with aruco_corner_collection node 

    Returns:
        LaunchDescription 
    """
    
    corner_marker_size = DeclareLaunchArgument(
        'corner_marker_size', default_value=str(CORNER_MARKER_SIZE),  
        description='Marker size in m, '
    )

    camera_reference_frame = DeclareLaunchArgument(
        'camera_reference_frame', default_value=CAMERA_TF_FRAME,
        description='Camera reference frame'
    )
    
    #ld.add_action(corner_marker_size)
    ld = LaunchDescription()
    ld.add_action(corner_marker_size)
    ld.add_action(camera_reference_frame)
    
    for id in MARKERS: 
        ld.add_action(get_marker_node(str(id), 
                                LaunchConfiguration('corner_marker_size'), 
                                LaunchConfiguration('camera_reference_frame')))
    
    ld.add_action(
        Node(
            package='gesture_pointer',
            executable='aruco_pose_collector',
            arguments=["--number_of_cycles", "1"],
            on_exit=Shutdown()
        )
    )

    return ld


