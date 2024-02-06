import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    package = get_package_share_directory("warthog_description")
    xacro_path = os.path.join(package, "urdf/warthog.urdf.xacro")

    is_warthog = os.environ.get("IS_WARTHOG", None)
    remappings = []
    if is_warthog == '1':
        remappings=[('tf_static', 'dummy_tf_static'), ('tf', 'dummy_tf')]
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{"robot_description": ParameterValue(Command(["xacro ",xacro_path]), value_type=str),
                         "use_sim_time": True,
                        #  "dof": params["dof"],
                        #  "vision": params["vision"],
                        #  "sim": params["sim"]
                         }],
            remappings=remappings
        ),
    ])
