from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import yaml
import os

def generate_launch_description():
    # Load YAML
    config_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'wave_rider_tf.yaml'
    )

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    nodes = []
    for tf in config['transforms']:
        xyz = [str(v) for v in tf['translation']]
        rpy = [str(v) for v in tf['rotation_rpy']]
        parent = tf['frame_id']
        child = tf['child_frame_id']

        args = xyz + rpy + [parent, child]

        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=args,
                name=f"{parent}_to_{child}_static_pub"
            )
        )

    return LaunchDescription(nodes)

