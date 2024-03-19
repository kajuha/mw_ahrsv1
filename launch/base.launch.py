from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

from launch.actions import OpaqueFunction

def evaluate_node(context, *args, **kwargs):
    outputs = LaunchConfiguration('message_display').perform(context)
    node = Node(
        package = 'mw_ahrsv1',
        executable = 'mw_ahrsv1',
        name = 'mw_ahrsv1',
        output = {
            'stdout': outputs,
            'stderr': outputs,
        },
        emulate_tty = True,
        parameters = [{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'message_display': LaunchConfiguration('message_display'),
            'frame_id': LaunchConfiguration('frame_id'),
            'parent_frame_id': LaunchConfiguration('parent_frame_id'),
            'tx_once': LaunchConfiguration('tx_once'),
            'node_name': LaunchConfiguration('node_name'),
        }]
    )

    return [node]

def generate_launch_description():
    serial_port_launch_arg = DeclareLaunchArgument(
        'serial_port',
        default_value = TextSubstitution(text='/dev/recipe.driver.ahrs'),
        description = 'serial_port'
    )
    baud_rate_launch_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value = TextSubstitution(text='115200'),
        description = 'baud_rate'
    )
    message_display_launch_arg = DeclareLaunchArgument(
        'message_display',
        default_value = TextSubstitution(text='screen'),
        description = 'message_display'
    )
    frame_id_launch_arg = DeclareLaunchArgument(
        'frame_id',
        default_value = TextSubstitution(text='imu_link'),
        description = 'frame_id'
    )
    parent_frame_id_launch_arg = DeclareLaunchArgument(
        'parent_frame_id',
        default_value = TextSubstitution(text='base_link'),
        description = 'parent_frame_id'
    )
    tx_once_launch_arg = DeclareLaunchArgument(
        'tx_once',
        default_value = TextSubstitution(text='0'),
        description = 'tx_once'
    )
    node_name_launch_arg = DeclareLaunchArgument(
        'node_name',
        default_value = TextSubstitution(text='mw_ahrsv1'),
        description = 'node_name'
    )

    return LaunchDescription([
        serial_port_launch_arg,
        baud_rate_launch_arg,
        message_display_launch_arg,
        frame_id_launch_arg,
        parent_frame_id_launch_arg,
        tx_once_launch_arg,
        node_name_launch_arg,
        OpaqueFunction(function=evaluate_node),
    ])