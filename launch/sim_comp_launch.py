import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='rt2_assign1_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
        ComposableNode(
            package="rt2_assignment1",
            plugin="rt2_assignment1::RandomPositionServer",
            name='position_service'),

    ComposableNode(
        package="rt2_assignment1",
        plugin="rt2_assignment1::StateMachine",
        name="comp_FSM"
        )
    ],
        output='screen',
    )
    return launch.LaunchDescription([container])