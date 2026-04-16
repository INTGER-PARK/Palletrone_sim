from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    
    plant = Node(
        package="plant",
        executable="plant",  
        name="plant",
        output="screen"
    )

    wrench_controller = Node(
        package="palletrone_controller",
        executable="wrench_controller",
        name="wrench_controller",
        output="screen"
    )

    allocator_controller = Node(
        package="palletrone_controller",
        executable="allocator_controller",
        name="allocator_controller",
        output="screen"
    )

    ekf_state_estimator = Node(
        package="palletrone_controller",
        executable="ekf_state_estimator",
        name="ekf_state_estimator",
        output="screen"
    )

    position_cmd = Node(
        package="palletrone_cmd",
        executable="position_cmd",
        name="position_cmd",
        output="screen"
    )

    start_controllers_after_plant = RegisterEventHandler(
        OnProcessStart(
            target_action=plant,
            # MoB/wrench_observer is intentionally not launched here.
            # Start it manually when needed:
            #   ros2 run palletrone_controller wrench_observer
            # It publishes /external_wrench_hat for monitoring. Enable compensation explicitly
            # on wrench_controller after tuning.
            on_start=[wrench_controller, allocator_controller, ekf_state_estimator]
        )
    )

    start_cmd_after_allocator = RegisterEventHandler(
        OnProcessStart(
            target_action=allocator_controller,
            on_start=[position_cmd]
        )
    )

    return LaunchDescription([
        plant,
        start_controllers_after_plant,
        start_cmd_after_allocator
    ])
