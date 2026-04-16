from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from datetime import datetime
from pathlib import Path

def generate_launch_description():
    workspace_dir = Path("/home/parkjeongsu/ros2_project/PPP_sim")
    bag_dir = workspace_dir / "bags"
    bag_dir.mkdir(parents=True, exist_ok=True)
    bag_name = str(bag_dir / f"bag_all_{datetime.now().strftime('%Y%m%d_%H%M%S')}")

    bag_record = ExecuteProcess(
        cmd=[
            "/opt/ros/humble/bin/ros2",
            "bag",
            "record",
            "-a",
            "-o", bag_name
        ],
        output="screen"
    )

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

    return LaunchDescription([
        bag_record,
        plant,
        start_controllers_after_plant
    ])
