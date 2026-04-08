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

    wrench_observer = Node(
        package="palletrone_controller",
        executable="wrench_observer",
        name="wrench_observer",
        output="screen"
    )

    start_controllers_after_plant = RegisterEventHandler(
        OnProcessStart(
            target_action=plant,
            on_start=[wrench_controller, allocator_controller, wrench_observer]
        )
    )

    return LaunchDescription([
        bag_record,
        plant,
        start_controllers_after_plant
    ])
