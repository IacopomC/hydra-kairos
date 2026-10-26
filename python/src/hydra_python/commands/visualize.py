"""Entry point to run open3d visualizer."""

import subprocess

import click
from spark_dsg.open3d_visualization import RemoteVisualizer


@click.command(name="visualize")
@click.argument("ros_arguments", nargs=-1)
@click.option("--rviz/--no-rviz", default=False, help="use rviz for visualization")
@click.option("--ip", default="127.0.0.1", help="IP to listen on")
@click.option("--port", default="8001", help="port to listen on")
def cli(ros_arguments, rviz, ip, port):
    """Run the Hydra visualizer with optional [ROS_ARGUMENTS...] using zmq."""
    if not rviz:
        visualizer = RemoteVisualizer(num_dynamic_to_skip=1)
        visualizer.run()
    else:
        command = [
            "ros2",
            "launch",
            "hydra_visualizer",
            "streaming_visualizer.launch.yaml",
            "use_zmq:=true",
            f"zmq_url:=tcp://{ip}:{port}",
            *ros_arguments,
        ]
        subprocess.run(command)
