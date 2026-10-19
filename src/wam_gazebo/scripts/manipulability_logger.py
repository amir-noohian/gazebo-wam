#!/usr/bin/env python3

import csv
from pathlib import Path
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


JOINT_COLUMNS = [f"q{i}" for i in range(1, 8)]
DESIRED_COLUMNS = [f"q_des{i}" for i in range(1, 8)]
NULL_COLUMNS = [f"null_direction{i}" for i in range(1, 8)]
CSV_COLUMNS = [
    "time",
    "phase",
    "phase_velocity",
    "position_error_norm",
    "orientation_error_norm",
    "translational_manipulability",
    "rotational_manipulability",
    "wrist_manipulability",
    "whole_manipulability_raw",
    "whole_manipulability_scaled",
    "sigma_min_translation",
    "condition_translation",
    "sigma_min_rotation",
    "condition_rotation",
    "sigma_min_full",
    "condition_full",
] + JOINT_COLUMNS + DESIRED_COLUMNS + NULL_COLUMNS


class ManipulabilityLogger(Node):

    def __init__(self):
        super().__init__("wam_manipulability_logger")
        self.declare_parameter(
            "topic", "/wam_pose_ns_controller/manipulability_metrics"
        )
        self.declare_parameter(
            "output_directory", str(Path.cwd() / "data")
        )

        topic = self.get_parameter("topic").value
        output_directory = Path(
            self.get_parameter("output_directory").value
        ).expanduser().resolve()
        output_directory.mkdir(parents=True, exist_ok=True)

        used_numbers = []
        for path in output_directory.iterdir():
            match = re.fullmatch(r"manipulability_(\d+)\.(csv|png)", path.name)
            if match:
                used_numbers.append(int(match.group(1)))
        experiment_number = max(used_numbers, default=0) + 1
        self.output_path = (
            output_directory / f"manipulability_{experiment_number}.csv"
        )
        self.output_file = self.output_path.open("w", newline="")
        self.writer = csv.writer(self.output_file)
        self.writer.writerow(CSV_COLUMNS)
        self.output_file.flush()

        self.subscription = self.create_subscription(
            Float64MultiArray, topic, self.metrics_callback, 20
        )
        self.get_logger().info(f"Recording experiment data to {self.output_path}")

    def metrics_callback(self, message):
        if len(message.data) != len(CSV_COLUMNS):
            self.get_logger().error(
                f"Expected {len(CSV_COLUMNS)} metrics, received {len(message.data)}"
            )
            return
        self.writer.writerow(message.data)
        self.output_file.flush()

    def destroy_node(self):
        if not self.output_file.closed:
            self.output_file.flush()
            self.output_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManipulabilityLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
