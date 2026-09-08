#!/usr/bin/env python3
"""Simple interactive CLI that publishes commands to /planner_command."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PlannerCli(Node):
    def __init__(self) -> None:
        super().__init__("planner_cli")
        self.publisher = self.create_publisher(String, "/planner_command", 10)

    def send(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    cli = PlannerCli()

    print("\nPlanner CLI connected to /planner_command")
    print("Commands: plan, execute, return home, export joints, export cartesian, clear")
    print("Type 'quit' to exit.\n")

    try:
        while rclpy.ok():
            command = input("planner> ").strip()
            if not command:
                continue
            if command.lower() in {"quit", "exit"}:
                break

            cli.send(command)
            rclpy.spin_once(cli, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
