#!/usr/bin/env python3
"""Simple interactive CLI that publishes commands to /planner_command."""

import threading
import time

import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

PLANNER_NODE = "multi_waypoint_planner"


class PlannerCli(Node):
    def __init__(self) -> None:
        super().__init__("planner_cli")
        self.publisher = self.create_publisher(String, "/planner_command", 10)
        # Echo the planner's own log lines so command results are visible here.
        self.create_subscription(Log, "/rosout", self.on_log, 100)

    def on_log(self, msg: Log) -> None:
        if msg.name == PLANNER_NODE:
            print(f"\n[planner] {msg.msg}\nplanner> ", end="", flush=True)

    def has_subscriber(self) -> bool:
        return self.publisher.get_subscription_count() > 0

    def send(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    cli = PlannerCli()
    executor = SingleThreadedExecutor()
    executor.add_node(cli)
    def spin() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    deadline = time.monotonic() + 5.0
    while not cli.has_subscriber() and time.monotonic() < deadline:
        time.sleep(0.1)

    print("\nPlanner CLI connected to /planner_command")
    if not cli.has_subscriber():
        print(f"WARNING: {PLANNER_NODE} is not running — commands will be dropped.")
        print("         Start it with: ur5 planner   (or use: ur5 plan <name>)")
    print("Commands: plan, execute, return home, export joints, export cartesian, clear")
    print("Waypoints come from RViz 'Publish Point' clicks on the robot/scene geometry.")
    print("Type 'quit' to exit.\n")

    try:
        while rclpy.ok():
            command = input("planner> ").strip()
            if not command:
                continue
            if command.lower() in {"quit", "exit"}:
                break

            if not cli.has_subscriber():
                print(f"WARNING: no subscriber ({PLANNER_NODE} not running), command dropped.")
                continue
            cli.send(command)
            print(f"sent: {command}")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        # Stop the spin thread before destroying the node, otherwise rmw aborts on exit.
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)
        executor.shutdown()
        cli.destroy_node()


if __name__ == "__main__":
    main()
