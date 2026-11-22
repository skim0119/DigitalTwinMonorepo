"""Print the latest ROS 2 motion capture poses at 10 Hz."""

import argparse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--name", action="extend", nargs="+",
        help="Rigid body names to watch (default: all arm_* bodies)",
    )
    args = parser.parse_args()

    import rclpy
    from motion_capture_tracking_interfaces.msg import NamedPoseArray
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data

    latest = dict.fromkeys(args.name or ())

    def receive(msg):
        for body in msg.poses:
            if body.name in latest or (args.name is None and body.name.startswith("arm_")):
                latest[body.name] = body.pose

    def report():
        lines = []
        for name, pose in latest.items():
            if pose is None:
                lines.append(f"{name}: waiting for pose")
                continue
            p, q = pose.position, pose.orientation
            lines.append(
                f"{name}: position (x, y, z)=({p.x:.6f}, {p.y:.6f}, {p.z:.6f}) "
                f"quaternion (x, y, z, w)=({q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f})"
            )
        print("\n".join(lines) or "Waiting for arm_* poses", flush=True)

    rclpy.init()
    node = rclpy.create_node("watch_posture")
    try:
        node.create_subscription(NamedPoseArray, "/poses", receive, qos_profile_sensor_data)
        node.create_timer(0.1, report)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
