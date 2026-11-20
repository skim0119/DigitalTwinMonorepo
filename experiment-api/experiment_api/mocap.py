"""Subscribe inside the Pi's ROS workspace and emit plain pose rows."""
import math
import time


def receive_poses(emit, stop, ready):
    import rclpy
    from motion_capture_tracking_interfaces.msg import NamedPoseArray
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    context = rclpy.Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("spirob_experiment_recorder", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)

    def receive(msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        for body in msg.poses:
            p, q = body.pose.position, body.pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if all(map(math.isfinite, values)) and sum(value**2 for value in values[3:]) > 0:
                emit(now, stamp, msg.header.frame_id, body.name, values)
                ready.set()

    node.create_subscription(NamedPoseArray, "/poses", receive, qos_profile_sensor_data)
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()
