# Generated / assisted by Claude
"""Signal-safe start / spin / stop shared by the nodes of this package.

Why not ``rclpy.spin`` + ``except KeyboardInterrupt``? Ctrl-C under ``ros2 launch`` reaches every
node twice (once from the terminal, once forwarded by launch). The second SIGINT then raises
KeyboardInterrupt in the middle of the clean-up, which is how a map would be lost on shutdown.
Here signals only set a flag: the spin loop ends, the clean-up runs to completion, and repeated
signals are harmless.
"""

from __future__ import annotations

import signal


def init_ros(args=None) -> None:
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)


def spin_until_stopped(node, before_shutdown=None) -> None:
    """Spin ``node`` until SIGINT / SIGTERM, then run ``before_shutdown`` and shut ROS down."""
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

    stop = {"requested": False}

    def _on_signal(_signum, _frame):
        stop["requested"] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop["requested"]:
            executor.spin_once(timeout_sec=0.2)
    except ExternalShutdownException:
        pass
    finally:
        try:
            if before_shutdown is not None:
                before_shutdown()
        finally:
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
