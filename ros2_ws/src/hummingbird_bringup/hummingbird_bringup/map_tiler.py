# Generated / assisted by Claude
"""ROS 2 node: FAST-LIO /cloud_registered -> /hummingbird/map/tile (hummingbird_interfaces/MapTile).

The air -> ground map channel from docs/air-ground-cooperation.md. Instead of streaming the dense
cloud, it sends small height-map tiles (data_kind "height", see map_tiles.py for the layout):
changed tiles first, then a slow round-robin refresh of everything else so a ground node that
joined late or lost packets still ends up with the whole map.

With the defaults a tile is 64 x 64 cells of 0.25 m (16 m x 16 m, 8 kB) and at most 8 tiles are
sent per second. This node only publishes map data; it has no command path of any kind.
"""

from __future__ import annotations

from hummingbird_bringup._runtime import init_ros, spin_until_stopped
from hummingbird_bringup.map_recorder import _cloud_to_numpy
from hummingbird_bringup.map_tiles import HeightTiles


def main(args=None):
    from hummingbird_interfaces.msg import MapTile
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    init_ros(args)
    node = Node("hummingbird_map_tiler")
    topic = node.declare_parameter("cloud_topic", "/cloud_registered").value
    tile_topic = node.declare_parameter("tile_topic", "/hummingbird/map/tile").value
    period = float(node.declare_parameter("publish_period_s", 1.0).value)
    budget = int(node.declare_parameter("max_tiles_per_cycle", 8).value)
    refresh = bool(node.declare_parameter("refresh_unchanged_tiles", True).value)
    tiles = HeightTiles(
        resolution=float(node.declare_parameter("resolution", 0.25).value),
        tile_cells=int(node.declare_parameter("tile_cells", 64).value),
        origin_z=float(node.declare_parameter("origin_z", 0.0).value),
        # Height band that is kept, in the cloud frame. Indoors, set max_z below the ceiling.
        min_z=float(node.declare_parameter("min_z", -1000.0).value),
        max_z=float(node.declare_parameter("max_z", 1000.0).value),
        max_tiles=int(node.declare_parameter("max_tiles", 4096).value),
    )
    state = {"frame_id": "", "warned_full": False}

    # RELIABLE so that both reliable and best-effort ground subscribers match.
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=max(10, 2 * budget))
    pub = node.create_publisher(MapTile, tile_topic, qos)

    def on_cloud(msg):
        state["frame_id"] = msg.header.frame_id
        xyz, _ = _cloud_to_numpy(msg)
        tiles.add(xyz)
        if tiles.dropped_points and not state["warned_full"]:
            node.get_logger().warn(
                f"tile budget of {tiles.max_tiles} tiles is full: points outside the mapped area "
                "are no longer added to the shared height map"
            )
            state["warned_full"] = True

    def on_timer():
        if pub.get_subscription_count() == 0:
            return  # keep changed tiles queued until somebody listens
        stamp = node.get_clock().now().to_msg()
        for t in tiles.take(budget, refresh=refresh):
            m = MapTile()
            m.header.stamp = stamp
            m.header.frame_id = state["frame_id"]
            m.tile_id = t.tile_id
            m.frame_id = state["frame_id"]
            m.data_kind = "height"
            m.resolution = t.resolution
            m.width = t.cells
            m.height = t.cells
            m.origin_x, m.origin_y, m.origin_z = t.origin_x, t.origin_y, t.origin_z
            m.data = t.data.reshape(-1).tolist()
            pub.publish(m)

    node.create_subscription(PointCloud2, topic, on_cloud, 20)
    node.create_timer(period, on_timer)
    node.get_logger().info(
        f"map_tiler: {topic} -> {tile_topic} ({tiles.tile_cells}x{tiles.tile_cells} cells of "
        f"{tiles.resolution:g} m, <= {budget} tiles every {period:g} s)"
    )
    spin_until_stopped(node)


if __name__ == "__main__":
    main()
