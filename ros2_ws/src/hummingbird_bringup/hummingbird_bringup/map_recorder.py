# Generated / assisted by Grok
# Reviewed and extended by Claude (autosave, save on shutdown, atomic writes)
"""ROS 2 node: accumulate FAST-LIO /cloud_registered into a voxel map and save it as PCD.

Why not only FAST-LIO's own /map_save? In FAST_LIO_ROS2 (ros2 @ 2fffc57) /map_save writes
``pcl_wait_pub``, which only grows when ``publish.map_en`` is true (1 Hz timer, unbounded,
re-published every second), and the on-shutdown ``scans.pcd`` path is commented out. This node
keeps a bounded voxel map instead. Both can be used; see scripts/save_map.sh.

Service: /hummingbird/save_map (std_srvs/Trigger) -> writes <output_dir>/hummingbird_map_<time>.pcd

The map only lives in RAM, so it is also written without being asked:
  * every ``autosave_period_s`` to <output_dir>/hummingbird_map_autosave.pcd (one rolling file,
    replaced atomically), so a crash or an empty battery costs at most one period of data;
  * once more on shutdown (Ctrl-C / launch stop) to a time-stamped file, if anything changed
    since the last save.

Memory: about 50 bytes per voxel, plus a short peak of roughly 3x while merging. The default
budget of 10 million voxels is ~0.5 GB steady; lower ``max_points`` on a loaded Jetson.
"""

from __future__ import annotations

import os
import time

import numpy as np

from hummingbird_bringup import pcd_io
from hummingbird_bringup._runtime import init_ros, spin_until_stopped


_OFF = 1 << 20  # 21 bits per axis -> +-2^20 voxels (+-52 km at 5 cm)

AUTOSAVE_NAME = "hummingbird_map_autosave.pcd"


def _pack(keys: np.ndarray) -> np.ndarray:
    k = (keys + _OFF).astype(np.int64)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


class VoxelMap:
    """One centroid per voxel, fully vectorised (np.unique on packed keys). Testable without ROS."""

    def __init__(self, voxel: float, max_points: int = 10_000_000, consolidate_every: int = 2_000_000):
        if voxel <= 0:
            raise ValueError("voxel must be > 0")
        self.voxel = float(voxel)
        self.max_points = int(max_points)
        self.consolidate_every = int(consolidate_every)
        self._k = np.zeros(0, np.int64)
        self._s = np.zeros((0, 3))
        self._i = np.zeros(0)
        self._c = np.zeros(0)
        self._pending = []
        self._pending_n = 0
        # Bumped by every add() that contributed points; lets a caller skip redundant saves.
        self.revision = 0

    def __len__(self):
        self._consolidate()
        return int(self._k.shape[0])

    def add(self, xyz: np.ndarray, intensity=None) -> None:
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        ok = np.isfinite(xyz).all(axis=1)
        xyz = xyz[ok]
        if xyz.shape[0] == 0:
            return
        inten = np.zeros(xyz.shape[0]) if intensity is None else np.asarray(intensity, np.float64).reshape(-1)[ok]
        keys = np.floor(xyz / self.voxel).astype(np.int64)
        inrange = np.all(np.abs(keys) < _OFF, axis=1)
        n = int(inrange.sum())
        if n == 0:
            return
        self._pending.append((_pack(keys[inrange]), xyz[inrange], inten[inrange], np.ones(n)))
        self._pending_n += n
        self.revision += 1
        if self._pending_n >= self.consolidate_every:
            self._consolidate()

    def _consolidate(self) -> None:
        if not self._pending:
            return
        k = np.concatenate([self._k] + [p[0] for p in self._pending])
        s = np.concatenate([self._s] + [p[1] for p in self._pending])
        i = np.concatenate([self._i] + [p[2] for p in self._pending])
        c = np.concatenate([self._c] + [p[3] for p in self._pending])
        self._pending, self._pending_n = [], 0
        uk, inv = np.unique(k, return_inverse=True)
        inv = inv.reshape(-1)
        ss = np.zeros((uk.shape[0], 3))
        np.add.at(ss, inv, s)
        ii = np.bincount(inv, weights=i, minlength=uk.shape[0])
        cc = np.bincount(inv, weights=c, minlength=uk.shape[0])
        if uk.shape[0] > self.max_points:
            # Keep the best-observed voxels when the budget is exceeded.
            keep = np.sort(np.argsort(-cc, kind="stable")[: self.max_points])
            uk, ss, ii, cc = uk[keep], ss[keep], ii[keep], cc[keep]
        self._k, self._s, self._i, self._c = uk, ss, ii, cc

    def points(self):
        self._consolidate()
        if self._k.shape[0] == 0:
            return np.zeros((0, 3), np.float32), np.zeros(0, np.float32)
        xyz = self._s / self._c[:, None]
        inten = self._i / self._c
        return xyz.astype(np.float32), inten.astype(np.float32)

    def save(self, path: str) -> int:
        """Write the map as binary PCD (atomically: temp file + rename). Returns the point count."""
        xyz, inten = self.points()
        pcd_io.write_pcd(path, xyz, inten, binary=True)
        return xyz.shape[0]


class MapSaver:
    """Decides when and where the map is written. No ROS, so the policy is unit-testable."""

    def __init__(self, vmap: VoxelMap, out_dir: str, clock=time.time):
        self.vmap = vmap
        self.out_dir = out_dir
        self._clock = clock
        self._saved_revision = 0       # revision covered by the last save of any kind

    def timestamped_path(self) -> str:
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self._clock()))
        return os.path.join(self.out_dir, f"hummingbird_map_{stamp}.pcd")

    @property
    def dirty(self) -> bool:
        return self.vmap.revision != self._saved_revision

    def save_now(self) -> tuple:
        """Operator request: always write a new time-stamped file. Returns (path, points)."""
        path = self.timestamped_path()
        revision = self.vmap.revision
        n = self.vmap.save(path)
        self._saved_revision = revision
        return path, n

    def autosave(self):
        """Rolling file; skipped when nothing changed. Returns (path, points) or None."""
        if not self.dirty or len(self.vmap) == 0:
            return None
        path = os.path.join(self.out_dir, AUTOSAVE_NAME)
        revision = self.vmap.revision
        n = self.vmap.save(path)
        self._saved_revision = revision
        return path, n

    def save_on_exit(self):
        """Time-stamped file at shutdown, unless the map is empty or already saved."""
        if not self.dirty or len(self.vmap) == 0:
            return None
        return self.save_now()


def _cloud_to_numpy(msg):
    from sensor_msgs_py import point_cloud2

    names = [f.name for f in msg.fields]
    want = ["x", "y", "z"] + (["intensity"] if "intensity" in names else [])
    arr = point_cloud2.read_points(msg, field_names=want, skip_nans=True)
    arr = np.asarray(arr)
    if arr.size == 0:
        return np.zeros((0, 3)), None
    xyz = np.column_stack([arr["x"], arr["y"], arr["z"]])
    inten = arr["intensity"] if "intensity" in want else None
    return xyz, inten


def main(args=None):
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from std_srvs.srv import Trigger

    init_ros(args)
    node = Node("hummingbird_map_recorder")
    topic = node.declare_parameter("cloud_topic", "/cloud_registered").value
    voxel = float(node.declare_parameter("voxel_size", 0.05).value)
    out_dir = os.path.expanduser(node.declare_parameter("output_dir", "~/hummingbird_maps").value)
    max_pts = int(node.declare_parameter("max_points", 10_000_000).value)
    autosave_period = float(node.declare_parameter("autosave_period_s", 60.0).value)
    save_on_exit = bool(node.declare_parameter("save_on_exit", True).value)
    os.makedirs(out_dir, exist_ok=True)
    vmap = VoxelMap(voxel, max_pts)
    saver = MapSaver(vmap, out_dir)

    def on_cloud(msg):
        xyz, inten = _cloud_to_numpy(msg)
        vmap.add(xyz, inten)

    def on_save(_req, res):
        try:
            path, n = saver.save_now()
            res.success, res.message = True, f"saved {n} points to {path}"
        except Exception as exc:  # noqa: BLE001 - report any IO error to the caller
            res.success, res.message = False, f"save failed: {exc}"
        node.get_logger().info(res.message)
        return res

    def on_autosave():
        try:
            saved = saver.autosave()
        except Exception as exc:  # noqa: BLE001 - a full disk must not kill the recorder
            node.get_logger().error(f"autosave failed: {exc}")
            return
        if saved:
            node.get_logger().info(f"autosaved {saved[1]} points to {saved[0]}")

    # Depth 50: a merge of a large map can block this callback for a second or more.
    node.create_subscription(PointCloud2, topic, on_cloud, 50)
    node.create_service(Trigger, "/hummingbird/save_map", on_save)
    if autosave_period > 0:
        node.create_timer(autosave_period, on_autosave)
    node.get_logger().info(
        f"map_recorder: {topic} voxel={voxel} m -> {out_dir} "
        f"(autosave every {autosave_period:g} s, save_on_exit={save_on_exit})"
    )

    def on_exit():
        # Runs after the spin loop stopped and before ROS shuts down. Launch only escalates to
        # SIGKILL after sigterm_timeout + sigkill_timeout (see mapping.launch.py).
        if not save_on_exit:
            return
        try:
            saved = saver.save_on_exit()
            if saved:
                node.get_logger().info(f"saved {saved[1]} points to {saved[0]} on shutdown")
        except Exception as exc:  # noqa: BLE001 - nothing left to do but report it
            node.get_logger().error(f"save on shutdown failed: {exc}")

    spin_until_stopped(node, before_shutdown=on_exit)


if __name__ == "__main__":
    main()
