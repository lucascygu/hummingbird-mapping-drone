"""CLI: downsample / crop / convert a FAST-LIO PCD map.

  ros2 run hummingbird_bringup pcd_export in.pcd out.pcd --voxel 0.1
  ros2 run hummingbird_bringup pcd_export in.pcd out.ply --voxel 0.05 --crop -50 -50 -5 50 50 30
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from hummingbird_bringup import pcd_io


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("output", help=".pcd | .ply | .xyz")
    ap.add_argument("--voxel", type=float, default=0.1, help="voxel size in metres (0 = no downsampling)")
    ap.add_argument("--crop", type=float, nargs=6, metavar=("XMIN", "YMIN", "ZMIN", "XMAX", "YMAX", "ZMAX"))
    ap.add_argument("--ascii", action="store_true", help="write ASCII PCD")
    a = ap.parse_args(argv)

    cloud = pcd_io.read_pcd(a.input)
    xyz = np.column_stack([cloud["x"], cloud["y"], cloud["z"]]).astype(np.float64)
    inten = cloud["intensity"].astype(np.float64) if "intensity" in cloud.dtype.names else None
    n_in = xyz.shape[0]
    if a.crop:
        xyz, inten = pcd_io.crop_box(xyz, a.crop[:3], a.crop[3:], inten)
    xyz, inten = pcd_io.voxel_downsample(xyz, a.voxel, inten)

    out = a.output.lower()
    if out.endswith(".pcd"):
        pcd_io.write_pcd(a.output, xyz, inten, binary=not a.ascii)
    elif out.endswith(".ply"):
        pcd_io.write_ply(a.output, xyz, inten)
    elif out.endswith(".xyz"):
        np.savetxt(a.output, xyz, fmt="%.4f")
    else:
        print("output must end with .pcd, .ply or .xyz", file=sys.stderr)
        return 2
    print(f"{a.input}: {n_in} points -> {a.output}: {xyz.shape[0]} points (voxel {a.voxel} m)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
