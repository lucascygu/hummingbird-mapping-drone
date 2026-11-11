# Generated / assisted by Grok
# Reviewed by Claude: atomic writes, empty ASCII clouds, fast voxel keys
"""Minimal PCD reader/writer + voxel downsampling (numpy only, no PCL/Open3D).

Supports DATA ascii and DATA binary (what FAST-LIO / pcl::PCDWriter::writeBinary produce).
binary_compressed is rejected with a clear error.
"""

from __future__ import annotations

import io
import os
from typing import Dict, List, Optional, Tuple

import numpy as np

_TYPE = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4",
         ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4", ("U", 8): "u8", ("I", 8): "i8"}


def _parse_header(fh) -> Tuple[Dict[str, List[str]], int]:
    header: Dict[str, List[str]] = {}
    while True:
        line = fh.readline()
        if not line:
            raise ValueError("unexpected EOF in PCD header")
        text = line.decode("ascii", errors="replace").strip()
        if not text or text.startswith("#"):
            continue
        key, *vals = text.split()
        header[key.upper()] = vals
        if key.upper() == "DATA":
            return header, fh.tell()


def _dtype(header) -> np.dtype:
    names = header["FIELDS"]
    sizes = [int(s) for s in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(c) for c in header.get("COUNT", ["1"] * len(names))]
    fields = []
    for i, (n, s, t, c) in enumerate(zip(names, sizes, types, counts)):
        name = n if n != "_" else f"_pad{i}"
        base = _TYPE[(t.upper(), s)]
        fields.append((name, base) if c == 1 else (name, base, (c,)))
    return np.dtype(fields)


def read_pcd(path: str) -> np.ndarray:
    """Return a structured array with at least x, y, z."""
    with open(path, "rb") as fh:
        header, _ = _parse_header(fh)
        dt = _dtype(header)
        n = int(header["POINTS"][0]) if "POINTS" in header else int(header["WIDTH"][0]) * int(header["HEIGHT"][0])
        kind = header["DATA"][0].lower()
        if kind == "binary":
            data = np.frombuffer(fh.read(n * dt.itemsize), dtype=dt, count=n)
        elif kind == "ascii":
            if n == 0:
                rows = np.zeros((0, sum(int(np.prod(dt[k].shape)) if dt[k].shape else 1 for k in dt.names)))
            else:
                rows = np.loadtxt(io.TextIOWrapper(fh, encoding="ascii"), ndmin=2)
            data = np.zeros(rows.shape[0], dtype=dt)
            col = 0
            for name in dt.names:
                width = int(np.prod(dt[name].shape)) if dt[name].shape else 1
                vals = rows[:, col:col + width]
                data[name] = vals if width > 1 else vals[:, 0]
                col += width
        else:
            raise ValueError(f"PCD DATA {kind!r} not supported (re-save as binary with pcl_convert_pcd_ascii_binary)")
    for k in ("x", "y", "z"):
        if k not in data.dtype.names:
            raise ValueError("PCD has no x/y/z fields")
    return data


def write_pcd(path: str, xyz: np.ndarray, intensity: Optional[np.ndarray] = None, binary: bool = True) -> None:
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    fields = [("x", "f4"), ("y", "f4"), ("z", "f4")]
    if intensity is not None:
        fields.append(("intensity", "f4"))
    arr = np.zeros(xyz.shape[0], dtype=np.dtype(fields))
    arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    if intensity is not None:
        arr["intensity"] = np.asarray(intensity, dtype=np.float32).reshape(-1)
    names = " ".join(n for n, _ in fields)
    k = len(fields)
    head = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        f"FIELDS {names}\nSIZE {' '.join(['4'] * k)}\nTYPE {' '.join(['F'] * k)}\nCOUNT {' '.join(['1'] * k)}\n"
        f"WIDTH {arr.shape[0]}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {arr.shape[0]}\n"
        f"DATA {'binary' if binary else 'ascii'}\n"
    )
    # Write next to the target and rename: a crash or a dying battery in the middle of a save
    # must never leave a truncated file where the previous good map used to be.
    tmp = f"{path}.tmp"
    with open(tmp, "wb") as fh:
        fh.write(head.encode("ascii"))
        if binary:
            fh.write(arr.tobytes())
        else:
            np.savetxt(fh, np.column_stack([arr[n] for n, _ in fields]), fmt="%.6f")
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def write_ply(path: str, xyz: np.ndarray, intensity: Optional[np.ndarray] = None) -> None:
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    props = "property float x\nproperty float y\nproperty float z\n"
    cols = [xyz]
    if intensity is not None:
        props += "property float intensity\n"
        cols.append(np.asarray(intensity, dtype=np.float32).reshape(-1, 1))
    body = np.ascontiguousarray(np.hstack(cols).astype("<f4"))
    head = f"ply\nformat binary_little_endian 1.0\nelement vertex {xyz.shape[0]}\n{props}end_header\n"
    with open(path, "wb") as fh:
        fh.write(head.encode("ascii"))
        fh.write(body.tobytes())


def voxel_downsample(xyz: np.ndarray, voxel: float, intensity: Optional[np.ndarray] = None):
    """Centroid-per-voxel downsampling. Returns (xyz, intensity_or_None)."""
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    if voxel <= 0 or xyz.shape[0] == 0:
        return xyz.astype(np.float32), intensity
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    if intensity is not None:
        intensity = np.asarray(intensity, dtype=np.float64).reshape(-1)[finite]
    if xyz.shape[0] == 0:
        return xyz.astype(np.float32), (None if intensity is None else intensity.astype(np.float32))
    keys = np.floor(xyz / voxel).astype(np.int64)
    keys -= keys.min(axis=0)
    if int(keys.max()) < (1 << 21):
        # 21 bits per axis fit one int64: sorting scalars is far faster than np.unique(axis=0).
        packed = (keys[:, 0] << 42) | (keys[:, 1] << 21) | keys[:, 2]
        _, inv, counts = np.unique(packed, return_inverse=True, return_counts=True)
    else:
        _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    out = np.zeros((counts.shape[0], 3))
    np.add.at(out, inv, xyz)
    out /= counts[:, None]
    out_i = None
    if intensity is not None:
        out_i = np.zeros(counts.shape[0])
        np.add.at(out_i, inv, intensity)
        out_i /= counts
        out_i = out_i.astype(np.float32)
    return out.astype(np.float32), out_i


def crop_box(xyz: np.ndarray, lo, hi, intensity=None):
    xyz = np.asarray(xyz).reshape(-1, 3)
    m = np.all((xyz >= np.asarray(lo)) & (xyz <= np.asarray(hi)), axis=1)
    return xyz[m], (None if intensity is None else np.asarray(intensity)[m])
