# Generated / assisted by Claude
"""2.5D height tiles for air -> ground map sharing (numpy only, no ROS).

docs/air-ground-cooperation.md asks for small ``MapTile`` fragments instead of the full dense
cloud. This module keeps a sparse grid of square tiles; each cell stores the highest point seen
in it, as int16 centimetres above ``origin_z`` (``UNKNOWN`` where nothing was observed).

Layout of one tile (matches hummingbird_interfaces/MapTile with data_kind "height"):
  * ``width`` = ``height`` = ``tile_cells`` cells of ``resolution`` metres;
  * the cell (row, col) covers x in [origin_x + col*res, origin_x + (col+1)*res) and
    y in [origin_y + row*res, origin_y + (row+1)*res);
  * ``data[row * width + col]`` is that cell's value.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

UNKNOWN = -32768
_MAX_CM = 32767
_MIN_CM = -32767


@dataclass(frozen=True)
class Tile:
    ix: int
    iy: int
    origin_x: float
    origin_y: float
    origin_z: float
    resolution: float
    cells: int
    data: np.ndarray  # int16, shape (cells, cells), [row=y, col=x]

    @property
    def tile_id(self) -> str:
        return f"height/{self.ix}/{self.iy}"


class HeightTiles:
    def __init__(self, resolution: float = 0.25, tile_cells: int = 64, origin_z: float = 0.0,
                 min_z: float = -1000.0, max_z: float = 1000.0, max_tiles: int = 4096):
        if resolution <= 0 or tile_cells < 1 or max_tiles < 1:
            raise ValueError("resolution must be > 0, tile_cells and max_tiles >= 1")
        if not min_z < max_z:
            raise ValueError("min_z must be below max_z")
        self.resolution = float(resolution)
        self.tile_cells = int(tile_cells)
        self.origin_z = float(origin_z)
        self.min_z = float(min_z)
        self.max_z = float(max_z)
        self.max_tiles = int(max_tiles)
        self._tiles: Dict[Tuple[int, int], np.ndarray] = {}
        self._dirty: Dict[Tuple[int, int], None] = {}   # insertion-ordered set
        self._sent_at: Dict[Tuple[int, int], int] = {}  # publish counter at last send
        self._send_counter = 0
        self.dropped_points = 0   # points that fell into tiles beyond max_tiles

    def __len__(self) -> int:
        return len(self._tiles)

    def add(self, xyz: np.ndarray) -> int:
        """Merge world-frame points. Returns how many tiles changed."""
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        ok = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] >= self.min_z) & (xyz[:, 2] <= self.max_z)
        xyz = xyz[ok]
        if xyz.shape[0] == 0:
            return 0
        cell = np.floor(xyz[:, :2] / self.resolution).astype(np.int64)
        tile = np.floor_divide(cell, self.tile_cells)
        local = cell - tile * self.tile_cells
        flat = local[:, 1] * self.tile_cells + local[:, 0]
        h_cm = np.clip(np.rint((xyz[:, 2] - self.origin_z) * 100.0), _MIN_CM, _MAX_CM).astype(np.int16)

        changed = 0
        order = np.lexsort((tile[:, 1], tile[:, 0]))
        tile, flat, h_cm = tile[order], flat[order], h_cm[order]
        starts = np.flatnonzero(np.r_[True, np.any(tile[1:] != tile[:-1], axis=1)])
        ends = np.r_[starts[1:], tile.shape[0]]
        for a, b in zip(starts, ends):
            key = (int(tile[a, 0]), int(tile[a, 1]))
            grid = self._tiles.get(key)
            if grid is None:
                if len(self._tiles) >= self.max_tiles:
                    self.dropped_points += int(b - a)
                    continue
                grid = np.full(self.tile_cells * self.tile_cells, UNKNOWN, np.int16)
                self._tiles[key] = grid
            before = grid.copy()
            np.maximum.at(grid, flat[a:b], h_cm[a:b])
            if not np.array_equal(before, grid):
                self._dirty[key] = None
                changed += 1
        return changed

    def _tile(self, key: Tuple[int, int]) -> Tile:
        size = self.tile_cells * self.resolution
        return Tile(key[0], key[1], key[0] * size, key[1] * size, self.origin_z, self.resolution,
                    self.tile_cells, self._tiles[key].reshape(self.tile_cells, self.tile_cells).copy())

    def take(self, budget: int, refresh: bool = True) -> List[Tile]:
        """Up to ``budget`` tiles to publish now: changed tiles first (oldest change first), then,
        if ``refresh``, the tiles that were sent longest ago, so a ground station that joined late
        or lost packets still converges to the full map."""
        out: List[Tuple[int, int]] = []
        for key in list(self._dirty):
            if len(out) >= budget:
                break
            out.append(key)
            del self._dirty[key]
        if refresh and len(out) < budget:
            taken = set(out)
            rest = sorted((k for k in self._tiles if k not in taken and k not in self._dirty),
                          key=lambda k: self._sent_at.get(k, -1))
            out.extend(rest[: budget - len(out)])
        for key in out:
            self._send_counter += 1
            self._sent_at[key] = self._send_counter
        return [self._tile(k) for k in out]

    def height_at(self, x: float, y: float) -> Optional[float]:
        """Height in metres (absolute z) of the cell containing (x, y), or None if unknown."""
        cx, cy = int(np.floor(x / self.resolution)), int(np.floor(y / self.resolution))
        key = (cx // self.tile_cells, cy // self.tile_cells)
        grid = self._tiles.get(key)
        if grid is None:
            return None
        v = int(grid[(cy - key[1] * self.tile_cells) * self.tile_cells + (cx - key[0] * self.tile_cells)])
        return None if v == UNKNOWN else self.origin_z + v / 100.0
