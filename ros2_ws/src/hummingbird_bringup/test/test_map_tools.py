# Generated / assisted by Grok
import numpy as np
import pytest

from hummingbird_bringup import pcd_export, pcd_io
from hummingbird_bringup.map_recorder import VoxelMap


def _cloud(n=5000, seed=0):
    rng = np.random.default_rng(seed)
    return rng.uniform(-10, 10, (n, 3)), rng.uniform(0, 255, n)


@pytest.mark.parametrize("binary", [True, False])
def test_pcd_roundtrip(tmp_path, binary):
    xyz, inten = _cloud(100)
    p = tmp_path / "a.pcd"
    pcd_io.write_pcd(str(p), xyz, inten, binary=binary)
    c = pcd_io.read_pcd(str(p))
    assert c.shape[0] == 100
    np.testing.assert_allclose(c["x"], xyz[:, 0].astype(np.float32), atol=1e-5)
    np.testing.assert_allclose(c["intensity"], inten.astype(np.float32), atol=1e-4)


def test_reads_pcl_style_padded_binary(tmp_path):
    # pcl::PointXYZINormal-like layout with padding field "_" (FAST-LIO PointType)
    dt = np.dtype([("x", "f4"), ("y", "f4"), ("z", "f4"), ("_pad3", "u1", (4,)), ("intensity", "f4")])
    arr = np.zeros(3, dt)
    arr["x"] = [1, 2, 3]
    arr["intensity"] = [7, 8, 9]
    head = ("VERSION 0.7\nFIELDS x y z _ intensity\nSIZE 4 4 4 1 4\nTYPE F F F U F\nCOUNT 1 1 1 4 1\n"
            "WIDTH 3\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA binary\n")
    p = tmp_path / "pcl.pcd"
    p.write_bytes(head.encode() + arr.tobytes())
    c = pcd_io.read_pcd(str(p))
    assert list(c["x"]) == [1, 2, 3] and list(c["intensity"]) == [7, 8, 9]


def test_compressed_rejected(tmp_path):
    p = tmp_path / "c.pcd"
    p.write_bytes(b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary_compressed\n")
    with pytest.raises(ValueError):
        pcd_io.read_pcd(str(p))


def test_voxel_downsample_centroids():
    xyz = np.array([[0.01, 0.01, 0.01], [0.03, 0.03, 0.03], [1.05, 0, 0]])
    out, inten = pcd_io.voxel_downsample(xyz, 0.1, np.array([1.0, 3.0, 5.0]))
    assert out.shape == (2, 3)
    order = np.argsort(out[:, 0])
    np.testing.assert_allclose(out[order][0], [0.02, 0.02, 0.02], atol=1e-6)
    np.testing.assert_allclose(inten[order], [2.0, 5.0])


def test_voxel_map_matches_batch_downsample():
    xyz, inten = _cloud(20000, 1)
    vm = VoxelMap(0.5, consolidate_every=3000)
    for chunk in np.array_split(np.arange(xyz.shape[0]), 9):
        vm.add(xyz[chunk], inten[chunk])
    a, _ = vm.points()
    b, _ = pcd_io.voxel_downsample(xyz, 0.5, inten)
    assert a.shape == b.shape
    np.testing.assert_allclose(np.sort(a, axis=0), np.sort(b, axis=0), atol=1e-4)


def test_voxel_map_budget_and_nan():
    vm = VoxelMap(0.1, max_points=10)
    xyz, _ = _cloud(1000, 2)
    xyz[0] = np.nan
    vm.add(xyz)
    assert len(vm) == 10


def test_pcd_export_cli(tmp_path):
    xyz, inten = _cloud(5000, 3)
    src = tmp_path / "in.pcd"
    pcd_io.write_pcd(str(src), xyz, inten)
    for name in ("out.pcd", "out.ply", "out.xyz"):
        dst = tmp_path / name
        assert pcd_export.main([str(src), str(dst), "--voxel", "1.0", "--crop", "-5", "-5", "-5", "5", "5", "5"]) == 0
        assert dst.stat().st_size > 0
    c = pcd_io.read_pcd(str(tmp_path / "out.pcd"))
    assert 0 < c.shape[0] < 5000
    assert np.all(np.abs(np.column_stack([c["x"], c["y"], c["z"]])) <= 5.0 + 1e-6)


# --------------------------------------------------------------------------- added by Claude
from hummingbird_bringup.map_recorder import AUTOSAVE_NAME, MapSaver  # noqa: E402
from hummingbird_bringup.map_tiles import UNKNOWN, HeightTiles  # noqa: E402


def test_write_pcd_is_atomic_and_leaves_no_temp_file(tmp_path):
    p = tmp_path / "m.pcd"
    pcd_io.write_pcd(str(p), np.zeros((3, 3)))
    good = p.read_bytes()
    with pytest.raises(ValueError):                       # a failing save ...
        pcd_io.write_pcd(str(p), np.zeros((2, 3)), intensity=np.zeros(5))
    assert p.read_bytes() == good                         # ... leaves the previous map untouched
    pcd_io.write_pcd(str(p), np.ones((4, 3)))
    assert [f.name for f in tmp_path.iterdir()] == ["m.pcd"]
    assert pcd_io.read_pcd(str(p)).shape[0] == 4


@pytest.mark.parametrize("binary", [True, False])
def test_empty_cloud_roundtrip(tmp_path, binary):
    p = tmp_path / "empty.pcd"
    pcd_io.write_pcd(str(p), np.zeros((0, 3)), np.zeros(0), binary=binary)
    c = pcd_io.read_pcd(str(p))
    assert c.shape[0] == 0 and {"x", "y", "z", "intensity"} <= set(c.dtype.names)
    out, inten = pcd_io.voxel_downsample(np.zeros((0, 3)), 0.1, np.zeros(0))
    assert out.shape == (0, 3)


def test_voxel_downsample_wide_extent_and_nan():
    # Far apart points exceed 21 bits per axis at 1 mm voxels: the slow path must still be right.
    xyz = np.array([[0.0, 0.0, 0.0], [0.0004, 0.0, 0.0], [5000.0, 0.0, 0.0], [np.nan, 0.0, 0.0]])
    out, _ = pcd_io.voxel_downsample(xyz, 0.001)
    assert out.shape == (2, 3)
    np.testing.assert_allclose(np.sort(out[:, 0]), [0.0002, 5000.0], atol=1e-3)


def test_map_saver_policy(tmp_path):
    vm = VoxelMap(0.5)
    now = [1_700_000_000.0]
    saver = MapSaver(vm, str(tmp_path), clock=lambda: now[0])
    assert saver.autosave() is None and saver.save_on_exit() is None      # empty map: nothing written
    assert list(tmp_path.iterdir()) == []

    vm.add(np.array([[0.1, 0.1, 0.1], [3.0, 3.0, 3.0]]))
    path, n = saver.autosave()
    assert path.endswith(AUTOSAVE_NAME) and n == 2
    assert saver.autosave() is None                                        # unchanged: skipped
    assert saver.save_on_exit() is None                                    # already on disk

    vm.add(np.array([[9.0, 9.0, 9.0]]))
    assert saver.dirty
    path2, n2 = saver.save_on_exit()                                       # new data -> time-stamped file
    assert n2 == 3 and path2 != path and "hummingbird_map_20" in path2
    assert pcd_io.read_pcd(path2).shape[0] == 3

    now[0] += 5.0
    path3, _ = saver.save_now()                                            # operator request: always writes
    assert path3 != path2 and len(list(tmp_path.iterdir())) == 3


def test_height_tiles_keep_the_highest_point_per_cell():
    ht = HeightTiles(resolution=0.5, tile_cells=4)             # one tile = 2 m x 2 m
    changed = ht.add(np.array([[0.1, 0.1, 1.00], [0.2, 0.3, 2.50], [1.9, 1.9, -0.30],
                               [-0.1, 0.1, 0.70], [np.nan, 0.0, 0.0]]))
    assert changed == 2 and len(ht) == 2
    assert ht.height_at(0.25, 0.25) == pytest.approx(2.5)
    assert ht.height_at(1.75, 1.75) == pytest.approx(-0.3)
    assert ht.height_at(-0.25, 0.25) == pytest.approx(0.7)     # negative side lands in tile (-1, 0)
    assert ht.height_at(1.25, 0.25) is None and ht.height_at(50.0, 50.0) is None
    assert ht.add(np.array([[0.1, 0.1, 0.5]])) == 0            # lower point: nothing changes

    tiles = {t.tile_id: t for t in ht.take(10, refresh=False)}
    assert set(tiles) == {"height/0/0", "height/-1/0"}
    t = tiles["height/0/0"]
    assert (t.origin_x, t.origin_y, t.cells, t.resolution) == (0.0, 0.0, 4, 0.5)
    assert t.data.shape == (4, 4) and t.data.dtype == np.int16
    assert t.data[0, 0] == 250 and t.data[3, 3] == -30         # [row = y, col = x], centimetres
    assert int((t.data == UNKNOWN).sum()) == 14
    assert tiles["height/-1/0"].origin_x == -2.0 and tiles["height/-1/0"].data[0, 3] == 70


def test_height_tiles_publish_order_and_refresh():
    ht = HeightTiles(resolution=1.0, tile_cells=2)
    for k in range(5):
        ht.add(np.array([[2.0 * k + 0.5, 0.5, 1.0]]))
    first = [t.tile_id for t in ht.take(3, refresh=True)]
    assert first == ["height/0/0", "height/1/0", "height/2/0"]             # changed tiles, oldest first
    ht.add(np.array([[0.5, 0.5, 3.0]]))                                    # tile 0 changes again
    second = [t.tile_id for t in ht.take(3, refresh=True)]
    assert second == ["height/3/0", "height/4/0", "height/0/0"]            # still oldest change first
    third = [t.tile_id for t in ht.take(2, refresh=True)]
    assert third == ["height/1/0", "height/2/0"]                           # round-robin refresh
    assert ht.take(5, refresh=False) == []                                 # nothing changed


def test_height_tiles_band_clamp_and_budget():
    ht = HeightTiles(resolution=1.0, tile_cells=2, origin_z=10.0, min_z=0.0, max_z=20.0, max_tiles=1)
    ht.add(np.array([[0.5, 0.5, 12.0], [0.5, 1.5, 25.0], [1.5, 0.5, -3.0]]))   # two outside the band
    assert ht.height_at(0.5, 0.5) == pytest.approx(12.0)
    assert ht.height_at(0.5, 1.5) is None and ht.height_at(1.5, 0.5) is None
    ht.add(np.array([[100.0, 100.0, 5.0]]))                                # second tile: over budget
    assert len(ht) == 1 and ht.dropped_points == 1
    big = HeightTiles(resolution=1.0, tile_cells=2)
    big.add(np.array([[0.5, 0.5, 900.0]]))                                 # beyond int16 centimetres
    assert big.height_at(0.5, 0.5) == pytest.approx(327.67)
    with pytest.raises(ValueError):
        HeightTiles(resolution=0.0)
    with pytest.raises(ValueError):
        HeightTiles(min_z=1.0, max_z=1.0)
