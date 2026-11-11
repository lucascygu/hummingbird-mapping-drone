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
