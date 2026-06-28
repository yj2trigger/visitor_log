from __future__ import annotations
import numpy as np
import zstandard as zstd

POINT_FLOATS = 6
POINT_BYTES  = POINT_FLOATS * 4


def validate_and_parse(raw_bytes: bytes, expected_count: int) -> np.ndarray:
    expected_bytes = expected_count * POINT_BYTES
    if len(raw_bytes) != expected_bytes:
        raise ValueError(f"bytes mismatch: got {len(raw_bytes)}, expected {expected_bytes}")
    arr = np.frombuffer(raw_bytes, dtype=np.float32).reshape(expected_count, POINT_FLOATS)
    if not np.isfinite(arr).all():
        raise ValueError("point cloud contains NaN or Inf values")
    return arr


def compress(arr: np.ndarray) -> bytes:
    raw = arr.astype(np.float32).tobytes()
    return zstd.ZstdCompressor(level=3).compress(raw)


def decompress(compressed: bytes, point_count: int) -> np.ndarray:
    raw = zstd.ZstdDecompressor().decompress(compressed, max_output_size=point_count * POINT_BYTES)
    return np.frombuffer(raw, dtype=np.float32).reshape(point_count, POINT_FLOATS)


def downsample_voxel(arr: np.ndarray, voxel_size: float = 0.02) -> np.ndarray:
    if len(arr) == 0:
        return arr
    voxel_indices = np.floor(arr[:, :3] / voxel_size).astype(np.int32)
    _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
    return arr[np.sort(unique_idx)]
