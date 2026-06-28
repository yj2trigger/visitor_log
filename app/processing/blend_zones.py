from __future__ import annotations
import numpy as np


def smooth_step(edge0: float, edge1: float, x: float) -> float:
    t = max(0.0, min(1.0, (x - edge0) / (edge1 - edge0)))
    return t * t * (3.0 - 2.0 * t)


def past_scene_alpha(
    frag_world_pos: np.ndarray,
    anchor_world_positions: list[np.ndarray],
    inner_radius: float = 0.3,
    outer_radius: float = 1.2,
) -> float:
    if not anchor_world_positions:
        return 0.0
    max_alpha = 0.0
    for anchor_pos in anchor_world_positions:
        dist = float(np.linalg.norm(frag_world_pos - anchor_pos))
        alpha = 1.0 - smooth_step(inner_radius, outer_radius, dist)
        max_alpha = max(max_alpha, alpha)
    return max_alpha
