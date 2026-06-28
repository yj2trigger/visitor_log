# realtime.py — 웹캠 실시간 카메라→3D 파이프라인 spike
#
# [실행 방법]
#     pip install -r spike/requirements_poc.txt opencv-python open3d keyboard
#     python spike/realtime.py
#
# [조작] 창 포커스 무관하게 작동
#     W/S         : 앞/뒤 (줌)
#     A/D         : 좌/우 이동
#     위/아래 화살표: 상/하 이동
#     마우스 드래그 : 시점 회전 (Open3D 창)
#     스크롤       : 줌 (Open3D 창)
#     s           : 현재 씬 타임스탬프 .ply 저장 + 블렌딩 시각화
#     [ / ]       : 저장된 .ply 파일 이전/다음 선택 후 뷰어 표시
#     q           : 종료
#
# [서비스 전환 시 교체 항목]
#     웹캠 + Depth-Anything v2 (현재 spike)
#         → Android (S24 등): ARCore Depth API — DEPTH_SCALE 불필요
#         → iPhone (12 Pro 이상): ARKit sceneDepth.depthMap (LiDAR) — DEPTH_SCALE 불필요
#         → iPhone (LiDAR 없는 모델): ARKit estimatedDepthData (ML 추정, 정확도 낮음)

import os
import sys
import threading
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import cv2
import keyboard
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from PIL import Image
from transformers import pipeline as hf_pipeline

from app.processing.pointcloud import downsample_voxel
from app.processing.blend_zones import past_scene_alpha

# ── 설정 상수 ──────────────────────────────────────────────────────────────
CAMERA_INDEX = 0       # 기본 웹캠
DEPTH_W      = 320     # 모델 입력 너비 (작을수록 빠름)
DEPTH_H      = 240     # 모델 입력 높이
VOXEL_SIZE   = 0.02    # 다운샘플 복셀 크기 (미터)
DEPTH_SCALE  = 30.0    # spike 전용 임시값. ARCore 전환 시 삭제
MAX_DEPTH    = 50.0    # 이 거리 초과 점 제거

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")

# ── 스레드 공유 상태 ────────────────────────────────────────────────────────
depth_lock          = threading.Lock()
latest_depth_result = None   # (depth_map, original_frame) 또는 None


def load_model():
    print("모델 로드 중... (최초 실행 시 ~100MB 다운로드)")
    return hf_pipeline(
        task="depth-estimation",
        model="depth-anything/Depth-Anything-V2-Small-hf",
    )


def depth_worker(pipe, rgb_small: np.ndarray, original_frame: np.ndarray) -> None:
    """백그라운드 스레드: depth 추정 후 결과를 latest_depth_result에 저장."""
    global latest_depth_result
    depth = estimate_depth(pipe, rgb_small)
    with depth_lock:
        latest_depth_result = (depth, original_frame)


def estimate_depth(pipe, rgb_arr: np.ndarray) -> np.ndarray:
    pil_img = Image.fromarray((rgb_arr * 255).astype(np.uint8))
    result = pipe(pil_img)
    depth = np.array(result["depth"], dtype=np.float32) / 255.0
    return depth * DEPTH_SCALE


def frame_to_rgb(frame: np.ndarray) -> np.ndarray:
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return rgb.astype(np.float32) / 255.0


def estimate_intrinsics(w: int, h: int) -> tuple[float, float, float, float]:
    fov_rad = np.radians(60)
    fy = h / (2 * np.tan(fov_rad / 2))
    fx = fy
    cx, cy = w / 2.0, h / 2.0
    return fx, fy, cx, cy


def unproject(rgb: np.ndarray, depth: np.ndarray,
              fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    h, w = depth.shape
    u = np.arange(w, dtype=np.float32)
    v = np.arange(h, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)

    mask = (depth > 0) & (depth <= MAX_DEPTH)

    X = (uu - cx) / fx * depth
    Y = -(vv - cy) / fy * depth
    Z = depth

    xyz = np.stack([X[mask], Y[mask], Z[mask]], axis=-1)
    rgb_flat = rgb[mask]
    return np.concatenate([xyz, rgb_flat], axis=-1).astype(np.float32)


def estimate_motion(prev_gray: np.ndarray, curr_gray: np.ndarray,
                    fx: float, cx: float, cy: float) -> np.ndarray:
    """Visual Odometry: 특징점 추적 → 4×4 변환행렬.
    ARCore 전환 시 이 함수 전체를 ARCore Pose로 교체.
    """
    pts_prev = cv2.goodFeaturesToTrack(prev_gray, maxCorners=300,
                                       qualityLevel=0.01, minDistance=7)
    if pts_prev is None or len(pts_prev) < 8:
        return np.eye(4, dtype=np.float32)

    pts_curr, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray,
                                                    pts_prev, None)
    mask = (status.ravel() == 1)
    if mask.sum() < 8:
        return np.eye(4, dtype=np.float32)

    p0, p1 = pts_prev[mask], pts_curr[mask]
    K = np.array([[fx, 0, cx], [0, fx, cy], [0, 0, 1]], dtype=np.float64)
    E, em = cv2.findEssentialMat(p1, p0, K, method=cv2.RANSAC,
                                 prob=0.999, threshold=1.0)
    if E is None:
        return np.eye(4, dtype=np.float32)

    _, R, t, _ = cv2.recoverPose(E, p1, p0, K, mask=em)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R.astype(np.float32)
    T[:3, 3]  = (t.ravel() * DEPTH_SCALE * 0.1).astype(np.float32)
    return T


def make_o3d_pointcloud(points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(points[:, 3:])
    return pcd


def save_ply(points: np.ndarray, path: str) -> None:
    n = len(points)
    header = (
        "ply\nformat ascii 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    xyz = points[:, :3]
    rgb = (points[:, 3:] * 255).astype(np.uint8)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(header)
        for i in range(n):
            f.write(f"{xyz[i,0]:.4f} {xyz[i,1]:.4f} {xyz[i,2]:.4f} "
                    f"{rgb[i,0]} {rgb[i,1]} {rgb[i,2]}\n")
    print(f"저장: {path}")


def visualize_blend(points: np.ndarray,
                    anchor_positions: list[np.ndarray],
                    inner_radius: float = 0.3,
                    outer_radius: float = 1.2) -> None:
    plt.rcParams['font.family'] = 'Malgun Gothic'
    sample = points[:5000]
    alphas = np.array([
        past_scene_alpha(pt[:3], anchor_positions, inner_radius, outer_radius)
        for pt in sample
    ])
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(sample[:, 0], sample[:, 2], sample[:, 1],
                    c=alphas, cmap="RdBu_r", s=0.5, alpha=0.6)
    plt.colorbar(sc, label="past scene alpha (1=과거, 0=현재)")
    ax.set_xlabel("X"); ax.set_ylabel("Z (깊이)"); ax.set_zlabel("Y")
    ax.set_title("공간 블렌딩 알파 분포")
    out = os.path.join(OUTPUT_DIR, "blend_alpha.png")
    plt.savefig(out, dpi=150)
    print(f"저장: {out}")
    plt.show()


def apply_keyboard_move(vc) -> None:
    if keyboard.is_pressed('w'):  vc.scale(1.1)
    if keyboard.is_pressed('s'):  vc.scale(0.9)
    if keyboard.is_pressed('a'):  vc.translate(-30, 0)
    if keyboard.is_pressed('d'):  vc.translate(30, 0)
    if keyboard.is_pressed('up'): vc.translate(0, -30)
    if keyboard.is_pressed('down'): vc.translate(0, 30)


def main():
    global latest_depth_result
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    pipe = load_model()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"웹캠 열기 실패 (CAMERA_INDEX={CAMERA_INDEX})")
        return

    ret, probe = cap.read()
    orig_h, orig_w = probe.shape[:2]
    depth_placeholder = np.zeros((orig_h, orig_w, 3), dtype=np.uint8)

    saved_index = [0]

    def get_ply_files():
        return sorted(f for f in os.listdir(OUTPUT_DIR) if f.endswith(".ply"))

    def load_ply_to_viewer(vis, pcd_ref):
        files = get_ply_files()
        if not files:
            print("저장된 파일 없음"); return
        idx = saved_index[0] % len(files)
        path = os.path.join(OUTPUT_DIR, files[idx])
        print(f"[{idx+1}/{len(files)}] {files[idx]}")
        pcd = o3d.io.read_point_cloud(path)
        if pcd_ref[0] is not None:
            vis.remove_geometry(pcd_ref[0], reset_bounding_box=False)
        vis.add_geometry(pcd, reset_bounding_box=False)
        pcd_ref[0] = pcd

    current_pcd_ref = [None]

    def make_cycle_callback(delta):
        def callback(vis, _key, action):
            if action != 1: return False
            saved_index[0] += delta
            load_ply_to_viewer(vis, current_pcd_ref)
            return False
        return callback

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window("3D Point Cloud", width=960, height=720)
    vis.register_key_action_callback(ord('['), make_cycle_callback(-1))
    vis.register_key_action_callback(ord(']'), make_cycle_callback(+1))

    # 깊이 intrinsics (축소된 DEPTH_W × DEPTH_H 기준)
    fx_d, fy_d, cx_d, cy_d = estimate_intrinsics(DEPTH_W, DEPTH_H)

    depth_thread: threading.Thread | None = None
    current_pcd   = None
    latest_points = None

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # ── 카메라 + depth placeholder 항상 동일 크기로 표시 ─────────────
        cv2.imshow("Camera + Depth", np.hstack([frame, depth_placeholder]))

        # ── 백그라운드 depth 스레드 관리 ─────────────────────────────────
        if depth_thread is None or not depth_thread.is_alive():
            rgb_small = cv2.resize(frame, (DEPTH_W, DEPTH_H))
            rgb_small = cv2.cvtColor(rgb_small, cv2.COLOR_BGR2RGB)
            rgb_small = rgb_small.astype(np.float32) / 255.0
            depth_thread = threading.Thread(
                target=depth_worker, args=(pipe, rgb_small, frame.copy()),
                daemon=True
            )
            depth_thread.start()

        # ── 완료된 depth 결과 처리 (메인 스레드에서만 뷰어 조작) ─────────
        with depth_lock:
            result = latest_depth_result
            latest_depth_result = None

        if result is not None:
            depth, saved_frame = result

            rgb_small_f = cv2.resize(
                cv2.cvtColor(saved_frame, cv2.COLOR_BGR2RGB), (DEPTH_W, DEPTH_H)
            ).astype(np.float32) / 255.0

            points = unproject(rgb_small_f, depth, fx_d, fy_d, cx_d, cy_d)
            points = downsample_voxel(points, VOXEL_SIZE)
            latest_points = points

            # depth placeholder 갱신 (창 크기 고정 유지)
            depth_norm = cv2.normalize(depth, None, 0, 255,
                                       cv2.NORM_MINMAX).astype(np.uint8)
            depth_vis  = cv2.applyColorMap(depth_norm, cv2.COLORMAP_MAGMA)
            depth_placeholder[:] = cv2.resize(depth_vis, (orig_w, orig_h))

            # 뷰어 업데이트 (단순 교체)
            pcd = make_o3d_pointcloud(points)
            if current_pcd is not None:
                vis.remove_geometry(current_pcd, reset_bounding_box=False)
            vis.add_geometry(pcd, reset_bounding_box=(current_pcd is None))
            current_pcd = pcd
            current_pcd_ref[0] = pcd

        vis.poll_events()
        apply_keyboard_move(vis.get_view_control())
        vis.update_renderer()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and latest_points is not None:
            from datetime import datetime
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            ply_path = os.path.join(OUTPUT_DIR, f"scene_{ts}.ply")
            save_ply(latest_points, ply_path)
            files = get_ply_files()
            print("── 저장된 파일 목록 ──")
            for i, f in enumerate(files):
                print(f"  [{i+1}] {f}")
            print("[ / ] 키로 파일 선택")
            centroid = latest_points[:, :3].mean(axis=0)
            visualize_blend(latest_points, [centroid],
                            inner_radius=3.0, outer_radius=10.0)
        elif key == ord('q'):
            break

    cap.release()
    vis.destroy_window()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()