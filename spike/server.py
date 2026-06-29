# server.py — ARCore WebSocket 수신 서버
#
# [실행 방법]
#     .venv\Scripts\activate
#     python spike/server.py
#
# [연결 방법]
#     Flutter 앱에서 ws://<노트북_IP>:8765 로 연결
#     무선 디버깅: 같은 WiFi 필요
#
# [조작] (창 포커스 무관)
#     W/S         : 앞/뒤
#     A/D         : 좌/우
#     ↑/↓         : 상/하
#     마우스 드래그 : 시점 회전
#     s           : 현재 씬 .ply 저장
#     q           : 종료
#
# [binary 프레임 포맷 — Flutter 앱과 공유]
#     header (96 bytes):
#       depth_w   int32  4B
#       depth_h   int32  4B
#       rgb_w     int32  4B   ← depth와 같은 값 (Flutter에서 리사이즈)
#       rgb_h     int32  4B
#       fx        float32 4B
#       fy        float32 4B
#       cx        float32 4B
#       cy        float32 4B
#       pose[16]  float32×16 = 64B  (4×4 camera-to-world, column-major)
#     depth_bytes: float32 × depth_w × depth_h
#     rgb_bytes:   uint8  × depth_w × depth_h × 3

import asyncio
import os
import queue
import struct
import sys
import threading
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import cv2
import keyboard
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import websockets

from app.processing.blend_zones import past_scene_alpha

# ── 설정 ────────────────────────────────────────────────────────────────────
HOST       = "0.0.0.0"
PORT       = 8765
VOXEL_SIZE = 0.02
MAX_DEPTH  = 8.191  # ARCore DEPTH16 최대값 (13비트 = 8191mm, 신뢰도 비트 제거 후)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")
HEADER_FMT = "<4i 4f 16f"   # little-endian: 4×int32, 4×float32, 16×float32
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # = 96 bytes (4×4 + 4×4 + 16×4)

# ── 스레드 공유 큐 ────────────────────────────────────────────────────────────
frame_queue  = queue.Queue(maxsize=1)   # WebSocket → 처리 스레드
result_queue = queue.Queue(maxsize=1)   # 처리 스레드 → 메인 스레드


# ── 프레임 파싱 ──────────────────────────────────────────────────────────────

def decode_frame(data: bytes):
    """binary 프레임 → (depth, rgb, pose, fx, fy, cx, cy)

    depth: float32 (H, W) — ARCore 실측 미터값
    rgb:   float32 (H, W, 3) — 0~1 정규화
    pose:  float32 (4, 4) — camera-to-world
    """
    if len(data) < HEADER_SIZE:
        raise ValueError(f"data too short: {len(data)} < {HEADER_SIZE}")

    fields = struct.unpack_from(HEADER_FMT, data, 0)
    depth_w, depth_h, rgb_w, rgb_h = fields[:4]
    fx, fy, cx, cy = fields[4:8]
    pose = np.array(fields[8:24], dtype=np.float32).reshape(4, 4)

    offset = HEADER_SIZE
    depth_bytes = depth_w * depth_h * 4   # float32
    rgb_bytes   = depth_w * depth_h * 3   # uint8

    depth = np.frombuffer(data[offset: offset + depth_bytes],
                          dtype='<f4').reshape(depth_h, depth_w).copy()
    rgb   = np.frombuffer(data[offset + depth_bytes: offset + depth_bytes + rgb_bytes],
                          dtype=np.uint8).reshape(depth_h, depth_w, 3)
    valid = int(((depth > 0) & (depth <= MAX_DEPTH)).sum())
    print(f"depth: {depth.min():.3f}~{depth.max():.3f}m  유효: {valid}/{depth_w*depth_h}  총크기: {len(data)}")
    rgb   = rgb.astype(np.float32) / 255.0

    return depth, rgb, pose, fx, fy, cx, cy


# ── 역투영 ────────────────────────────────────────────────────────────────────

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
    return np.concatenate([xyz, rgb[mask]], axis=-1).astype(np.float32)


# ── 처리 스레드 ───────────────────────────────────────────────────────────────

def process_worker(stop_event: threading.Event) -> None:
    """frame_queue에서 프레임 꺼내 unproject + 월드 변환 후 누적, result_queue에 저장."""
    from app.processing.pointcloud import downsample_voxel
    accumulated: np.ndarray | None = None

    while not stop_event.is_set():
        try:
            depth, rgb, pose, fx, fy, cx, cy = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        points = unproject(rgb, depth, fx, fy, cx, cy)
        if len(points) == 0:
            continue

        ones      = np.ones((len(points), 1), dtype=np.float32)
        xyz_world = (pose @ np.hstack([points[:, :3], ones]).T).T[:, :3]
        points_world = np.hstack([xyz_world, points[:, 3:]])

        # 누적 + 다운샘플
        if accumulated is None:
            accumulated = points_world
        else:
            accumulated = np.vstack([accumulated, points_world])
        accumulated = downsample_voxel(accumulated, VOXEL_SIZE)

        if len(accumulated) == 0:
            continue

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(accumulated[:, :3])
        pcd.colors = o3d.utility.Vector3dVector(accumulated[:, 3:])

        print(f"pcd: {len(pcd.points)} points → queue")
        try:
            result_queue.put_nowait(pcd)
        except queue.Full:
            pass   # 메인 스레드가 아직 이전 결과 처리 중이면 드롭


# ── WebSocket 핸들러 ──────────────────────────────────────────────────────────

async def handle_client(websocket) -> None:
    addr = websocket.remote_address
    print(f"[연결] {addr}")
    try:
        async for data in websocket:
            if isinstance(data, bytes):
                try:
                    decoded = decode_frame(data)
                    try:
                        frame_queue.put_nowait(decoded)
                    except queue.Full:
                        pass   # 처리 중 — 드롭
                except ValueError as e:
                    print(f"[파싱 오류] {e}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        print(f"[해제] {addr}")


# ── 유틸 ─────────────────────────────────────────────────────────────────────

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


def visualize_blend(points: np.ndarray) -> None:
    plt.rcParams['font.family'] = 'Malgun Gothic'
    sample  = points[:5000]
    centroid = sample[:, :3].mean(axis=0)
    alphas  = np.array([
        past_scene_alpha(pt[:3], [centroid], 3.0, 10.0) for pt in sample
    ])
    fig = plt.figure(figsize=(10, 7))
    ax  = fig.add_subplot(111, projection="3d")
    sc  = ax.scatter(sample[:, 0], sample[:, 2], sample[:, 1],
                     c=alphas, cmap="RdBu_r", s=0.5, alpha=0.6)
    plt.colorbar(sc, label="past scene alpha")
    ax.set_title("공간 블렌딩 알파 분포")
    out = os.path.join(OUTPUT_DIR, "blend_alpha.png")
    plt.savefig(out, dpi=150)
    plt.show()


def apply_keyboard_move(vc) -> None:
    if keyboard.is_pressed('w'):     vc.scale(-2)
    if keyboard.is_pressed('x'):     vc.scale(2)
    if keyboard.is_pressed('a'):     vc.rotate(-10, 0)
    if keyboard.is_pressed('d'):     vc.rotate(10, 0)
    if keyboard.is_pressed('left'):  vc.translate(-30, 0)
    if keyboard.is_pressed('right'): vc.translate(30, 0)
    if keyboard.is_pressed('up'):    vc.translate(0, -30)
    if keyboard.is_pressed('down'):  vc.translate(0, 30)


# ── 메인 ─────────────────────────────────────────────────────────────────────

def run_websocket_server(stop_event: threading.Event) -> None:
    """WebSocket 서버를 별도 스레드의 asyncio 루프에서 실행."""
    async def _serve():
        async with websockets.serve(handle_client, HOST, PORT):
            print(f"[서버] ws://{HOST}:{PORT} 대기 중...")
            while not stop_event.is_set():
                await asyncio.sleep(0.1)

    asyncio.run(_serve())


def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    stop_event = threading.Event()

    # 처리 스레드 시작
    worker = threading.Thread(target=process_worker, args=(stop_event,), daemon=True)
    worker.start()

    # WebSocket 서버 스레드 시작
    ws_thread = threading.Thread(target=run_websocket_server, args=(stop_event,), daemon=True)
    ws_thread.start()

    # Open3D 뷰어
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window("3D Point Cloud (ARCore)", width=960, height=720)

    current_pcd   = None
    latest_points = None

    print("Flutter 앱 연결 대기 중... (ws://[노트북IP]:8765)")

    while True:
        # 처리 완료된 pcd 수신
        try:
            pcd = result_queue.get_nowait()
            print(f"메인: pcd {len(pcd.points)} points 수신")
            first = current_pcd is None
            if current_pcd is not None:
                vis.remove_geometry(current_pcd, reset_bounding_box=False)
            current_pcd = pcd
            vis.add_geometry(current_pcd, reset_bounding_box=first)

            latest_points = np.hstack([
                np.asarray(current_pcd.points, dtype=np.float32),
                np.asarray(current_pcd.colors, dtype=np.float32),
            ])
        except queue.Empty:
            pass

        apply_keyboard_move(vis.get_view_control())
        vis.poll_events()
        vis.update_renderer()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and latest_points is not None:
            from datetime import datetime
            ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_ply(latest_points, os.path.join(OUTPUT_DIR, f"scene_{ts}.ply"))
            visualize_blend(latest_points)
        elif key == ord('q'):
            break

    stop_event.set()
    vis.destroy_window()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
