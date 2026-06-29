# viewer.py — 저장된 .ply 포인트 클라우드 뷰어
#
# [실행 방법]
#     .venv\Scripts\activate
#     python spike/viewer.py
#
# [조작]
#     [ / ]       : 이전/다음 파일
#     W / X       : 앞/뒤 (줌)
#     A / D       : 좌/우
#     ↑ / ↓       : 위/아래
#     마우스 드래그 : 시점 회전
#     스크롤       : 줌
#     q           : 종료

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import keyboard
import open3d as o3d

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def apply_keyboard_move(vc) -> None:
    if keyboard.is_pressed('w'):    vc.scale(-1)
    if keyboard.is_pressed('x'):    vc.scale(1)
    if keyboard.is_pressed('a'):    vc.rotate(10, 0)
    if keyboard.is_pressed('d'):    vc.rotate(-10, 0)
    if keyboard.is_pressed('left'): vc.translate(30, 0)
    if keyboard.is_pressed('right'):vc.translate(-30, 0)
    if keyboard.is_pressed('up'):   vc.translate(0, 30)
    if keyboard.is_pressed('down'): vc.translate(0, -30)


def load_and_show(vis, files: list[str], index: int,
                  pcd_ref: list) -> None:
    path = os.path.join(OUTPUT_DIR, files[index % len(files)])
    pcd  = o3d.io.read_point_cloud(path)
    if pcd_ref[0] is not None:
        vis.remove_geometry(pcd_ref[0], reset_bounding_box=False)
    vis.add_geometry(pcd, reset_bounding_box=(pcd_ref[0] is None))
    pcd_ref[0] = pcd
    print(f"[{index % len(files) + 1}/{len(files)}] {files[index % len(files)]}")


def main() -> None:
    files = sorted(f for f in os.listdir(OUTPUT_DIR) if f.endswith(".ply")) \
            if os.path.isdir(OUTPUT_DIR) else []

    if not files:
        print(f"저장된 .ply 파일 없음 ({OUTPUT_DIR})")
        print("realtime.py에서 s 키로 먼저 씬을 저장하세요.")
        return

    print("── 저장된 파일 목록 ──")
    for i, f in enumerate(files):
        print(f"  [{i+1}] {f}")
    print()

    vis = o3d.visualization.Visualizer()
    vis.create_window("Point Cloud Viewer", width=960, height=720)

    index    = [0]
    pcd_ref  = [None]
    load_and_show(vis, files, index[0], pcd_ref)

    prev_keys = {'[': False, ']': False, 'q': False}

    while True:
        apply_keyboard_move(vis.get_view_control())
        vis.poll_events()
        vis.update_renderer()

        cur = {k: keyboard.is_pressed(k) for k in prev_keys}

        if cur['['] and not prev_keys['[']:
            index[0] -= 1
            load_and_show(vis, files, index[0], pcd_ref)

        if cur[']'] and not prev_keys[']']:
            index[0] += 1
            load_and_show(vis, files, index[0], pcd_ref)

        if cur['q'] and not prev_keys['q']:
            break

        prev_keys = cur

    vis.destroy_window()


if __name__ == "__main__":
    main()
