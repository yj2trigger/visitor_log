# visitor_log

VPS·AR 앵커 기능의 씬 저장·서빙 서비스.  
Android(ARCore)가 촬영한 depth + RGB + pose를 WebSocket으로 노트북/서버에 스트리밍하고,  
포인트 클라우드로 변환·누적·압축해 나중에 AR 렌더링에 재사용한다.

---

## 아키텍처 개요

```
[S24 Flutter App]  →(WebSocket)→  [노트북/서버 Python]  →(저장)→  [visitor_log API]
  ARCore depth                       unproject                        zstd 압축 저장
  ARCore RGB                         pose 변환                        앵커별 씬 관리
  ARCore Pose                        누적 + 다운샘플
  Camera Intrinsics                  Open3D 시각화
```

---

## 시퀀스 다이어그램

```mermaid
sequenceDiagram
    participant App as S24 Flutter (ARCore)
    participant WS  as WebSocket Server (노트북)
    participant PC  as PointCloudProcessor
    participant Viz as Open3D Visualizer

    App->>WS: WebSocket 연결

    Note over App: ARSession 시작

    loop 5fps throttle (매 6번째 ARFrame만 전송)
        App->>App: ARFrame.acquireDepthImage()<br/>→ Float32 depth (160×120, 미터)
        App->>App: ARFrame.acquireImage()<br/>→ RGB (640×480)
        App->>App: ARFrame.camera.getPose()<br/>→ 4×4 camera-to-world 행렬
        App->>App: camera.getImageIntrinsics()<br/>→ fx,fy,cx,cy (depth 해상도 기준)

        Note over App,WS: depth + RGB + pose + intrinsics 원자적 번들링<br/>(분리 전송 시 pose-frame 불일치 발생)

        App->>WS: Binary 프레임 전송<br/>{header(48B): point_count·intrinsics·pose[16]}<br/>{depth_bytes: float32×160×120}<br/>{rgb_bytes: uint8×160×120×3 (리사이즈)}

        alt 이전 프레임 처리 중
            Note over WS: 프레임 드롭 (큐 누적 방지)
        else 처리 가능
            WS->>PC: decode_frame(data)
            PC->>PC: depth·RGB를 동일 해상도(160×120)로 정렬
            PC->>PC: unproject(rgb, depth, intrinsics)<br/>→ (N,6) [x,y,z,r,g,b] 카메라 공간
            PC->>PC: pose @ xyz_homogeneous<br/>→ 월드 공간 변환
            PC->>PC: downsample_voxel(voxel=0.02m)
            PC->>Viz: 포인트 클라우드 교체
            Viz->>Viz: render
        end
    end

    Note over App,WS: 앵커 작성 완료 시
    App->>WS: POST /scenes {anchor_id, pose, intrinsics, point_count}
    App->>WS: Binary points_bin (zstd 압축)
    WS->>WS: 저장 (visitor_log DB)
    WS-->>App: {scene_id, storage_bytes}
```

---

## 클래스 다이어그램

```mermaid
classDiagram
    class CameraIntrinsics {
        +float fx
        +float fy
        +float cx
        +float cy
        +int width
        +int height
    }

    class ARFrame {
        +ndarray depth_m        %%  float32 (H×W), 미터
        +ndarray rgb            %%  uint8 (H×W×3)
        +ndarray pose           %%  float32 (4×4) camera-to-world
        +CameraIntrinsics intrinsics
        +from_bytes(data) ARFrame$
    }

    class PointCloudProcessor {
        -float voxel_size
        +process(frame ARFrame) ndarray
        -unproject(rgb, depth, intrinsics) ndarray
        -transform_to_world(points, pose) ndarray
        -downsample(points) ndarray
    }

    class WebSocketReceiver {
        -str host
        -int port
        -bool processing
        -PointCloudProcessor processor
        -Open3DVisualizer visualizer
        +start()
        -on_message(data bytes)
        -drop_if_busy(data bytes) bool
    }

    class Open3DVisualizer {
        -PointCloud current_pcd
        -VisualizerWithKeyCallback vis
        +update(points ndarray)
        +apply_keyboard_move()
        +run_once()
    }

    class SceneStorage {
        +save(anchor_id, points, pose, intrinsics) int
        +load(anchor_id) ARFrame
        -compress(points ndarray) bytes
        -decompress(data bytes, count int) ndarray
    }

    class ARCoreStreamer {
        <<Flutter - Android>>
        -ArSession session
        -WebSocketChannel ws
        -int frameCount
        -int THROTTLE_N = 6
        +startSession()
        +onFrame(frame)
        -shouldSend() bool
        -bundleFrame(frame) Uint8List
        -sendFrame(bundle)
    }

    ARFrame --> CameraIntrinsics
    PointCloudProcessor ..> ARFrame : uses
    WebSocketReceiver --> PointCloudProcessor
    WebSocketReceiver --> Open3DVisualizer
    WebSocketReceiver ..> ARFrame : decodes
    SceneStorage ..> ARFrame : stores
    ARCoreStreamer ..> ARFrame : creates
```

---

## 비판적 설계 결정

| 결정 | 이유 |
|------|------|
| 5fps throttle (매 6번째 프레임) | 30fps × 75KB/frame = 2.25MB/s 초과. WiFi 안정성 고려 5fps(~375KB/s)로 제한 |
| depth + RGB를 동일 해상도(160×120)로 정렬 | ARCore depth(160×120) ≠ RGB(640×480). 역투영 전 RGB를 depth 크기로 리사이즈 후 전송 |
| Pose + frame 원자적 번들링 | 분리 전송 시 네트워크 지연으로 pose-frame 불일치 → 포인트 위치 오류 |
| 처리 중 신규 프레임 드롭 | 큐 누적 시 메모리 폭발 + 지연 누적. 최신 프레임만 처리 |
| 카메라 공간 → 월드 공간 변환 | ARCore Pose가 절대 world 좌표계 제공 → DEPTH_SCALE 불필요, 누적 오차 없음 |

---

## 실행 방법

### spike (노트북 웹캠, ARCore 없이 파이프라인 구조 검증)

```bash
cd visitor_log
py -3.12 -m venv .venv
.venv\Scripts\activate
pip install -r spike/requirements_poc.txt opencv-python open3d keyboard
python spike/realtime.py
```

### 서비스 전환 (예정)

1. S24 Flutter ARCore 앱 → WebSocket 스트리밍
2. `python server.py` (WebSocket 수신 + 포인트 클라우드 처리)
3. `visitor_log` FastAPI 서버 → 씬 저장·서빙

---

## 현재 상태

| 컴포넌트 | 상태 |
|---------|------|
| spike/realtime.py (웹캠) | ✅ 작동 — 구조 검증 완료 |
| ARCoreStreamer (Flutter) | ⏳ 미구현 |
| WebSocketReceiver | ⏳ 미구현 |
| SceneStorage (FastAPI) | ⏳ 미구현 |
