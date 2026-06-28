# PoC — 카메라→3D 파이프라인 검증

## 실행 방법

```bash
# 1. 의존성 설치
pip install -r poc/requirements_poc.txt

# 2. 테스트 이미지 배치
#    poc/sample/image.jpg 에 임의의 사진 1장 넣기

# 3. 실행
python poc/pipeline.py
```

## 출력

| 파일 | 내용 |
|------|------|
| `poc/output/scene.ply` | 3D 포인트 클라우드 — CloudCompare로 열기 |
| `poc/output/blend_alpha.png` | 공간 블렌딩 알파 분포 시각화 |

## PoC가 증명하는 것

1. RGB 이미지 → 단안 깊이 추정 → 역투영 파이프라인 작동 여부
2. 포인트 클라우드 zstd 압축률 (목표: 원본의 50% 이하)
3. 공간 블렌딩 알파가 앵커 거리에 따라 올바르게 분포하는지
4. .ply로 저장된 씬이 시각적으로 의미있는 3D 구조를 가지는지

## 서비스 통합 판단 기준

- [ ] 압축 후 씬 1개당 10MB 이하
- [ ] 포인트 클라우드가 실제 씬을 알아볼 수 있는 수준으로 복원됨
- [ ] 블렌딩 경계가 자연스러움 (CloudCompare 시각화 기준)
- [ ] 파이프라인 전체 실행 시간 3초 이하 (모델 로드 제외)
