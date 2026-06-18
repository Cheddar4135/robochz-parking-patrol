# Distribution_Warehouse 시각 에셋 복원

용량 문제로 **시각 메시/텍스처는 git에서 제외**되어 있다(`.gitignore`).
`model.sdf`의 inline collision 박스로 **물리/LiDAR/맵/순찰은 그대로 동작**하며,
아래 에셋은 Gazebo에서 창고를 **보이게** 할 때만 필요하다.

## 제외된 파일
- `base_visual.dae` (83MB)
- `base_visual_texture_*.jpg` (27장)

## 복원 방법
1. Fuel에서 모델 다운로드: https://fuel.gazebosim.org/1.0/OpenRobotics/models/Distribution_Warehouse
   → `base_visual.glb` 확보.
2. Gazebo Classic은 .glb 렌더 불가 → `assimp`로 변환:
   ```bash
   assimp export base_visual.glb base_visual.dae
   ```
   (텍스처 .jpg 들이 같은 폴더에 함께 추출됨)
3. `model.sdf` 의 visual `<uri>base_visual.dae</uri>` 와 일치하면 끝.
