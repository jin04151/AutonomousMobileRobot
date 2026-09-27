## 우리가 구현할 것

`[x]`는 구현함, `[ ]`는 아직 구현하지 않음이다. 일부만 구현한 항목은 하위 기능별로 표시했다.

1. localization : 로봇의 현재 위치를 계속 추정하는 기능(Differential Drive, 질문 답변에서 IMU 기능 구성 개별 센서들을 준다고 했으니 이걸 이용해서 방향 오차 보정) → 어느 정도 수준까지 localization을 할 것인가? (scan matching)
   1. [x] 바퀴 엔코더 기반 Differential Drive Odometry로 로봇 위치 `(x, y, θ)` 추정.
   2. [ ] 자이로·나침반의 축과 영점을 확인한 뒤 방향 오차 보정. 현재는 센서값을 읽고 저장하는 단계.
   4. [ ] 필요 시 EKF 또는 LiDAR Scan Matching으로 누적 오차 보정.
   5. [ ] GPS/WPS 등 절대 위치 센서가 제공·허용되는지 확인 후 사용 검토.
   참고: slam_toolbox의 Scan Matching 및 지도 생성 방식.

2. Mapping : 주변 환경 지도 만들기
   1. [x] localization에서 얻은 현재 위치랑 LiDAR 활용해서 지도 생성.
   2. [x] NumPy 기반 2D Occupancy Grid Map 구현.
   3. [x] LiDAR 좌표를 Map 좌표로 변환해 해당 격자 칸에 반영.
   4. [x] LiDAR 관측을 log-odds로 누적해 Free / Occupied / Unknown으로 관리.
   5. [ ] 로봇 크기와 안전거리를 반영해 장애물을 확장한 `safe_grid` 생성.
   참고: slam_toolbox의 지도 생성 구조, Nav2 Inflation Layer의 안전거리 처리 방식.

3. exploration(search)
   1. [ ] target 위치가 제공되지 않으므로 미탐색 공간을 로봇이 돌아다님.
   2. [ ] 목표가 어디 있는지 모르니까 후보 지점을 따라서 탐색하고 다음 후보 지점을 고름.
   3. 기본 구현
      - [ ] Frontier Detection: 확인된 Free 공간과 Unknown 공간의 경계 찾기.
      - [ ] 이동 가능한 Frontier 후보 추출.
      - [ ] 가장 가까운 Frontier 를 다음 목표로 선택.
   4. [ ] 이동 거리가 짧고 새로운 영역을 많이 볼 수 있는 곳으로 이동하도록 알고리즘 선택.
4. object detection : 목표 물체를 찾았는가?
   1. [ ] camera 이미지에서 target 찾기. Camera 장착·활성화까지만 완료.
   2. [ ] 규칙 기반 CV 또는 딥러닝 모델.
   3. [ ] 타겟이 어떤 건지 보고 결정. 답변상 단순한 작업이므로 OpenCV부터 검토.
   4. [ ] 미리 구현한다면 인터페이스를 동일하게 만들고 두 방식 모두 시도.
5. target localization
   1. [ ] Object Detection으로 발견한 타겟의 방향·거리 추정.
   2. [ ] 방향·거리와 현재 로봇 위치를 이용해 타겟의 지도상 좌표 계산.
   참고: [Autoware의 카메라·LiDAR 투영 방식]
6. global path planning : 목적지까지 경로 생성
   1. [ ] baseline은 A\*로 구현.
   2. 기본 구현
      - [ ] Occupancy Grid 기반 A\*.
      - [ ] 8방향 이동.
      - [ ] 장애물 회피.
      - [ ] A\* 경로에서 안전하게 보이는 앞쪽 추종점 생성.
      - [ ] Obstacle Inflation이 적용된 safe_grid에서 A* 실행.
      - [ ] 목표가 바뀌거나 현재 경로가 막혔을 때 새 경로 생성.
7. local planning & control - 안전하게 실제 이동하는 것
   1. [ ] 충돌 회피
   2. [ ] 장애물이 감지되면 감속하다가 특정거리 이내로 들어가면 정지하는 방식? -
   3. [ ] 장애물이 지나가지 않는 경우를 고려해서 진행경로가 막히는 경우 멈추고 다시 replanning 하는 것까지
   4. [ ] 필요하면 DWA 검토.
   참고: Nav2 Regulated Pure Pursuit의 경로 추종 방식, 필요 시 DWB Controller의 회피 방식.
8. 전체 연결 - main 
    1. [x] 센서 → 위치 추정 → 지도 → 검출/탐색 → 전역 경로 → 주행 명령의 인터페이스 연결.
    2. [ ] 대회 mission flow: 미탐색 공간 탐색 → 모든 target 식별·방문 → 시작점 복귀.

## 팀 작업용 인터페이스

현재는 기본틀까지 연결한 상태다. 탐색·검출·경로 계획 함수는 `None`을 반환하므로 로봇은 `WAIT_GOAL` 상태로 정지한다. 센서 읽기, 지도 갱신, 1초 간격 콘솔·파일 로그, 5초 간격 `maps/map.png` 덮어쓰기와 종료 시 저장은 계속한다. 시간은 시뮬레이션 기준이다.

| 함수 | 입력 | 출력 |
| --- | --- | --- |
| `Localization.update(left_angle, right_angle, gyro, acceleration, compass, now)` | 바퀴 각도(rad), 센서값, 시간(s) | `Pose(x, y, theta)` |
| `grid.update(pose, lidar_points)` | 자세, LiDAR 점 | `grid.data` 갱신 |
| `grid.make_safe_grid(clearance_m)` | 로봇 외곽에 더할 안전거리(m) | bool 배열, `True`는 통행 불가이며 Unknown 포함 |
| `perception.detect(camera_image, width, height)` | Webots 원본 이미지 버퍼, 크기(px) | `Detection` 또는 `None` |
| `perception.locate_target(detection, pose, lidar_points)` | 검출 결과, 자세, LiDAR 점 | 목표물 좌표 `(x, y)` 또는 `None` |
| `exploration.choose_goal(grid, safe_grid, pose)` | 지도, 안전 지도, 자세 | 탐색 목표 `(x, y)` 또는 `None` |
| `global_planning.plan(grid, safe_grid, pose, goal)` | 지도, 안전 지도, 자세, 목표 | 다음 경유지부터의 `(x, y)` 목록 또는 `None` |
| `local_planning.command(pose, path, lidar_points)` | 자세, 경로, LiDAR 점 | `(v, omega, status)` |

### 공통 규칙

1. 위치·목표·경로는 미터 단위의 같은 월드 좌표계를 사용한다. `theta`는 rad, `omega`는 rad/s, `v`는 m/s이다.
2. 격자는 `grid.data[row, col]`로 접근하고, 좌표 변환은 `grid.world_to_grid()` / `grid.grid_to_world()`을 사용한다.
3. `goal`이나 `path`가 `None`이면 목표가 없거나 경로를 찾지 못한 상태다. 메인은 모터를 정지하고 다음 목표 또는 재계획을 기다린다.

좌표 설정: 현재 `main.START_POSE = None`이면 기존처럼 출발점을 `(0, 0, 0)`으로 둔 상대 좌표를 사용한다. Webots 월드 좌표와 맞추려면 확인된 시작 위치·방향을 `Pose(x, y, theta)`로 지정해야 한다. 다른 모듈도 같은 좌표계를 사용한다.

### 아직 비어 있는 부분

- `make_safe_grid()`는 구현 전 임시로 모든 칸을 `True`로 반환한다. 실제 장애물 팽창은 A 담당이다.
- `Detection` 내부 필드는 perception에서 해석한다. 다른 모듈은 결과를 그대로 전달한다.
- 경로는 시작점을 제외한 경유지 목록이다. `[]`는 도착, `None`은 경로 실패이다. 물체 중심이 장애물 칸이면 전역 계획에서 안전한 접근 지점까지 계획해야 한다.
- `command()`의 상태는 `MOVING`, `ARRIVED`, `BLOCKED`이다. 현재는 빈 경로에 `ARRIVED`, 나머지에 `BLOCKED`와 속도 0을 반환한다. 메인은 `MOVING`일 때만 이동 명령을 전달한다.
- 기존 `choose_motion()`은 수동 주행 시험용으로 남겼으며 메인에서 호출하지 않는다.
- 현재 메인은 매 반복마다 목표 선택·경로 함수를 호출한다. 목표 유지, 방문 목록, 복귀, 재계획 주기는 통합 담당의 TODO다.

### 파일 담당

| 담당 | 수정할 파일 | 구현 범위 |
| --- | --- | --- |
| A | `localization.py`, `mapping.py` | 위치 추정, 지도, 안전 지도 |
| B | `perception.py`, `exploration.py` | 검출, 목표물 위치 추정, 탐색 목표 선택 |
| C | `global_planning.py`, `local_planning.py` | A*, 경로 추종, 충돌 회피, 속도 명령 |

이 기본틀을 공통 기준으로 먼저 공유한 뒤 각자 담당 파일을 작업한다. 
