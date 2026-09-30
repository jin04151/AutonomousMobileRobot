"""Webots entry point: read devices and apply motion commands."""

import math
import os
from datetime import datetime
from pathlib import Path

from controller import Robot
import exploration
import global_planning
import local_planning
import perception
from localization import Localization, Pose
from mapping import OccupancyGrid


TIME_STEP = 32
# TODO(통합): 제공된 월드 시작 자세를 Pose(x, y, theta)로 지정한다.
START_POSE = None
# TODO(통합): 안전 지도 구현 후 로봇 외곽의 안전거리(m)를 결정한다.
SAFETY_CLEARANCE_M = 0.0
# 확인된 보정/대응 기준을 제공하기 전에는 위치 추정과 목표 등록을 대기한다.
PERCEPTION_CALIBRATION = None
PERCEPTION_ASSOCIATION = None
TARGET_MATCH_DISTANCE_M = None
# 카메라가 매 TIME_STEP에 갱신되고 now가 영상 관측 시각임을 확인한 뒤 활성화.
CAMERA_TIME_CONFIRMED = False


def main():
    robot = Robot()

    lidar = robot.getDevice('lidar')
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()

    camera = robot.getDevice('camera')
    camera.enable(TIME_STEP)

    gyro = robot.getDevice('gyro')
    accelerometer = robot.getDevice('accelerometer')
    compass = robot.getDevice('compass')
    for sensor in (gyro, accelerometer, compass):
        sensor.enable(TIME_STEP)

    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')
    left_encoder.enable(TIME_STEP)
    right_encoder.enable(TIME_STEP)

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    for motor in (left_motor, right_motor):
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    localization = Localization(initial_pose=START_POSE)
    grid = OccupancyGrid(rows=201, cols=201)
    selector = exploration.FrontierSelector()
    diagnostics = exploration.Diagnostics()
    registry = (perception.TargetRegistry(TARGET_MATCH_DISTANCE_M)
                if TARGET_MATCH_DISTANCE_M is not None else None)
    pose = localization.pose
    controller_dir = Path(__file__).resolve().parent
    maps_dir = controller_dir / 'maps'
    logs_dir = controller_dir / 'logs'
    maps_dir.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)
    map_path = maps_dir / 'map.png'
    log_name = f"run_{datetime.now():%Y%m%d_%H%M%S_%f}_{os.getpid()}.log"
    log_path = logs_dir / log_name
    next_map_time = 5.0
    next_log_time = 0.0

    # C: 전역 경로 재계획 상태
    current_path = None
    planned_goal = None
    last_plan_time = -math.inf
    REPLAN_RETRY_INTERVAL = 0.5

    with log_path.open('w', encoding='utf-8', buffering=1) as log_file:
        def write_log(message):
            """콘솔과 실행 로그에 같은 메시지를 기록한다."""
            print(message, flush=True)
            print(message, file=log_file, flush=True)

        try:
            while robot.step(TIME_STEP) != -1:
                now = robot.getTime()

                # 0. 센서 읽기
                lidar_points = lidar.getPointCloud() or []
                camera_image = camera.getImage()
                gyro_values = gyro.getValues()
                acceleration = accelerometer.getValues()
                north = compass.getValues()
                left_angle = left_encoder.getValue()
                right_angle = right_encoder.getValue()

                # 1. A: Localization
                pose = localization.update(
                    left_angle, right_angle,
                    gyro_values, acceleration, north, now)

                # 2. A: Mapping
                grid.update(pose, lidar_points)
                safe_grid = grid.make_safe_grid(SAFETY_CLEARANCE_M)
                # 3. B: 타겟 검출 및 탐색 목표 선택
                image = perception.camera_image_to_bgr(
                    camera_image, camera.getWidth(), camera.getHeight())
                detections = [] if image is None else perception.detect_targets(image)
                located_count = 0
                if PERCEPTION_CALIBRATION is None or PERCEPTION_ASSOCIATION is None:
                    perception_state = 'WAIT_CALIBRATION'
                elif not CAMERA_TIME_CONFIRMED:
                    perception_state = 'WAIT_CAMERA_TIME'
                else:
                    timing = perception.ObservationTimes(now, now, tuple(
                        getattr(point, 'time', math.nan) for point in lidar_points))
                    observations = [(detection, perception.locate_target(
                        detection, lidar_points, pose, PERCEPTION_CALIBRATION,
                        association=PERCEPTION_ASSOCIATION, timing=timing)) for detection in detections]
                    located_count = sum(position is not None for _, position in observations)
                    perception_state = 'READY' if registry is not None else 'WAIT_REGISTRY_CONFIG'
                    if registry is not None:
                        registry.update(observations, now)
                # 물체 목록은 관측만 연결. 접근 목표·방문 조건 확정 후 임무 전환을 추가한다.
                goal = selector.choose(grid, safe_grid, pose, now, allow_switch=False)

                # 4. C: 전역 경로 생성
                if goal is None:
                    current_path = None
                    planned_goal = None

                else:
                    # 처음 goal이 생겼거나 이전 goal과 달라진 경우
                    goal_changed = (
                        planned_goal is None
                        or goal != planned_goal
                    )

                    # 기존 경로가 최신 safe_grid에서 막혔는지 확인
                    path_blocked = (
                        current_path is not None
                        and global_planning.path_is_blocked(
                            grid,
                            safe_grid,
                            pose,
                            current_path
                        )
                    )

                    need_replan = (
                        current_path is None
                        or goal_changed
                        or path_blocked
                    )

                    if need_replan:
                        # goal 변경 또는 path 차단은 즉시 재계획
                        immediate_replan = (
                            goal_changed
                            or path_blocked
                        )

                        # 이전 planning이 실패했다면
                        # 매 loop가 아니라 0.5초마다 재시도
                        retry_ready = (
                            now - last_plan_time
                            >= REPLAN_RETRY_INTERVAL
                        )

                        if immediate_replan or retry_ready:
                            current_path = global_planning.plan(
                                grid,
                                safe_grid,
                                pose,
                                goal
                            )

                            planned_goal = goal
                            last_plan_time = now

                path = current_path

                # 5. C: 근거리 주행 및 충돌 회피
                linear, angular = 0.0, 0.0
                if goal is None:
                    state = 'WAIT_GOAL'
                elif path is None:
                    state = 'WAIT_PATH'
                else:
                    linear, angular, state = local_planning.command(
                        pose, path, lidar_points)
                    if state != 'MOVING':
                        linear, angular = 0.0, 0.0

                front_distance = local_planning.nearest_front_obstacle(
                    lidar_points)

                # 6. 모터 명령
                left_speed, right_speed = local_planning.wheel_speeds(
                    linear, angular)
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)

                # 7. 이동거리는 매 주기 누적하고 진단 로그는 내부에서 1초 간격으로 출력.
                diagnostics.update(now, grid, safe_grid, pose, selector.goal, write_log, selector=selector)
                if now >= next_log_time:
                    distance_text = (
                        'clear' if math.isinf(front_distance)
                        else f'{front_distance:.2f}m'
                    )

                    write_log(
                        f't={now:.2f}s state={state} '
                        f'front={distance_text} '
                        f'pose=({pose.x:+.2f}, {pose.y:+.2f}, '
                        f'{math.degrees(pose.theta):+.1f}deg) '
                        f'goal={selector.goal} detections={len(detections)} located={located_count} '
                        f'targets={len(registry.targets) if registry is not None else 0} '
                        f'perception={perception_state}'
                    )

                    next_log_time += 1.0
                if now >= next_map_time:
                    diagnostics.save_png(map_path, grid, pose)
                    write_log(f't={now:.2f}s map_saved={map_path}')
                    next_map_time += 5.0

        finally:
            # 종료 시 마지막 지도도 같은 파일에 저장한다.
            if diagnostics.snapshot is not None:
                diagnostics.update(robot.getTime(), grid, safe_grid, pose, selector.goal, write_log, selector=selector)
                diagnostics.save_png(map_path, grid, pose)
            else:
                grid.save_png(map_path)
            write_log(
                f't={robot.getTime():.2f}s '
                f'map_saved={map_path} final'
            )


if __name__ == '__main__':
    main()
