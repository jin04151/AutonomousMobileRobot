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

    # 전역 경로 재계획 상태
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
                    left_angle,
                    right_angle,
                    gyro_values,
                    acceleration,
                    north,
                    now
                )

                # 2. A: Mapping
                grid.update(pose, lidar_points)
                safe_grid = grid.make_safe_grid(SAFETY_CLEARANCE_M)

                if now >= next_map_time:
                    grid.save_png(map_path)
                    write_log(f't={now:.2f}s map_saved={map_path}')
                    next_map_time += 5.0

                # 3. B: 타겟 검출 및 탐색 목표 선택
                detection = perception.detect(
                    camera_image,
                    camera.getWidth(),
                    camera.getHeight()
                )

                if detection is not None:
                    goal = perception.locate_target(
                        detection,
                        pose,
                        lidar_points
                    )
                else:
                    goal = exploration.choose_goal(
                        grid,
                        safe_grid,
                        pose
                    )

                # TODO(통합): 목표 유지·방문 목록·시작점 복귀 상태를 관리한다.

                # 4. C: 전역 경로 생성
                if goal is None:
                    current_path = None
                    planned_goal = None

                else:
                    # 기존 경로를 만들 때 사용했던 goal과
                    # 현재 goal이 달라졌는지 확인한다.
                    goal_changed = (
                        planned_goal is None
                        or goal != planned_goal
                    )

                    # 최신 safe_grid에서 현재 경로가
                    # 더 이상 통과 가능한지 확인한다.
                    path_blocked = (
                        current_path is not None
                        and global_planning.path_is_blocked(
                            current_path,
                            safe_grid
                        )
                    )

                    need_replan = (
                        current_path is None
                        or goal_changed
                        or path_blocked
                    )

                    if need_replan:
                        # goal 변경 또는 기존 경로 차단은 즉시 재계획한다.
                        immediate_replan = (
                            goal_changed
                            or path_blocked
                        )

                        # 이전 A*가 실패해서 current_path가 None인 경우에는
                        # 매 32 ms마다 반복하지 않고 일정 간격으로 재시도한다.
                        if (
                            immediate_replan
                            or now - last_plan_time >= REPLAN_RETRY_INTERVAL
                        ):
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
                        pose,
                        path,
                        lidar_points
                    )

                    if state != 'MOVING':
                        linear, angular = 0.0, 0.0

                front_distance = (
                    local_planning.nearest_front_obstacle(
                        lidar_points
                    )
                )

                # 6. 모터 명령
                left_speed, right_speed = (
                    local_planning.wheel_speeds(
                        linear,
                        angular
                    )
                )

                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)

                # 7. 시뮬레이션 시간 1초 간격 로그
                if now >= next_log_time:
                    distance_text = (
                        'clear'
                        if math.isinf(front_distance)
                        else f'{front_distance:.2f}m'
                    )

                    write_log(
                        f't={now:.2f}s state={state} '
                        f'front={distance_text} '
                        f'pose=({pose.x:+.2f}, {pose.y:+.2f}, '
                        f'{math.degrees(pose.theta):+.1f}deg)'
                    )

                    next_log_time += 1.0

        finally:
            # 종료 시 마지막 지도도 같은 파일에 저장한다.
            grid.save_png(map_path)

            write_log(
                f't={robot.getTime():.2f}s '
                f'map_saved={map_path} final'
            )


if __name__ == '__main__':
    main()
