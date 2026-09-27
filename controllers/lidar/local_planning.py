"""Decide immediate motion from nearby LiDAR points."""

import math


WHEEL_RADIUS = 0.075
WHEEL_BASE = 0.30
FORWARD_SPEED = 0.15
TURN_SPEED = 0.7
OBSTACLE_DISTANCE = 0.60
ROBOT_HALF_WIDTH = 0.18
LIDAR_HEIGHT = 0.45


def nearest_front_obstacle(points):
    nearest = math.inf

    for point in points:
        if not all(math.isfinite(value) for value in
                   (point.x, point.y, point.z)):
            continue

        height = LIDAR_HEIGHT + point.z
        if (0.0 < point.x < nearest and
                abs(point.y) < ROBOT_HALF_WIDTH and
                0.12 < height < 1.40):
            nearest = point.x

    return nearest


def choose_motion(points):
    """Return (linear speed, angular speed, state, front distance)."""
    front_distance = nearest_front_obstacle(points)

    if front_distance < OBSTACLE_DISTANCE:
        return 0.0, TURN_SPEED, 'TURN', front_distance

    return FORWARD_SPEED, 0.0, 'FORWARD', front_distance


def wheel_speeds(linear, angular):
    """Convert robot speed to left and right wheel speeds."""
    left = (linear - angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS
    right = (linear + angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS
    return left, right


def command(pose, path, lidar_points) -> tuple[float, float, str]:
    """자세·지도 좌표 경로·LiDAR 점으로 (v, ω, status)를 반환한다.

    v는 m/s, ω는 rad/s이다. 상태는 MOVING, ARRIVED, BLOCKED 중 하나이다.
    """
    if path is None:
        return 0.0, 0.0, 'BLOCKED'
    if not path:
        return 0.0, 0.0, 'ARRIVED'
    # TODO(C): 경로 추종과 충돌 회피 구현 전까지 정지한다.
    return 0.0, 0.0, 'BLOCKED'
