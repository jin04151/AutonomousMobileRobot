"""Decide immediate motion from global path and nearby LiDAR points."""

import math


# ============================================================
# Robot parameters
# ============================================================

WHEEL_RADIUS = 0.075
WHEEL_BASE = 0.30

# 기존 starter code의 최대 주행 속도/회전 속도 유지
FORWARD_SPEED = 0.15
MIN_FORWARD_SPEED = 0.04
TURN_SPEED = 0.7

ROBOT_HALF_WIDTH = 0.18
LIDAR_HEIGHT = 0.45


# ============================================================
# Path following parameters
# ============================================================

# 중간 waypoint에 이 정도 가까워지면 다음 waypoint로 넘어간다.
WAYPOINT_REACHED_DISTANCE = 0.20

# 최종 goal 도착 판정 거리
GOAL_REACHED_DISTANCE = 0.15

# 목표 방향과 이 각도 이상 틀어져 있으면 전진하지 않고 먼저 회전
TURN_IN_PLACE_ANGLE = math.radians(45.0)

# heading error에 대한 비례 제어 gain
HEADING_KP = 1.4


# ============================================================
# Obstacle avoidance parameters
# ============================================================

# 이 거리보다 가까이 앞을 막으면 local avoidance 시작
OBSTACLE_DISTANCE = 0.60

# 이 거리 이내면 매우 가까운 상황 -> 전진하지 않고 제자리 회전
EMERGENCY_DISTANCE = 0.35

# avoidance를 종료할 거리.
#
# 시작 조건(0.60 m)보다 크게 두어서
# LEFT -> NORMAL -> LEFT -> NORMAL 식으로
# 상태가 흔들리는 것을 막는다.
AVOID_EXIT_DISTANCE = 0.80

# 이 거리부터 장애물이 가까워질수록 전진 속도를 줄인다.
SLOWDOWN_DISTANCE = 1.00

# 장애물을 천천히 돌아나갈 때 사용하는 전진 속도
AVOID_FORWARD_SPEED = 0.04

# 정면 충돌 판단 폭.
# 실제 robot half width보다 약간 넓게 본다.
FRONT_CORRIDOR_HALF_WIDTH = ROBOT_HALF_WIDTH + 0.08


# ============================================================
# Avoidance direction scoring parameters
# ============================================================

# 왼쪽/오른쪽 공간을 판단할 LiDAR sector
SIDE_SECTOR_MIN_ANGLE = math.radians(20.0)
SIDE_SECTOR_MAX_ANGLE = math.radians(100.0)

# 공간이 너무 넓으면 무한대 대신 이 값까지만 score로 사용
CLEARANCE_CAP = 2.0

# Global path 방향을 회피 방향 선택에 얼마나 반영할지
PATH_DIRECTION_BIAS = 0.35

# 현재 선택한 회피 방향이 너무 좁을 때
# 반대쪽이 이만큼 더 넓으면 방향 변경
SIDE_SWITCH_MARGIN = 0.15


# ============================================================
# Local planner internal state
# ============================================================

# cached global path를 따라가기 위한 현재 waypoint index
_active_path_id = None
_waypoint_index = 0

# avoidance hysteresis 상태
_avoid_active = False

# +1 = LEFT
# -1 = RIGHT
#  0 = 선택 안 됨
_avoid_direction = 0


# ============================================================
# Utility
# ============================================================

def _clamp(value, low, high):
    return max(low, min(high, value))


def _normalize_angle(angle):
    """Angle을 -pi ~ +pi 범위로 정규화한다."""

    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def _distance_xy(x1, y1, x2, y2):
    return math.hypot(
        x2 - x1,
        y2 - y1,
    )


def _valid_lidar_point(point):
    """실제 주행에 사용할 LiDAR point인지 확인한다."""

    if not all(
        math.isfinite(value)
        for value in (
            point.x,
            point.y,
            point.z,
        )
    ):
        return False

    height = LIDAR_HEIGHT + point.z

    return (
        0.12 < height < 1.40
    )


# ============================================================
# LiDAR processing
# ============================================================

def nearest_front_obstacle(points):
    """로봇 정면 통로에서 가장 가까운 장애물 거리를 반환한다."""

    nearest = math.inf

    for point in points:

        if not _valid_lidar_point(point):
            continue

        # LiDAR local frame:
        #
        # +x = 앞
        # +y = 왼쪽
        if (
            0.0 < point.x < nearest
            and abs(point.y) < FRONT_CORRIDOR_HALF_WIDTH
        ):
            nearest = point.x

    return nearest


def _side_clearances(points):
    """로봇 왼쪽/오른쪽의 장애물 여유거리를 계산한다."""

    left = math.inf
    right = math.inf

    for point in points:

        if not _valid_lidar_point(point):
            continue

        # 뒤쪽 점은 회피 방향 판단에서 제외한다.
        if point.x <= 0.0:
            continue

        distance = math.hypot(
            point.x,
            point.y,
        )

        angle = math.atan2(
            point.y,
            point.x,
        )

        # 왼쪽 sector
        if (
            SIDE_SECTOR_MIN_ANGLE
            <= angle
            <= SIDE_SECTOR_MAX_ANGLE
        ):
            left = min(
                left,
                distance,
            )

        # 오른쪽 sector
        elif (
            -SIDE_SECTOR_MAX_ANGLE
            <= angle
            <= -SIDE_SECTOR_MIN_ANGLE
        ):
            right = min(
                right,
                distance,
            )

    return left, right


def _clearance_score(distance):
    """inf를 포함한 clearance를 score로 변환한다."""

    if math.isinf(distance):
        return CLEARANCE_CAP

    return min(
        distance,
        CLEARANCE_CAP,
    )


# ============================================================
# Path tracking
# ============================================================

def _reset_path_tracking():
    global _active_path_id
    global _waypoint_index

    _active_path_id = None
    _waypoint_index = 0


def _target_waypoint(pose, path):
    """현재 따라가야 할 waypoint를 선택한다.

    Returns:
        (waypoint, arrived)

    arrived가 True이면 최종 goal에 도착한 상태이다.
    """

    global _active_path_id
    global _waypoint_index

    path_id = id(path)

    # Global planner가 새 path를 만들었다면
    # waypoint를 처음부터 다시 추적한다.
    if path_id != _active_path_id:
        _active_path_id = path_id
        _waypoint_index = 0

    # --------------------------------------------------------
    # 최종 goal 도착 확인
    # --------------------------------------------------------

    goal_x, goal_y = path[-1]

    goal_distance = _distance_xy(
        pose.x,
        pose.y,
        goal_x,
        goal_y,
    )

    if goal_distance <= GOAL_REACHED_DISTANCE:
        return None, True

    # --------------------------------------------------------
    # 중간 waypoint progression
    # --------------------------------------------------------

    while _waypoint_index < len(path) - 1:

        waypoint_x, waypoint_y = path[
            _waypoint_index
        ]

        distance = _distance_xy(
            pose.x,
            pose.y,
            waypoint_x,
            waypoint_y,
        )

        # 아직 현재 waypoint에 충분히 가까워지지 않았다.
        if distance > WAYPOINT_REACHED_DISTANCE:
            break

        # 현재 waypoint 도착
        # -> 다음 waypoint로 진행
        _waypoint_index += 1

    return (
        path[_waypoint_index],
        False,
    )


# ============================================================
# Path-aware avoidance
# ============================================================

def _choose_avoid_direction(
    heading_error,
    left_clearance,
    right_clearance,
):
    """LiDAR 공간 + Global Path 방향을 이용해 회피 방향을 결정한다."""

    left_score = _clearance_score(
        left_clearance
    )

    right_score = _clearance_score(
        right_clearance
    )

    # --------------------------------------------------------
    # Global path direction bias
    # --------------------------------------------------------
    #
    # heading_error > 0
    # -> 현재 waypoint가 왼쪽 방향
    #
    # heading_error < 0
    # -> 현재 waypoint가 오른쪽 방향
    #
    # 단, path 방향은 보조 점수일 뿐이고
    # LiDAR 안전 공간이 기본 판단 기준이다.

    path_bias = min(
        abs(heading_error)
        / math.radians(90.0),
        1.0,
    ) * PATH_DIRECTION_BIAS

    if heading_error > 0.0:

        left_score += path_bias

    elif heading_error < 0.0:

        right_score += path_bias

    # +1 = LEFT
    # -1 = RIGHT

    if left_score >= right_score:
        return 1

    return -1


def _reset_avoidance():
    global _avoid_active
    global _avoid_direction

    _avoid_active = False
    _avoid_direction = 0


def _avoidance_command(
    heading_error,
    front_distance,
    left_clearance,
    right_clearance,
):
    """필요하면 LiDAR 기반 avoidance command를 반환한다.

    회피할 필요가 없으면 None을 반환한다.
    """

    global _avoid_active
    global _avoid_direction

    # --------------------------------------------------------
    # 1. Avoidance 시작
    # --------------------------------------------------------

    if (
        not _avoid_active
        and front_distance < OBSTACLE_DISTANCE
    ):

        _avoid_active = True

        _avoid_direction = (
            _choose_avoid_direction(
                heading_error,
                left_clearance,
                right_clearance,
            )
        )

    # 회피 상태가 아니면
    # 일반 path following으로 넘어간다.
    if not _avoid_active:
        return None

    # --------------------------------------------------------
    # 2. Avoidance 종료
    # --------------------------------------------------------
    #
    # 시작 거리:
    #   0.60 m
    #
    # 종료 거리:
    #   0.80 m
    #
    # 서로 다른 값을 사용하여 hysteresis 구현

    if front_distance >= AVOID_EXIT_DISTANCE:

        _reset_avoidance()

        return None

    # --------------------------------------------------------
    # 3. 현재 회피 방향이 갑자기 막힌 경우
    # --------------------------------------------------------

    if _avoid_direction > 0:

        chosen_clearance = left_clearance
        opposite_clearance = right_clearance

    else:

        chosen_clearance = right_clearance
        opposite_clearance = left_clearance

    if (
        chosen_clearance < EMERGENCY_DISTANCE
        and
        opposite_clearance
        > chosen_clearance + SIDE_SWITCH_MARGIN
    ):

        _avoid_direction *= -1

    # --------------------------------------------------------
    # 4. 앞/왼쪽/오른쪽이 전부 너무 가까운 경우
    # --------------------------------------------------------

    if (
        front_distance < EMERGENCY_DISTANCE
        and left_clearance < EMERGENCY_DISTANCE
        and right_clearance < EMERGENCY_DISTANCE
    ):

        return (
            0.0,
            0.0,
            'BLOCKED',
        )

    # --------------------------------------------------------
    # 5. 실제 회피 명령
    # --------------------------------------------------------

    angular = (
        _avoid_direction
        * TURN_SPEED
    )

    # 너무 가까우면 앞으로 가지 않고 제자리 회전
    if front_distance < EMERGENCY_DISTANCE:

        return (
            0.0,
            angular,
            'MOVING',
        )

    # 어느 정도 여유가 있으면
    # 천천히 전진하면서 장애물을 돌아간다.
    return (
        AVOID_FORWARD_SPEED,
        angular,
        'MOVING',
    )


# ============================================================
# Normal path following
# ============================================================

def _path_follow_command(
    heading_error,
    front_distance,
):
    """Waypoint 방향을 따라가는 기본 주행 명령을 계산한다."""

    abs_error = abs(
        heading_error
    )

    # --------------------------------------------------------
    # Angular velocity
    # --------------------------------------------------------

    angular = _clamp(
        HEADING_KP * heading_error,
        -TURN_SPEED,
        TURN_SPEED,
    )

    # 목표 방향과 많이 틀어져 있으면
    # 먼저 제자리에서 방향을 맞춘다.
    if abs_error >= TURN_IN_PLACE_ANGLE:

        return (
            0.0,
            angular,
            'MOVING',
        )

    # --------------------------------------------------------
    # Heading-based adaptive speed
    # --------------------------------------------------------
    #
    # heading_error가 작으면 빠르게,
    # 크면 전진 속도를 낮춘다.

    heading_factor = max(
        0.35,
        math.cos(abs_error),
    )

    # --------------------------------------------------------
    # Obstacle-based adaptive speed
    # --------------------------------------------------------

    if (
        math.isinf(front_distance)
        or front_distance >= SLOWDOWN_DISTANCE
    ):

        obstacle_factor = 1.0

    else:

        obstacle_factor = (
            front_distance
            - OBSTACLE_DISTANCE
        ) / (
            SLOWDOWN_DISTANCE
            - OBSTACLE_DISTANCE
        )

        obstacle_factor = _clamp(
            obstacle_factor,
            0.0,
            1.0,
        )

        # OBSTACLE_DISTANCE 이하는
        # 정상적으로 avoidance가 처리하지만
        # 경계 구간에서 갑자기 속도가 0이 되는 것을 방지한다.
        obstacle_factor = max(
            0.30,
            obstacle_factor,
        )

    linear = (
        FORWARD_SPEED
        * heading_factor
        * obstacle_factor
    )

    if linear > 0.0:

        linear = max(
            MIN_FORWARD_SPEED,
            linear,
        )

    return (
        linear,
        angular,
        'MOVING',
    )


# ============================================================
# Existing helper
# ============================================================

def choose_motion(points):
    """기존 starter code와 호환되는 LiDAR-only helper."""

    front_distance = nearest_front_obstacle(
        points
    )

    if front_distance < OBSTACLE_DISTANCE:

        return (
            0.0,
            TURN_SPEED,
            'TURN',
            front_distance,
        )

    return (
        FORWARD_SPEED,
        0.0,
        'FORWARD',
        front_distance,
    )


def wheel_speeds(linear, angular):
    """Convert robot speed to left and right wheel speeds."""

    left = (
        linear
        - angular * WHEEL_BASE / 2.0
    ) / WHEEL_RADIUS

    right = (
        linear
        + angular * WHEEL_BASE / 2.0
    ) / WHEEL_RADIUS

    return left, right


# ============================================================
# Main Local Planner interface
# ============================================================

def command(
    pose,
    path,
    lidar_points,
) -> tuple[float, float, str]:
    """자세·전역 경로·LiDAR를 이용하여 실제 이동 명령을 결정한다.

    Returns:
        (linear velocity, angular velocity, state)

    state:
        MOVING
        ARRIVED
        BLOCKED
    """

    # --------------------------------------------------------
    # 1. Path 존재 확인
    # --------------------------------------------------------

    if path is None:

        _reset_path_tracking()
        _reset_avoidance()

        return (
            0.0,
            0.0,
            'BLOCKED',
        )

    if not path:

        _reset_path_tracking()
        _reset_avoidance()

        return (
            0.0,
            0.0,
            'ARRIVED',
        )

    # --------------------------------------------------------
    # 2. 현재 따라갈 waypoint 선택
    # --------------------------------------------------------

    waypoint, arrived = _target_waypoint(
        pose,
        path,
    )

    if arrived:

        _reset_path_tracking()
        _reset_avoidance()

        return (
            0.0,
            0.0,
            'ARRIVED',
        )

    target_x, target_y = waypoint

    # --------------------------------------------------------
    # 3. Waypoint 방향 계산
    # --------------------------------------------------------

    desired_heading = math.atan2(
        target_y - pose.y,
        target_x - pose.x,
    )

    heading_error = _normalize_angle(
        desired_heading - pose.theta
    )

    # --------------------------------------------------------
    # 4. LiDAR 분석
    # --------------------------------------------------------

    front_distance = nearest_front_obstacle(
        lidar_points
    )

    left_clearance, right_clearance = (
        _side_clearances(
            lidar_points
        )
    )

    # --------------------------------------------------------
    # 5. Local obstacle avoidance
    # --------------------------------------------------------

    avoidance_command = (
        _avoidance_command(
            heading_error,
            front_distance,
            left_clearance,
            right_clearance,
        )
    )

    if avoidance_command is not None:
        return avoidance_command

    # --------------------------------------------------------
    # 6. Normal path following
    # --------------------------------------------------------

    return _path_follow_command(
        heading_error,
        front_distance,
    )
