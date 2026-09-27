"""바퀴 엔코더로 로봇의 위치와 방향을 추정한다."""

from dataclasses import dataclass
import math


WHEEL_RADIUS = 0.075
WHEEL_BASE = 0.30


def wrap_angle(angle):
    """각도를 -π 이상 π 미만으로 맞춘다."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class Localization:
    """좌우 바퀴 엔코더로 차동 구동 로봇의 자세를 계산한다."""

    def __init__(self, initial_pose=None):
        # 제공받은 시작 자세를 복사한다. 생략하면 출발점 기준 좌표를 사용한다.
        start = initial_pose if initial_pose is not None else Pose()
        self.pose = Pose(start.x, start.y, start.theta)
        self.previous_left = None
        self.previous_right = None
        self.last_update_time = None

        # 진단을 위해 각 센서의 최근 원본 값을 보관한다.
        self.gyro = (0.0, 0.0, 0.0)
        self.acceleration = (0.0, 0.0, 0.0)
        self.compass = (0.0, 0.0, 0.0)

    def update(self, left_angle, right_angle, gyro, acceleration, compass, now):
        """센서값을 받아 자세를 갱신한다. 위치는 m, 방향은 rad 단위다."""
        # 방향 센서의 축·부호·잡음·영점을 확인할 때까지 원본 값만 보관한다.
        # TODO: 보정 후 자이로·가속도계·나침반 값을 위치 추정에 융합한다.
        self.gyro = tuple(gyro)
        self.acceleration = tuple(acceleration)
        self.compass = tuple(compass)
        self.last_update_time = now

        if self.previous_left is None:
            self.previous_left = left_angle
            self.previous_right = right_angle
            return self.pose

        left_distance = (left_angle - self.previous_left) * WHEEL_RADIUS
        right_distance = (right_angle - self.previous_right) * WHEEL_RADIUS
        distance = 0.5 * (left_distance + right_distance)
        turn = (right_distance - left_distance) / WHEEL_BASE

        middle_heading = self.pose.theta + 0.5 * turn
        self.pose.x += distance * math.cos(middle_heading)
        self.pose.y += distance * math.sin(middle_heading)
        self.pose.theta = wrap_angle(self.pose.theta + turn)

        self.previous_left = left_angle
        self.previous_right = right_angle
        return self.pose
