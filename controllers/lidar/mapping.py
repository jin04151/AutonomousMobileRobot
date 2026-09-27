"""LiDAR 점군을 2차원 점유 격자에 기록한다."""

import math

import numpy as np
from PIL import Image


UNKNOWN = -1
FREE = 0
OCCUPIED = 100
FREE_UPDATE = -0.35
OCCUPIED_UPDATE = 0.85
MIN_LOG_ODDS = -4.0
MAX_LOG_ODDS = 4.0
FREE_THRESHOLD = -0.7
OCCUPIED_THRESHOLD = 1.2

# 현재 월드의 LiDAR 위치와 장애물로 볼 높이 범위(단위: m).
LIDAR_X = 0.0
LIDAR_Y = 0.0
LIDAR_Z = 0.45
MIN_OBSTACLE_HEIGHT = 0.12
MAX_OBSTACLE_HEIGHT = 1.40
MIN_RANGE = 0.05


class OccupancyGrid:
    """Unknown 상태로 시작하는 2차원 점유 격자.

    월드 좌표의 단위는 m이고 격자 좌표의 단위는 칸이다.
    월드 원점 (0, 0)은 격자 중앙 셀에 놓인다.
    월드 +x는 열(col) 증가, +y는 행(row) 증가 방향이다.
    resolution은 한 셀의 한 변 길이(m/cell)이다.
    LiDAR 점은 로봇 기준 x=전방, y=좌측, z=위쪽으로 해석한다.
    """

    def __init__(self, rows=121, cols=121, resolution=0.1):
        if rows <= 0 or cols <= 0:
            raise ValueError("격자 크기는 1 이상이어야 합니다.")
        if resolution <= 0:
            raise ValueError("해상도는 0보다 커야 합니다.")

        self.rows = int(rows)
        self.cols = int(cols)
        self.resolution = float(resolution)
        self.origin_row = self.rows // 2
        self.origin_col = self.cols // 2
        self.data = np.full((self.rows, self.cols), UNKNOWN, dtype=np.int8)
        self.log_odds = np.zeros((self.rows, self.cols), dtype=np.float64)
        self.last_point_time = -math.inf

    def is_inside(self, row, col):
        """격자 좌표가 지도 안에 있는지 반환한다."""
        return 0 <= row < self.rows and 0 <= col < self.cols

    def world_to_grid(self, x, y):
        """월드 좌표(m)를 가장 가까운 격자 셀의 (row, col)로 변환한다.

        지도 밖의 좌표는 None을 반환한다.
        """
        col = self.origin_col + math.floor(float(x) / self.resolution + 0.5)
        row = self.origin_row + math.floor(float(y) / self.resolution + 0.5)
        return (row, col) if self.is_inside(row, col) else None

    def grid_to_world(self, row, col):
        """격자 좌표를 해당 셀 중심의 월드 좌표 (x, y)(m)로 변환한다.

        지도 밖의 좌표는 None을 반환한다.
        """
        if not self.is_inside(row, col):
            return None

        x = (col - self.origin_col) * self.resolution
        y = (row - self.origin_row) * self.resolution
        return x, y

    def make_safe_grid(self, clearance_m: float) -> np.ndarray:
        """True가 통행 불가인 bool 배열을 반환한다. Unknown도 차단한다.

        clearance_m은 로봇 외곽에 더할 안전거리(m)이다.
        """
        # TODO(A): 로봇 크기와 안전거리로 장애물을 팽창시킨다.
        # 구현 전에는 모든 셀을 차단한다.
        return np.ones(self.data.shape, dtype=bool)

    def save_png(self, path):
        """+x는 오른쪽, +y는 위쪽인 흑백 지도를 PNG로 저장한다."""
        pixels = np.full(self.data.shape, 128, dtype=np.uint8)
        pixels[self.data == FREE] = 255
        pixels[self.data == OCCUPIED] = 0
        Image.fromarray(np.flipud(pixels).copy()).save(path)

    def _ray_cells(self, start, end):
        """두 격자 셀을 잇는 광선의 셀을 차례로 반환한다."""
        row, col = start
        end_row, end_col = end
        row_step = 1 if row < end_row else -1
        col_step = 1 if col < end_col else -1
        row_distance = abs(end_row - row)
        col_distance = abs(end_col - col)
        error = col_distance - row_distance

        while True:
            yield row, col
            if (row, col) == end:
                break
            doubled_error = 2 * error
            if doubled_error > -row_distance:
                error -= row_distance
                col += col_step
            if doubled_error < col_distance:
                error += col_distance
                row += row_step

    def _clip_to_map(self, start_x, start_y, end_x, end_y):
        """지도 밖 끝점을 지도 경계의 마지막 셀로 제한한다."""
        dx = end_x - start_x
        dy = end_y - start_y
        x_min = -(self.origin_col + 0.5) * self.resolution
        x_max = (self.cols - self.origin_col - 0.5) * self.resolution
        y_min = -(self.origin_row + 0.5) * self.resolution
        y_max = (self.rows - self.origin_row - 0.5) * self.resolution
        fraction = 1.0

        if dx > 0:
            fraction = min(fraction, (x_max - start_x) / dx)
        elif dx < 0:
            fraction = min(fraction, (x_min - start_x) / dx)
        if dy > 0:
            fraction = min(fraction, (y_max - start_y) / dy)
        elif dy < 0:
            fraction = min(fraction, (y_min - start_y) / dy)

        x = start_x + fraction * dx
        y = start_y + fraction * dy
        col = self.origin_col + math.floor(x / self.resolution + 0.5)
        row = self.origin_row + math.floor(y / self.resolution + 0.5)
        return max(0, min(row, self.rows - 1)), max(0, min(col, self.cols - 1))

    def update(self, pose, lidar_points):
        """LiDAR 관측을 log-odds에 누적하고 셀 상태를 갱신한다.

        pose의 x, y는 m, theta는 rad이다. 점의 x, y, z도 m이다.
        관측하지 않은 셀은 Unknown으로 둔다.
        """
        x, y, theta = float(pose.x), float(pose.y), float(pose.theta)
        if not all(math.isfinite(value) for value in (x, y, theta)):
            raise ValueError("로봇 자세에 유효하지 않은 값이 있습니다.")

        cosine = math.cos(theta)
        sine = math.sin(theta)
        sensor_x = x + cosine * LIDAR_X - sine * LIDAR_Y
        sensor_y = y + sine * LIDAR_X + cosine * LIDAR_Y
        start = self.world_to_grid(sensor_x, sensor_y)
        if start is None:
            return self.data

        newest_point_time = self.last_point_time
        for point in lidar_points:
            try:
                local_x = float(point.x)
                local_y = float(point.y)
                local_z = float(point.z)
            except (AttributeError, TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in
                       (local_x, local_y, local_z)):
                continue

            # Webots가 이전 반복에서 반환한 같은 점은 다시 누적하지 않는다.
            point_time = getattr(point, 'time', None)
            if point_time is not None:
                try:
                    point_time = float(point_time)
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(point_time):
                    continue
                if point_time <= self.last_point_time + 1e-6:
                    continue

            height = LIDAR_Z + local_z
            if not MIN_OBSTACLE_HEIGHT <= height <= MAX_OBSTACLE_HEIGHT:
                continue
            if math.hypot(local_x, local_y) < MIN_RANGE:
                continue

            hit_x = sensor_x + cosine * local_x - sine * local_y
            hit_y = sensor_y + sine * local_x + cosine * local_y
            if not math.isfinite(hit_x) or not math.isfinite(hit_y):
                continue
            if point_time is not None:
                newest_point_time = max(newest_point_time, point_time)

            hit = self.world_to_grid(hit_x, hit_y)
            end = hit if hit is not None else self._clip_to_map(
                sensor_x, sensor_y, hit_x, hit_y)
            cells = list(self._ray_cells(start, end))

            # 통과한 셀은 Free, 점이 닿은 셀은 Occupied 증거를 누적한다.
            for row, col in cells[:-1]:
                self.log_odds[row, col] = max(
                    MIN_LOG_ODDS, self.log_odds[row, col] + FREE_UPDATE)

            row, col = cells[-1]
            if hit is not None:
                self.log_odds[row, col] = min(
                    MAX_LOG_ODDS, self.log_odds[row, col] + OCCUPIED_UPDATE)
            else:
                self.log_odds[row, col] = max(
                    MIN_LOG_ODDS, self.log_odds[row, col] + FREE_UPDATE)

        self.last_point_time = newest_point_time
        self.data.fill(UNKNOWN)
        self.data[self.log_odds <= FREE_THRESHOLD] = FREE
        self.data[self.log_odds >= OCCUPIED_THRESHOLD] = OCCUPIED
        return self.data
