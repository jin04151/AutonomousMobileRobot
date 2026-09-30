"""C 담당: 전역 경로 계획 인터페이스."""

import heapq
import math


# Tie-break에 사용할 장애물 탐색 반경.
# 실제 path cost에는 들어가지 않으며,
# 동일한 f-score 후보 중 더 여유 있는 셀을 고르는 용도이다.
CLEARANCE_SEARCH_RADIUS = 6

SQRT2 = math.sqrt(2.0)
EPS = 1e-9


# (row 변화량, col 변화량, 이동 비용)
MOVES = (
    (-1,  0, 1.0),
    ( 1,  0, 1.0),
    ( 0, -1, 1.0),
    ( 0,  1, 1.0),

    (-1, -1, SQRT2),
    (-1,  1, SQRT2),
    ( 1, -1, SQRT2),
    ( 1,  1, SQRT2),
)


def _in_bounds(shape, cell):
    """cell이 grid 범위 안에 있는지 확인한다."""
    row, col = cell

    return (
        0 <= row < shape[0]
        and 0 <= col < shape[1]
    )


def _is_blocked(safe_grid, cell):
    """safe_grid에서 통행 금지 셀인지 확인한다."""
    row, col = cell
    return bool(safe_grid[row, col])


def _heuristic(a, b):
    """8방향 이동에 맞는 Octile Distance를 계산한다."""

    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])

    diagonal = min(dr, dc)
    straight = max(dr, dc) - diagonal

    return diagonal * SQRT2 + straight


def _clearance(grid, cell, cache):
    """가장 가까운 Occupied 셀까지의 거리를 계산한다.

    이 값은 A*의 실제 path cost에 더하지 않는다.
    f-score가 같은 후보들의 tie-break 용도로만 사용한다.

    Unknown(-1)은 장애물로 취급하지 않고,
    실제 Occupied(100)만 사용한다.
    """

    if cell in cache:
        return cache[cell]

    row, col = cell

    rows, cols = grid.data.shape

    radius = CLEARANCE_SEARCH_RADIUS
    nearest = math.inf

    row_min = max(0, row - radius)
    row_max = min(rows, row + radius + 1)

    col_min = max(0, col - radius)
    col_max = min(cols, col + radius + 1)

    for rr in range(row_min, row_max):
        for cc in range(col_min, col_max):

            # 실제 Occupied만 사용.
            # Unknown(-1)은 tie-break obstacle로 사용하지 않는다.
            if grid.data[rr, cc] != 100:
                continue

            distance = math.hypot(
                rr - row,
                cc - col,
            )

            if distance < nearest:
                nearest = distance

    # 탐색 반경 내에 장애물이 없다면
    # 충분히 멀리 떨어져 있다고 간주한다.
    if math.isinf(nearest):
        nearest = radius + 1.0

    cache[cell] = nearest

    return nearest


def _can_step(safe_grid, current, nxt):
    """current에서 nxt로 이동할 수 있는지 확인한다."""

    if not _in_bounds(
        safe_grid.shape,
        nxt,
    ):
        return False

    # safe_grid=True면 절대 통과하지 않는다.
    if _is_blocked(
        safe_grid,
        nxt,
    ):
        return False

    dr = nxt[0] - current[0]
    dc = nxt[1] - current[1]

    # 대각선 이동인 경우 corner cutting을 방지한다.
    if dr != 0 and dc != 0:

        side1 = (
            current[0] + dr,
            current[1],
        )

        side2 = (
            current[0],
            current[1] + dc,
        )

        if (
            _is_blocked(safe_grid, side1)
            or _is_blocked(safe_grid, side2)
        ):
            return False

    return True


def _reconstruct_path(came_from, current):
    """came_from을 따라 start부터 goal까지 경로를 복원한다."""

    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()

    return path


def _astar(
    grid,
    safe_grid,
    start,
    goals,
):
    """start에서 goals 중 하나까지 A*를 수행한다.

    실제 비용은 순수 이동 거리만 사용한다.

        f = g + h

    clearance는 f-score가 같은 경우의 tie-break에만 사용한다.
    """

    goal_set = set(goals)

    if not goal_set:
        return None

    if start in goal_set:
        return [start]

    def heuristic_to_goals(cell):
        return min(
            _heuristic(cell, goal)
            for goal in goal_set
        )

    clearance_cache = {}

    came_from = {}
    g_score = {
        start: 0.0,
    }

    open_heap = []

    push_id = 0

    start_h = heuristic_to_goals(start)
    start_clearance = _clearance(
        grid,
        start,
        clearance_cache,
    )

    # 우선순위:
    #
    # 1. f score가 작을수록 우선
    # 2. f가 같다면 clearance가 클수록 우선
    # 3. 그래도 같다면 h가 작을수록 우선
    #
    # heapq는 작은 값부터 꺼내므로 clearance에는 음수를 사용한다.
    heapq.heappush(
        open_heap,
        (
            start_h,
            -start_clearance,
            start_h,
            push_id,
            0.0,
            start,
        ),
    )

    closed = set()

    while open_heap:

        (
            _,
            _,
            _,
            _,
            current_g,
            current,
        ) = heapq.heappop(open_heap)

        if current in closed:
            continue

        # 오래된 heap entry 제거
        if current_g > g_score.get(
            current,
            math.inf,
        ) + EPS:
            continue

        if current in goal_set:
            return _reconstruct_path(
                came_from,
                current,
            )

        closed.add(current)

        for dr, dc, move_cost in MOVES:

            nxt = (
                current[0] + dr,
                current[1] + dc,
            )

            if not _can_step(
                safe_grid,
                current,
                nxt,
            ):
                continue

            tentative_g = (
                current_g
                + move_cost
            )

            old_g = g_score.get(
                nxt,
                math.inf,
            )

            if tentative_g + EPS >= old_g:
                continue

            came_from[nxt] = current
            g_score[nxt] = tentative_g

            h_score = heuristic_to_goals(nxt)

            f_score = (
                tentative_g
                + h_score
            )

            clearance = _clearance(
                grid,
                nxt,
                clearance_cache,
            )

            push_id += 1

            heapq.heappush(
                open_heap,
                (
                    f_score,
                    -clearance,
                    h_score,
                    push_id,
                    tentative_g,
                    nxt,
                ),
            )

    return None


def _ring_candidates(
    safe_grid,
    center,
    radius,
):
    """goal 주변 특정 반경의 통행 가능한 셀을 반환한다."""

    center_row, center_col = center

    candidates = []

    for row in range(
        center_row - radius,
        center_row + radius + 1,
    ):
        for col in range(
            center_col - radius,
            center_col + radius + 1,
        ):

            # 정사각형 ring의 바깥 테두리만 확인한다.
            if max(
                abs(row - center_row),
                abs(col - center_col),
            ) != radius:
                continue

            cell = (row, col)

            if not _in_bounds(
                safe_grid.shape,
                cell,
            ):
                continue

            if _is_blocked(
                safe_grid,
                cell,
            ):
                continue

            candidates.append(cell)

    return candidates


def _plan_to_goal_or_approach(
    grid,
    safe_grid,
    start,
    target,
):
    """목표 위치 또는 목표 주변의 안전 접근점까지 계획한다."""

    # 일반적인 frontier goal,
    # start position 등의 경우.
    if not _is_blocked(
        safe_grid,
        target,
    ):
        return _astar(
            grid,
            safe_grid,
            start,
            [target],
        )

    # 목표 물체 중심 등이 safe_grid에서 막혀 있다면
    # 가까운 ring부터 접근 가능한 후보를 찾는다.
    max_radius = max(
        safe_grid.shape
    )

    for radius in range(
        1,
        max_radius + 1,
    ):

        candidates = _ring_candidates(
            safe_grid,
            target,
            radius,
        )

        if not candidates:
            continue

        # 같은 radius에 여러 후보가 있으면
        # A*가 실제 이동 거리가 가장 짧은
        # 도달 가능한 후보를 선택한다.
        path = _astar(
            grid,
            safe_grid,
            start,
            candidates,
        )

        if path is not None:
            return path

    return None


def _bresenham_cells(start, end):
    """두 grid cell 사이 직선이 지나는 셀들을 반환한다."""

    row0, col0 = start
    row1, col1 = end

    dcol = abs(col1 - col0)
    drow = abs(row1 - row0)

    step_col = (
        1 if col0 < col1 else -1
    )

    step_row = (
        1 if row0 < row1 else -1
    )

    error = dcol - drow

    cells = []

    while True:

        cells.append(
            (row0, col0)
        )

        if (
            row0 == row1
            and col0 == col1
        ):
            break

        error2 = 2 * error

        if error2 > -drow:
            error -= drow
            col0 += step_col

        if error2 < dcol:
            error += dcol
            row0 += step_row

    return cells


def _line_of_sight(
    safe_grid,
    start,
    end,
):
    """두 셀 사이를 직선으로 안전하게 이동할 수 있는지 검사한다."""

    cells = _bresenham_cells(
        start,
        end,
    )

    previous = cells[0]

    for cell in cells:

        if _is_blocked(
            safe_grid,
            cell,
        ):
            return False

        dr = cell[0] - previous[0]
        dc = cell[1] - previous[1]

        # smoothing 과정에서도
        # corner cutting을 허용하지 않는다.
        if dr != 0 and dc != 0:

            side1 = (
                previous[0] + dr,
                previous[1],
            )

            side2 = (
                previous[0],
                previous[1] + dc,
            )

            if (
                _is_blocked(
                    safe_grid,
                    side1,
                )
                or
                _is_blocked(
                    safe_grid,
                    side2,
                )
            ):
                return False

        previous = cell

    return True


def _smooth_path(
    safe_grid,
    path,
):
    """직선 이동 가능한 구간의 중간 waypoint를 제거한다."""

    if (
        path is None
        or len(path) <= 2
    ):
        return path

    smoothed = [
        path[0]
    ]

    anchor = 0

    while anchor < len(path) - 1:

        next_index = anchor + 1

        # 현재 anchor에서
        # 직선 이동 가능한 가장 먼 waypoint를 찾는다.
        for candidate in range(
            len(path) - 1,
            anchor,
            -1,
        ):

            if _line_of_sight(
                safe_grid,
                path[anchor],
                path[candidate],
            ):
                next_index = candidate
                break

        smoothed.append(
            path[next_index]
        )

        anchor = next_index

    return smoothed

def path_is_blocked(grid, safe_grid, pose, path):
    """현재 위치부터 남은 path가 최신 safe_grid에서 막혔는지 확인한다."""

    if path is None:
        return True

    if not path:
        return False

    # 현재 로봇 위치를 grid 좌표로 변환
    current = grid.world_to_grid(
        pose.x,
        pose.y,
    )

    if current is None:
        return True

    current = tuple(current)

    if not _in_bounds(safe_grid.shape, current):
        return True

    # 현재 로봇 위치는 출발점이므로 통행 가능하게 취급
    checking_grid = safe_grid.copy()
    checking_grid[current[0], current[1]] = False

    previous = current

    # 현재 위치 -> 첫 waypoint
    # waypoint -> 다음 waypoint
    # 모든 구간을 검사
    for waypoint in path:

        waypoint_cell = grid.world_to_grid(
            waypoint[0],
            waypoint[1],
        )

        if waypoint_cell is None:
            return True

        waypoint_cell = tuple(waypoint_cell)

        if not _in_bounds(
            checking_grid.shape,
            waypoint_cell,
        ):
            return True

        # 기존에 있는 _line_of_sight() 재사용
        if not _line_of_sight(
            checking_grid,
            previous,
            waypoint_cell,
        ):
            return True

        previous = waypoint_cell

    return False

def plan(
    grid,
    safe_grid,
    pose,
    goal,
) -> list[tuple[float, float]] | None:
    """다음 경유지부터 목적지까지의 지도 좌표(m) 목록을 반환한다.

    None은 경로 실패,
    빈 목록은 이미 도착한 상태이다.
    """

    if goal is None:
        return None

    # grid와 safe_grid 크기가 다르면
    # 올바른 경로 계획이 불가능하다.
    if grid.data.shape != safe_grid.shape:
        return None

    start = grid.world_to_grid(
        pose.x,
        pose.y,
    )

    target = grid.world_to_grid(
        goal[0],
        goal[1],
    )

    if (
        start is None
        or target is None
    ):
        return None

    start = tuple(start)
    target = tuple(target)

    if not _in_bounds(
        safe_grid.shape,
        start,
    ):
        return None

    if not _in_bounds(
        safe_grid.shape,
        target,
    ):
        return None

    # 이미 같은 grid cell에 있다면 도착.
    if start == target:
        return []

    # 현재 로봇은 실제로 start 위치에 존재하므로
    # map noise 또는 inflation 때문에 자신의 셀이
    # blocked로 표시돼도 start만큼은 허용한다.
    planning_grid = safe_grid.copy()

    planning_grid[
        start[0],
        start[1],
    ] = False

    cell_path = _plan_to_goal_or_approach(
        grid,
        planning_grid,
        start,
        target,
    )

    if cell_path is None:
        return None

    # 불필요한 중간 waypoint 제거.
    cell_path = _smooth_path(
        planning_grid,
        cell_path,
    )

    # cell_path[0]은 현재 로봇 위치이므로 제외한다.
    world_path = []

    for row, col in cell_path[1:]:

        x, y = grid.grid_to_world(
            row,
            col,
        )

        world_path.append(
            (
                float(x),
                float(y),
            )
        )

    return world_path
