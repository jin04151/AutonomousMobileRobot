"""C 담당: 전역 경로 계획 인터페이스."""


def plan(grid, safe_grid, pose, goal) -> list[tuple[float, float]] | None:
    """다음 경유지부터 목적지까지의 지도 좌표(m) 목록을 반환한다.

    None은 경로 실패, 빈 목록은 이미 도착한 상태이다.
    """
    # TODO(C): A* 구현. 물체 중심이 막힌 셀이면 안전한 접근 지점으로 계획한다.
    return None
