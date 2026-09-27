"""B 담당: 목표물 검출과 위치 추정 인터페이스."""

from dataclasses import dataclass

from localization import Pose


@dataclass
class Detection:
    """화면 검출 결과. 내부 필드는 perception에서만 해석한다."""

    label: str
    center_px: tuple[float, float]
    bbox_xywh: tuple[int, int, int, int]
    image_size: tuple[int, int]


def detect(camera_image: bytes | None, width: int, height: int) -> Detection | None:
    """Webots 카메라 원본 버퍼와 크기(px)를 받아 검출 결과를 반환한다."""
    # TODO(B): 색상·형태 또는 모델로 목표물을 검출한다.
    return None


def locate_target(detection: Detection, pose: Pose, lidar_points) -> tuple[float, float] | None:
    """검출 결과와 LiDAR 점으로 목표물의 지도 좌표(x, y)(m)를 반환한다."""
    # TODO(B): 카메라 시야각·장착 위치를 확인하고 LiDAR 거리와 결합한다.
    return None
