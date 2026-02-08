"""TaskTracker 포트 인터페이스.

RMF Task의 최종 목적지를 조회하기 위한 추상 인터페이스.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class TaskTracker(ABC):
    """Task 최종 목적지 추적 인터페이스.

    /task_api_requests와 /task_api_responses ROS 2 토픽을
    통해 Task의 최종 목적지를 파악한다.
    """

    @abstractmethod
    def get_final_destination(self, booking_id: str) -> str | None:
        """Task의 최종 목적지 waypoint 이름을 반환한다.

        Args:
            booking_id: RMF booking ID (current_task_id() 반환값).

        Returns:
            최종 목적지 waypoint 이름, 또는 아직 매핑 전이면 None.
        """

    @abstractmethod
    def clear_booking(self, booking_id: str) -> None:
        """완료된 task의 캐시를 정리한다.

        Args:
            booking_id: 정리할 booking ID.
        """
