"""RMF Fleet 통신 포트 인터페이스.

Open-RMF Fleet Adapter와의 통신을 추상화한다.
infra/ros2/ 레이어에서 구현한다.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable

from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.value_objects.position import AgvPosition


class TaskHandle(ABC):
    """RMF 태스크 핸들 인터페이스.

    RMF로부터 할당받은 개별 태스크의 생명주기를 관리한다.
    """

    @abstractmethod
    def is_finished(self) -> bool:
        """태스크 완료 여부."""

    @abstractmethod
    def complete(self) -> None:
        """태스크를 정상 완료 처리한다."""

    @abstractmethod
    def fail(self, error_message: str) -> None:
        """태스크를 실패 처리한다.

        Args:
            error_message: 실패 사유.
        """


class NavigationHandle(ABC):
    """RMF 내비게이션 핸들 인터페이스.

    RMF로부터 할당받은 내비게이션 명령의 생명주기를 관리한다.
    """

    @abstractmethod
    def get_remaining_waypoints(self) -> list[tuple[float, float, float]]:
        """남은 경유지 목록을 반환한다.

        Returns:
            (x, y, theta) 튜플 리스트.
        """

    @abstractmethod
    def arrived_at_waypoint(self, waypoint_index: int) -> None:
        """경유지 도착을 RMF에 알린다.

        Args:
            waypoint_index: 도착한 경유지 인덱스.
        """

    @abstractmethod
    def complete(self) -> None:
        """내비게이션을 정상 완료 처리한다."""

    @abstractmethod
    def fail(self, error_message: str) -> None:
        """내비게이션을 실패 처리한다.

        Args:
            error_message: 실패 사유.
        """


class FleetGateway(ABC):
    """RMF Fleet Adapter와의 통신을 담당하는 포트.

    Open-RMF에 AGV 상태를 보고하고,
    RMF로부터 태스크/내비게이션 명령을 수신하는 인터페이스를 정의한다.
    """

    # -- 상태 보고 (Adapter → RMF) --

    @abstractmethod
    def update_position(
        self, agv_id: str, position: AgvPosition
    ) -> None:
        """AGV 위치를 RMF에 보고한다.

        Args:
            agv_id: AGV 식별자.
            position: 현재 위치.
        """

    @abstractmethod
    def update_battery(self, agv_id: str, battery_percent: float) -> None:
        """AGV 배터리 상태를 RMF에 보고한다.

        Args:
            agv_id: AGV 식별자.
            battery_percent: 충전율 (%).
        """

    @abstractmethod
    def update_state(self, agv_id: str, state: AgvState) -> None:
        """AGV 전체 상태를 RMF에 보고한다.

        Args:
            agv_id: AGV 식별자.
            state: AGV 전체 상태.
        """

    # -- 명령 수신 콜백 등록 (RMF → Adapter) --

    @abstractmethod
    def on_navigate(
        self,
        callback: Callable[[str, NavigationHandle], None],
    ) -> None:
        """RMF 내비게이션 명령 수신 콜백을 등록한다.

        Args:
            callback: 내비게이션 명령 수신 시 호출할 콜백 (agv_id, handle).
        """

    @abstractmethod
    def on_stop(
        self,
        callback: Callable[[str], None],
    ) -> None:
        """RMF 정지 명령 수신 콜백을 등록한다.

        Args:
            callback: 정지 명령 수신 시 호출할 콜백 (agv_id).
        """

    @abstractmethod
    def on_action(
        self,
        callback: Callable[[str, str, dict], None],
    ) -> None:
        """RMF 액션 명령 수신 콜백을 등록한다.

        Args:
            callback: 액션 명령 수신 시 호출할 콜백
                      (agv_id, action_type, parameters).
        """

    # -- Fleet 관리 --

    @abstractmethod
    def register_agv(self, agv_id: str) -> None:
        """AGV를 RMF Fleet에 등록한다.

        Args:
            agv_id: 등록할 AGV 식별자.
        """

    @abstractmethod
    def start(self) -> None:
        """Fleet Adapter를 시작한다."""

    @abstractmethod
    def shutdown(self) -> None:
        """Fleet Adapter를 종료한다."""
