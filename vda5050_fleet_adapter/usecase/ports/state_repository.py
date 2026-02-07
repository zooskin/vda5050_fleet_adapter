"""AGV 상태 저장소 포트 인터페이스.

AGV 상태의 저장/조회를 추상화한다.
인메모리 또는 외부 저장소로 구현 가능하다.
"""

from abc import ABC, abstractmethod

from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.entities.order import Order
from vda5050_fleet_adapter.domain.enums import ConnectionState


class StateRepository(ABC):
    """AGV 상태 저장소 인터페이스.

    각 AGV의 최신 상태, 주문, 연결 상태를 관리한다.
    """

    # -- AGV 상태 --

    @abstractmethod
    def get_state(self, agv_id: str) -> AgvState | None:
        """AGV의 최신 상태를 조회한다.

        Args:
            agv_id: AGV 식별자.

        Returns:
            최신 AgvState 또는 미등록 시 None.
        """

    @abstractmethod
    def save_state(self, agv_id: str, state: AgvState) -> None:
        """AGV 상태를 저장한다.

        Args:
            agv_id: AGV 식별자.
            state: 저장할 상태.
        """

    @abstractmethod
    def get_all_agv_ids(self) -> list[str]:
        """등록된 모든 AGV ID를 반환한다."""

    # -- 주문 --

    @abstractmethod
    def get_current_order(self, agv_id: str) -> Order | None:
        """AGV의 현재 실행 중인 주문을 조회한다.

        Args:
            agv_id: AGV 식별자.

        Returns:
            현재 Order 또는 없으면 None.
        """

    @abstractmethod
    def save_order(self, agv_id: str, order: Order) -> None:
        """AGV의 현재 주문을 저장한다.

        Args:
            agv_id: AGV 식별자.
            order: 저장할 주문.
        """

    @abstractmethod
    def clear_order(self, agv_id: str) -> None:
        """AGV의 현재 주문을 제거한다.

        Args:
            agv_id: AGV 식별자.
        """

    # -- 연결 상태 --

    @abstractmethod
    def get_connection_state(self, agv_id: str) -> ConnectionState:
        """AGV의 연결 상태를 조회한다.

        Args:
            agv_id: AGV 식별자.

        Returns:
            현재 연결 상태.
        """

    @abstractmethod
    def save_connection_state(
        self, agv_id: str, state: ConnectionState
    ) -> None:
        """AGV의 연결 상태를 저장한다.

        Args:
            agv_id: AGV 식별자.
            state: 연결 상태.
        """
