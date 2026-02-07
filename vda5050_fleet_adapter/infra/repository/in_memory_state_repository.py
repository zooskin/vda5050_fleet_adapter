"""인메모리 AGV 상태 저장소 구현체."""

import threading

from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.entities.order import Order
from vda5050_fleet_adapter.domain.enums import ConnectionState
from vda5050_fleet_adapter.usecase.ports.state_repository import StateRepository


class InMemoryStateRepository(StateRepository):
    """StateRepository의 인메모리 구현체.

    dict 기반으로 AGV 상태를 메모리에 저장한다.
    모든 접근은 Lock으로 스레드 안전성을 보장한다.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._states: dict[str, AgvState] = {}
        self._orders: dict[str, Order] = {}
        self._connections: dict[str, ConnectionState] = {}

    # -- AGV 상태 --

    def get_state(self, agv_id: str) -> AgvState | None:
        with self._lock:
            return self._states.get(agv_id)

    def save_state(self, agv_id: str, state: AgvState) -> None:
        with self._lock:
            self._states[agv_id] = state

    def get_all_agv_ids(self) -> list[str]:
        with self._lock:
            return list(self._states.keys())

    # -- 주문 --

    def get_current_order(self, agv_id: str) -> Order | None:
        with self._lock:
            return self._orders.get(agv_id)

    def save_order(self, agv_id: str, order: Order) -> None:
        with self._lock:
            self._orders[agv_id] = order

    def clear_order(self, agv_id: str) -> None:
        with self._lock:
            self._orders.pop(agv_id, None)

    # -- 연결 상태 --

    def get_connection_state(self, agv_id: str) -> ConnectionState:
        with self._lock:
            return self._connections.get(agv_id, ConnectionState.OFFLINE)

    def save_connection_state(
        self, agv_id: str, state: ConnectionState
    ) -> None:
        with self._lock:
            self._connections[agv_id] = state
