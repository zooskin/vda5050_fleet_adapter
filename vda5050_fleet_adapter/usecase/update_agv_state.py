"""AGV 상태 업데이트 유스케이스.

AGV로부터 수신한 VDA5050 State 메시지를 처리하여
상태 저장소를 갱신하고 RMF에 보고한다.
"""

from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.entities.connection import Connection
from vda5050_fleet_adapter.domain.enums import ConnectionState
from vda5050_fleet_adapter.domain.events.agv_events import (
    ConnectionChangedEvent,
    NodeReachedEvent,
    OperatingModeChangedEvent,
)
from vda5050_fleet_adapter.usecase.ports.event_publisher import EventPublisher
from vda5050_fleet_adapter.usecase.ports.fleet_gateway import FleetGateway
from vda5050_fleet_adapter.usecase.ports.state_repository import StateRepository


class UpdateAgvState:
    """AGV 상태 업데이트 유스케이스.

    VDA5050 State 수신 → 저장소 갱신 → RMF 보고 → 이벤트 발행.

    Args:
        state_repo: 상태 저장소.
        fleet_gateway: RMF Fleet 통신 포트.
        event_publisher: 이벤트 발행자.
    """

    def __init__(
        self,
        state_repo: StateRepository,
        fleet_gateway: FleetGateway,
        event_publisher: EventPublisher,
    ) -> None:
        self._state_repo = state_repo
        self._fleet_gateway = fleet_gateway
        self._event_publisher = event_publisher

    def handle_state(self, agv_id: str, new_state: AgvState) -> None:
        """AGV State 메시지를 처리한다.

        Args:
            agv_id: AGV 식별자.
            new_state: 수신한 AGV 상태.
        """
        previous_state = self._state_repo.get_state(agv_id)
        self._state_repo.save_state(agv_id, new_state)

        # RMF에 위치/배터리 보고
        if new_state.agv_position is not None:
            self._fleet_gateway.update_position(
                agv_id, new_state.agv_position
            )

        if new_state.battery_state is not None:
            self._fleet_gateway.update_battery(
                agv_id, new_state.battery_state.battery_charge
            )

        self._fleet_gateway.update_state(agv_id, new_state)

        # 상태 변경 이벤트 발행
        self._detect_and_publish_events(agv_id, previous_state, new_state)

    def handle_connection(self, agv_id: str, connection: Connection) -> None:
        """AGV Connection 메시지를 처리한다.

        Args:
            agv_id: AGV 식별자.
            connection: 수신한 연결 상태.
        """
        previous = self._state_repo.get_connection_state(agv_id)
        new = connection.connection_state

        self._state_repo.save_connection_state(agv_id, new)

        if previous != new:
            self._event_publisher.publish(
                ConnectionChangedEvent(
                    agv_id=agv_id,
                    previous_state=previous,
                    new_state=new,
                )
            )

    def _detect_and_publish_events(
        self,
        agv_id: str,
        previous: AgvState | None,
        current: AgvState,
    ) -> None:
        """이전/현재 상태를 비교하여 도메인 이벤트를 발행한다."""
        if previous is None:
            return

        # 노드 통과 감지
        if current.last_node_id != previous.last_node_id:
            self._event_publisher.publish(
                NodeReachedEvent(
                    agv_id=agv_id,
                    node_id=current.last_node_id,
                    sequence_id=current.last_node_sequence_id,
                )
            )

        # 운영 모드 변경 감지
        if current.operating_mode != previous.operating_mode:
            self._event_publisher.publish(
                OperatingModeChangedEvent(
                    agv_id=agv_id,
                    previous_mode=previous.operating_mode,
                    new_mode=current.operating_mode,
                )
            )
