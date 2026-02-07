"""VDA5050 도메인 이벤트 정의.

도메인 레이어에서 발생하는 이벤트를 정의한다.
usecase/infra 레이어에서 이벤트를 구독하여 부가 로직을 처리한다.
"""

from dataclasses import dataclass, field
from datetime import UTC, datetime

from vda5050_fleet_adapter.domain.enums import (
    ActionStatus,
    ConnectionState,
    OperatingMode,
)


@dataclass(frozen=True)
class DomainEvent:
    """도메인 이벤트 기본 클래스.

    Args:
        timestamp: 이벤트 발생 시각 (UTC).
    """

    timestamp: datetime = field(default_factory=lambda: datetime.now(UTC))


@dataclass(frozen=True)
class OrderReceivedEvent(DomainEvent):
    """새 주문 수신 이벤트.

    Args:
        agv_id: AGV 식별자.
        order_id: 주문 ID.
        order_update_id: 주문 업데이트 번호.
    """

    agv_id: str = ""
    order_id: str = ""
    order_update_id: int = 0


@dataclass(frozen=True)
class OrderUpdateEvent(DomainEvent):
    """주문 업데이트 수신 이벤트.

    Args:
        agv_id: AGV 식별자.
        order_id: 주문 ID.
        order_update_id: 주문 업데이트 번호.
    """

    agv_id: str = ""
    order_id: str = ""
    order_update_id: int = 0


@dataclass(frozen=True)
class OrderCancelledEvent(DomainEvent):
    """주문 취소 이벤트.

    Args:
        agv_id: AGV 식별자.
        order_id: 취소된 주문 ID.
    """

    agv_id: str = ""
    order_id: str = ""


@dataclass(frozen=True)
class NodeReachedEvent(DomainEvent):
    """노드 도착 이벤트.

    Args:
        agv_id: AGV 식별자.
        node_id: 도착한 노드 ID.
        sequence_id: 노드 시퀀스 ID.
    """

    agv_id: str = ""
    node_id: str = ""
    sequence_id: int = 0


@dataclass(frozen=True)
class ActionStatusChangedEvent(DomainEvent):
    """액션 상태 변경 이벤트.

    Args:
        agv_id: AGV 식별자.
        action_id: 액션 ID.
        action_type: 액션 종류.
        previous_status: 이전 상태.
        new_status: 새 상태.
    """

    agv_id: str = ""
    action_id: str = ""
    action_type: str = ""
    previous_status: ActionStatus = ActionStatus.WAITING
    new_status: ActionStatus = ActionStatus.WAITING


@dataclass(frozen=True)
class ConnectionChangedEvent(DomainEvent):
    """AGV 연결 상태 변경 이벤트.

    Args:
        agv_id: AGV 식별자.
        previous_state: 이전 연결 상태.
        new_state: 새 연결 상태.
    """

    agv_id: str = ""
    previous_state: ConnectionState = ConnectionState.OFFLINE
    new_state: ConnectionState = ConnectionState.OFFLINE


@dataclass(frozen=True)
class OperatingModeChangedEvent(DomainEvent):
    """AGV 운영 모드 변경 이벤트.

    Args:
        agv_id: AGV 식별자.
        previous_mode: 이전 모드.
        new_mode: 새 모드.
    """

    agv_id: str = ""
    previous_mode: OperatingMode = OperatingMode.AUTOMATIC
    new_mode: OperatingMode = OperatingMode.AUTOMATIC


@dataclass(frozen=True)
class AgvErrorEvent(DomainEvent):
    """AGV 에러 발생 이벤트.

    Args:
        agv_id: AGV 식별자.
        error_type: 에러 유형.
        error_level: 에러 레벨.
        error_description: 에러 설명.
    """

    agv_id: str = ""
    error_type: str = ""
    error_level: str = ""
    error_description: str = ""
