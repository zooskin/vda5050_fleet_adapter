"""도메인 이벤트 발행 포트 인터페이스.

도메인 이벤트의 발행/구독을 추상화한다.
인메모리 이벤트 버스 또는 외부 메시지 큐로 구현 가능하다.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable

from vda5050_fleet_adapter.domain.events.agv_events import DomainEvent


class EventPublisher(ABC):
    """도메인 이벤트 발행자 인터페이스."""

    @abstractmethod
    def publish(self, event: DomainEvent) -> None:
        """도메인 이벤트를 발행한다.

        Args:
            event: 발행할 도메인 이벤트.
        """

    @abstractmethod
    def subscribe(
        self,
        event_type: type[DomainEvent],
        handler: Callable[[DomainEvent], None],
    ) -> None:
        """특정 타입의 도메인 이벤트를 구독한다.

        Args:
            event_type: 구독할 이벤트 타입.
            handler: 이벤트 수신 시 호출할 핸들러.
        """
