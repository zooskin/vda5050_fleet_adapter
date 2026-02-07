"""인메모리 도메인 이벤트 발행자 구현체."""

import logging
import threading
from collections import defaultdict
from collections.abc import Callable

from vda5050_fleet_adapter.domain.events.agv_events import DomainEvent
from vda5050_fleet_adapter.usecase.ports.event_publisher import EventPublisher

logger = logging.getLogger(__name__)


class InMemoryEventPublisher(EventPublisher):
    """EventPublisher의 인메모리 구현체.

    동기 방식으로 이벤트를 핸들러에 전달한다.
    핸들러는 이벤트 타입별로 등록/호출된다.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._handlers: dict[
            type[DomainEvent], list[Callable[[DomainEvent], None]]
        ] = defaultdict(list)

    def publish(self, event: DomainEvent) -> None:
        """도메인 이벤트를 발행한다.

        등록된 모든 핸들러를 동기적으로 호출한다.
        개별 핸들러의 예외는 로깅 후 무시하여
        다른 핸들러 실행에 영향을 주지 않는다.
        """
        event_type = type(event)
        with self._lock:
            handlers = list(self._handlers.get(event_type, []))

        logger.debug(
            "Publishing event: %s (handlers=%d)",
            event_type.__name__, len(handlers),
        )

        for handler in handlers:
            try:
                handler(event)
            except Exception:
                logger.exception(
                    "Error in event handler for %s", event_type.__name__
                )

    def subscribe(
        self,
        event_type: type[DomainEvent],
        handler: Callable[[DomainEvent], None],
    ) -> None:
        """특정 타입의 도메인 이벤트를 구독한다."""
        with self._lock:
            self._handlers[event_type].append(handler)
        logger.debug(
            "Subscribed to event: %s", event_type.__name__
        )
