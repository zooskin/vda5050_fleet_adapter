"""InMemoryEventPublisher 단위 테스트."""

import pytest

from vda5050_fleet_adapter.domain.events.agv_events import (
    ConnectionChangedEvent,
    DomainEvent,
    NodeReachedEvent,
    OrderReceivedEvent,
)
from vda5050_fleet_adapter.infra.event import InMemoryEventPublisher


@pytest.fixture
def publisher():
    return InMemoryEventPublisher()


class TestPublishSubscribe:
    def test_handler_receives_event(self, publisher):
        received = []
        publisher.subscribe(OrderReceivedEvent, received.append)

        event = OrderReceivedEvent(agv_id="A1", order_id="o1")
        publisher.publish(event)

        assert len(received) == 1
        assert received[0].order_id == "o1"

    def test_multiple_handlers(self, publisher):
        r1, r2 = [], []
        publisher.subscribe(NodeReachedEvent, r1.append)
        publisher.subscribe(NodeReachedEvent, r2.append)

        event = NodeReachedEvent(agv_id="A1", node_id="n0")
        publisher.publish(event)

        assert len(r1) == 1
        assert len(r2) == 1

    def test_type_isolation(self, publisher):
        order_events = []
        node_events = []
        publisher.subscribe(OrderReceivedEvent, order_events.append)
        publisher.subscribe(NodeReachedEvent, node_events.append)

        publisher.publish(OrderReceivedEvent(agv_id="A1", order_id="o1"))
        publisher.publish(NodeReachedEvent(agv_id="A1", node_id="n0"))

        assert len(order_events) == 1
        assert len(node_events) == 1

    def test_no_handler_no_error(self, publisher):
        publisher.publish(OrderReceivedEvent(agv_id="A1", order_id="o1"))

    def test_handler_exception_does_not_break_others(self, publisher):
        results = []

        def bad_handler(event):
            raise ValueError("boom")

        def good_handler(event):
            results.append(event)

        publisher.subscribe(OrderReceivedEvent, bad_handler)
        publisher.subscribe(OrderReceivedEvent, good_handler)

        publisher.publish(OrderReceivedEvent(agv_id="A1", order_id="o1"))

        assert len(results) == 1
