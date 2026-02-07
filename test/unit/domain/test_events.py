"""도메인 이벤트 단위 테스트."""

from datetime import datetime

from vda5050_fleet_adapter.domain.enums import ActionStatus, ConnectionState
from vda5050_fleet_adapter.domain.events.agv_events import (
    ActionStatusChangedEvent,
    ConnectionChangedEvent,
    DomainEvent,
    NodeReachedEvent,
    OrderCancelledEvent,
    OrderReceivedEvent,
)


class TestDomainEvent:
    def test_timestamp_auto_set(self):
        event = DomainEvent()
        assert isinstance(event.timestamp, datetime)

    def test_frozen(self):
        event = DomainEvent()
        try:
            event.timestamp = datetime.now()
            assert False, "Should raise FrozenInstanceError"
        except AttributeError:
            pass


class TestOrderReceivedEvent:
    def test_fields(self):
        event = OrderReceivedEvent(
            agv_id="AGV-001", order_id="o1", order_update_id=0,
        )
        assert event.agv_id == "AGV-001"
        assert event.order_id == "o1"


class TestNodeReachedEvent:
    def test_fields(self):
        event = NodeReachedEvent(
            agv_id="AGV-001", node_id="n2", sequence_id=2,
        )
        assert event.node_id == "n2"
        assert event.sequence_id == 2


class TestConnectionChangedEvent:
    def test_fields(self):
        event = ConnectionChangedEvent(
            agv_id="AGV-001",
            previous_state=ConnectionState.ONLINE,
            new_state=ConnectionState.CONNECTIONBROKEN,
        )
        assert event.previous_state == ConnectionState.ONLINE
        assert event.new_state == ConnectionState.CONNECTIONBROKEN
