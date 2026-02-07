"""ProcessOrder 유스케이스 단위 테스트."""

from unittest.mock import MagicMock, call

import pytest

from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.entities.order import Order
from vda5050_fleet_adapter.domain.events.agv_events import (
    OrderReceivedEvent,
    OrderUpdateEvent,
)
from vda5050_fleet_adapter.domain.exceptions import (
    AgvNotFoundError,
    OrderValidationError,
)
from vda5050_fleet_adapter.usecase.process_order import ProcessOrder


@pytest.fixture
def mock_deps(sample_config, sample_agv_state):
    agv_gateway = MagicMock()
    state_repo = MagicMock()
    event_publisher = MagicMock()

    state_repo.get_state.return_value = sample_agv_state

    return agv_gateway, state_repo, event_publisher, sample_config


@pytest.fixture
def usecase(mock_deps):
    gw, repo, pub, cfg = mock_deps
    return ProcessOrder(
        agv_gateway=gw, state_repo=repo,
        event_publisher=pub, config=cfg,
    )


class TestSendNewOrder:
    def test_creates_and_sends_order(self, usecase, mock_deps):
        gw, repo, pub, _ = mock_deps
        waypoints = [(1.0, 2.0, 0.0, "map1"), (5.0, 6.0, 1.57, "map1")]

        result = usecase.send_new_order("AGV-001", "o1", waypoints)

        assert isinstance(result, Order)
        assert result.order_id == "o1"
        assert len(result.nodes) == 2
        assert len(result.edges) == 1
        gw.send_order.assert_called_once()
        repo.save_order.assert_called_once()

    def test_publishes_order_received_event(self, usecase, mock_deps):
        _, _, pub, _ = mock_deps
        waypoints = [(0.0, 0.0, 0.0, "map1")]

        usecase.send_new_order("AGV-001", "o1", waypoints)

        pub.publish.assert_called_once()
        event = pub.publish.call_args[0][0]
        assert isinstance(event, OrderReceivedEvent)
        assert event.order_id == "o1"

    def test_agv_not_found_raises(self, usecase, mock_deps):
        _, repo, _, _ = mock_deps
        repo.get_state.return_value = None

        with pytest.raises(AgvNotFoundError):
            usecase.send_new_order("UNKNOWN", "o1", [(0, 0, 0, "m")])

    def test_node_sequence_ids_are_even(self, usecase, mock_deps):
        waypoints = [(i, i, 0.0, "m1") for i in range(5)]
        result = usecase.send_new_order("AGV-001", "o1", waypoints)

        for node in result.nodes:
            assert node.sequence_id % 2 == 0

    def test_edge_sequence_ids_are_odd(self, usecase, mock_deps):
        waypoints = [(i, i, 0.0, "m1") for i in range(3)]
        result = usecase.send_new_order("AGV-001", "o1", waypoints)

        for edge in result.edges:
            assert edge.sequence_id % 2 == 1


class TestUpdateOrder:
    def test_updates_existing_order(self, usecase, mock_deps):
        gw, repo, pub, _ = mock_deps

        existing_order = usecase.send_new_order(
            "AGV-001", "o1",
            [(0, 0, 0, "m1"), (1, 1, 0, "m1")],
        )
        repo.get_current_order.return_value = existing_order

        result = usecase.update_order(
            "AGV-001", [(10, 10, 0, "m1")],
        )

        assert result.order_id == "o1"
        assert result.order_update_id == 1

    def test_no_current_order_raises(self, usecase, mock_deps):
        _, repo, _, _ = mock_deps
        repo.get_current_order.return_value = None

        with pytest.raises(OrderValidationError, match="현재 주문"):
            usecase.update_order("AGV-001", [(0, 0, 0, "m1")])

    def test_publishes_order_update_event(self, usecase, mock_deps):
        _, repo, pub, _ = mock_deps

        existing = usecase.send_new_order(
            "AGV-001", "o1", [(0, 0, 0, "m1"), (1, 1, 0, "m1")],
        )
        repo.get_current_order.return_value = existing
        pub.reset_mock()

        usecase.update_order("AGV-001", [(5, 5, 0, "m1")])

        event = pub.publish.call_args[0][0]
        assert isinstance(event, OrderUpdateEvent)
        assert event.order_update_id == 1
