"""HandleAction 유스케이스 단위 테스트."""

from unittest.mock import MagicMock

import pytest

from vda5050_fleet_adapter.domain.entities.action import Action
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.entities.order import Order
from vda5050_fleet_adapter.domain.enums import BlockingType
from vda5050_fleet_adapter.domain.events.agv_events import (
    OrderCancelledEvent,
)
from vda5050_fleet_adapter.domain.exceptions import AgvNotFoundError
from vda5050_fleet_adapter.usecase.handle_action import HandleAction


@pytest.fixture
def mock_deps(sample_agv_state):
    agv_gateway = MagicMock()
    state_repo = MagicMock()
    event_publisher = MagicMock()

    state_repo.get_state.return_value = sample_agv_state
    return agv_gateway, state_repo, event_publisher


@pytest.fixture
def usecase(mock_deps):
    gw, repo, pub = mock_deps
    return HandleAction(
        agv_gateway=gw, state_repo=repo, event_publisher=pub,
    )


class TestPause:
    def test_sends_start_pause(self, usecase, mock_deps):
        gw, _, _ = mock_deps
        usecase.pause("AGV-001", "p1")

        gw.send_instant_actions.assert_called_once()
        actions = gw.send_instant_actions.call_args[0][1]
        assert len(actions) == 1
        assert actions[0].action_type == "startPause"
        assert actions[0].blocking_type == BlockingType.HARD

    def test_agv_not_found_raises(self, usecase, mock_deps):
        _, repo, _ = mock_deps
        repo.get_state.return_value = None

        with pytest.raises(AgvNotFoundError):
            usecase.pause("UNKNOWN", "p1")


class TestResume:
    def test_sends_stop_pause(self, usecase, mock_deps):
        gw, _, _ = mock_deps
        usecase.resume("AGV-001", "r1")

        actions = gw.send_instant_actions.call_args[0][1]
        assert actions[0].action_type == "stopPause"


class TestCancelOrder:
    def test_sends_cancel_and_clears_order(self, usecase, mock_deps):
        gw, repo, pub = mock_deps
        repo.get_current_order.return_value = Order(
            header=Header(version="2.0.0", manufacturer="T",
                          serial_number="A1"),
            order_id="o1", order_update_id=0,
        )

        usecase.cancel_order("AGV-001", "c1")

        actions = gw.send_instant_actions.call_args[0][1]
        assert actions[0].action_type == "cancelOrder"
        repo.clear_order.assert_called_once_with("AGV-001")

    def test_publishes_order_cancelled_event(self, usecase, mock_deps):
        _, repo, pub = mock_deps
        repo.get_current_order.return_value = Order(
            header=Header(version="2.0.0", manufacturer="T",
                          serial_number="A1"),
            order_id="o1", order_update_id=0,
        )

        usecase.cancel_order("AGV-001", "c1")

        event = pub.publish.call_args[0][0]
        assert isinstance(event, OrderCancelledEvent)
        assert event.order_id == "o1"

    def test_no_event_when_no_current_order(self, usecase, mock_deps):
        _, repo, pub = mock_deps
        repo.get_current_order.return_value = None

        usecase.cancel_order("AGV-001", "c1")
        pub.publish.assert_not_called()


class TestCharging:
    def test_start_charging(self, usecase, mock_deps):
        gw, _, _ = mock_deps
        usecase.start_charging("AGV-001", "ch1")

        actions = gw.send_instant_actions.call_args[0][1]
        assert actions[0].action_type == "startCharging"

    def test_stop_charging(self, usecase, mock_deps):
        gw, _, _ = mock_deps
        usecase.stop_charging("AGV-001", "ch2")

        actions = gw.send_instant_actions.call_args[0][1]
        assert actions[0].action_type == "stopCharging"


class TestSendCustomAction:
    def test_sends_custom_action(self, usecase, mock_deps):
        gw, _, _ = mock_deps
        action = Action(
            action_type="customScan",
            action_id="s1",
            blocking_type=BlockingType.SOFT,
        )

        usecase.send_custom_action("AGV-001", action)

        gw.send_instant_actions.assert_called_once_with("AGV-001", [action])
