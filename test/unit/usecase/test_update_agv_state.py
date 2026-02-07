"""UpdateAgvState 유스케이스 단위 테스트."""

from unittest.mock import MagicMock, call

import pytest

from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.entities.battery import BatteryState
from vda5050_fleet_adapter.domain.entities.connection import Connection
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.enums import ConnectionState, OperatingMode
from vda5050_fleet_adapter.domain.events.agv_events import (
    ConnectionChangedEvent,
    NodeReachedEvent,
    OperatingModeChangedEvent,
)
from vda5050_fleet_adapter.domain.value_objects.position import AgvPosition
from vda5050_fleet_adapter.usecase.update_agv_state import UpdateAgvState


@pytest.fixture
def mock_deps():
    state_repo = MagicMock()
    fleet_gateway = MagicMock()
    event_publisher = MagicMock()
    return state_repo, fleet_gateway, event_publisher


@pytest.fixture
def usecase(mock_deps):
    repo, fleet, pub = mock_deps
    return UpdateAgvState(
        state_repo=repo, fleet_gateway=fleet, event_publisher=pub,
    )


class TestHandleState:
    def test_saves_state_and_reports_to_rmf(
        self, usecase, mock_deps, sample_agv_state,
    ):
        repo, fleet, _ = mock_deps
        repo.get_state.return_value = None

        usecase.handle_state("AGV-001", sample_agv_state)

        repo.save_state.assert_called_once_with("AGV-001", sample_agv_state)
        fleet.update_position.assert_called_once()
        fleet.update_battery.assert_called_once_with("AGV-001", 85.0)
        fleet.update_state.assert_called_once()

    def test_detects_node_reached(
        self, usecase, mock_deps, sample_agv_state, sample_header,
    ):
        repo, _, pub = mock_deps

        prev_state = AgvState(
            header=sample_header,
            last_node_id="n0",
            last_node_sequence_id=0,
            operating_mode=OperatingMode.AUTOMATIC,
        )
        repo.get_state.return_value = prev_state

        sample_agv_state.last_node_id = "n2"
        sample_agv_state.last_node_sequence_id = 2

        usecase.handle_state("AGV-001", sample_agv_state)

        events = [c[0][0] for c in pub.publish.call_args_list]
        node_events = [e for e in events if isinstance(e, NodeReachedEvent)]
        assert len(node_events) == 1
        assert node_events[0].node_id == "n2"

    def test_detects_operating_mode_change(
        self, usecase, mock_deps, sample_agv_state, sample_header,
    ):
        repo, _, pub = mock_deps

        prev_state = AgvState(
            header=sample_header,
            last_node_id="n0",
            operating_mode=OperatingMode.AUTOMATIC,
        )
        repo.get_state.return_value = prev_state

        sample_agv_state.last_node_id = "n0"
        sample_agv_state.operating_mode = OperatingMode.MANUAL

        usecase.handle_state("AGV-001", sample_agv_state)

        events = [c[0][0] for c in pub.publish.call_args_list]
        mode_events = [
            e for e in events if isinstance(e, OperatingModeChangedEvent)
        ]
        assert len(mode_events) == 1
        assert mode_events[0].new_mode == OperatingMode.MANUAL

    def test_no_events_when_no_previous_state(
        self, usecase, mock_deps, sample_agv_state,
    ):
        repo, _, pub = mock_deps
        repo.get_state.return_value = None

        usecase.handle_state("AGV-001", sample_agv_state)
        pub.publish.assert_not_called()

    def test_no_position_report_when_none(
        self, usecase, mock_deps, sample_header,
    ):
        repo, fleet, _ = mock_deps
        repo.get_state.return_value = None

        state = AgvState(header=sample_header)
        usecase.handle_state("AGV-001", state)

        fleet.update_position.assert_not_called()
        fleet.update_battery.assert_not_called()


class TestHandleConnection:
    def test_saves_connection_state(self, usecase, mock_deps, sample_header):
        repo, _, _ = mock_deps
        repo.get_connection_state.return_value = ConnectionState.OFFLINE

        conn = Connection(
            header=sample_header,
            connection_state=ConnectionState.ONLINE,
        )
        usecase.handle_connection("AGV-001", conn)

        repo.save_connection_state.assert_called_once_with(
            "AGV-001", ConnectionState.ONLINE,
        )

    def test_publishes_event_on_change(self, usecase, mock_deps, sample_header):
        repo, _, pub = mock_deps
        repo.get_connection_state.return_value = ConnectionState.OFFLINE

        conn = Connection(
            header=sample_header,
            connection_state=ConnectionState.ONLINE,
        )
        usecase.handle_connection("AGV-001", conn)

        event = pub.publish.call_args[0][0]
        assert isinstance(event, ConnectionChangedEvent)
        assert event.new_state == ConnectionState.ONLINE

    def test_no_event_when_same_state(
        self, usecase, mock_deps, sample_header,
    ):
        repo, _, pub = mock_deps
        repo.get_connection_state.return_value = ConnectionState.ONLINE

        conn = Connection(
            header=sample_header,
            connection_state=ConnectionState.ONLINE,
        )
        usecase.handle_connection("AGV-001", conn)

        pub.publish.assert_not_called()
