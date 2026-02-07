"""InMemoryStateRepository 단위 테스트."""

import threading

import pytest

from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.entities.node import Node
from vda5050_fleet_adapter.domain.entities.order import Order
from vda5050_fleet_adapter.domain.enums import ConnectionState
from vda5050_fleet_adapter.infra.repository import InMemoryStateRepository


@pytest.fixture
def repo():
    return InMemoryStateRepository()


@pytest.fixture
def header():
    return Header(version="2.0.0", manufacturer="T", serial_number="A1")


class TestStateOperations:
    def test_get_returns_none_for_unknown(self, repo):
        assert repo.get_state("UNKNOWN") is None

    def test_save_and_get(self, repo, sample_agv_state):
        repo.save_state("AGV-001", sample_agv_state)
        result = repo.get_state("AGV-001")
        assert result is sample_agv_state

    def test_overwrite(self, repo, sample_agv_state, header):
        repo.save_state("AGV-001", sample_agv_state)
        new_state = AgvState(header=header, driving=False)
        repo.save_state("AGV-001", new_state)
        assert repo.get_state("AGV-001").driving is False

    def test_get_all_agv_ids(self, repo, sample_agv_state, header):
        repo.save_state("AGV-001", sample_agv_state)
        repo.save_state("AGV-002", AgvState(header=header))
        ids = repo.get_all_agv_ids()
        assert set(ids) == {"AGV-001", "AGV-002"}

    def test_get_all_agv_ids_empty(self, repo):
        assert repo.get_all_agv_ids() == []


class TestOrderOperations:
    def test_get_returns_none_for_unknown(self, repo):
        assert repo.get_current_order("AGV-001") is None

    def test_save_and_get(self, repo, sample_order):
        repo.save_order("AGV-001", sample_order)
        assert repo.get_current_order("AGV-001") is sample_order

    def test_clear_order(self, repo, sample_order):
        repo.save_order("AGV-001", sample_order)
        repo.clear_order("AGV-001")
        assert repo.get_current_order("AGV-001") is None

    def test_clear_nonexistent_no_error(self, repo):
        repo.clear_order("UNKNOWN")


class TestConnectionOperations:
    def test_default_is_offline(self, repo):
        assert repo.get_connection_state("AGV-001") == ConnectionState.OFFLINE

    def test_save_and_get(self, repo):
        repo.save_connection_state("AGV-001", ConnectionState.ONLINE)
        assert repo.get_connection_state("AGV-001") == ConnectionState.ONLINE


class TestThreadSafety:
    def test_concurrent_writes(self, repo, header):
        errors = []

        def writer(agv_id):
            try:
                for i in range(100):
                    state = AgvState(header=header, order_id=f"o{i}")
                    repo.save_state(agv_id, state)
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=writer, args=(f"AGV-{i}",))
            for i in range(10)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0
        assert len(repo.get_all_agv_ids()) == 10
