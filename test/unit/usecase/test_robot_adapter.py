"""RobotAdapter 유닛 테스트."""

from unittest.mock import MagicMock

import pytest

from vda5050_fleet_adapter.usecase.ports.robot_api import (
    RobotAPIResult,
    RobotUpdateData,
)
from vda5050_fleet_adapter.usecase.robot_adapter import RobotAdapter


@pytest.fixture
def mock_api():
    """Mock RobotAPI."""
    api = MagicMock()
    api.navigate.return_value = RobotAPIResult.SUCCESS
    api.stop.return_value = RobotAPIResult.SUCCESS
    api.start_activity.return_value = RobotAPIResult.SUCCESS
    api.is_command_completed.return_value = False
    api.get_data.return_value = RobotUpdateData(
        robot_name='AGV-001',
        map_name='map1',
        position=[1.0, 2.0, 0.0],
        battery_soc=0.85,
    )
    return api


@pytest.fixture
def mock_node():
    """Mock ROS 2 node."""
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    node.get_clock.return_value = MagicMock()
    return node


@pytest.fixture
def adapter(mock_api, mock_node, sample_nav_nodes, sample_nav_edges):
    """Create RobotAdapter instance with mocks."""
    from vda5050_fleet_adapter.infra.nav_graph.graph_utils import (
        create_graph,
    )
    graph = create_graph(sample_nav_nodes, sample_nav_edges)
    robot = RobotAdapter(
        name='AGV-001',
        api=mock_api,
        node=mock_node,
        fleet_handle=MagicMock(),
        nav_nodes=sample_nav_nodes,
        nav_edges=sample_nav_edges,
        nav_graph=graph,
    )
    robot.configuration = MagicMock()
    return robot


class TestRobotAdapterNavigate:
    """navigate() 테스트."""

    def test_navigate_increments_cmd_id(self, adapter):
        """Navigate 호출 시 cmd_id가 증가한다."""
        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        dest.dock = None
        execution = MagicMock()

        initial_cmd_id = adapter.cmd_id
        adapter.navigate(dest, execution)

        assert adapter.cmd_id == initial_cmd_id + 1

    def test_navigate_sets_execution(self, adapter):
        """Navigate 호출 시 execution이 설정된다."""
        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        dest.dock = None
        execution = MagicMock()

        adapter.navigate(dest, execution)

        assert adapter.execution is execution

    def test_navigate_calls_api(self, adapter, mock_api):
        """navigate가 API.navigate를 호출한다."""
        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        dest.dock = None
        execution = MagicMock()

        adapter.navigate(dest, execution)
        # attempt_cmd_until_success가 스레드에서 호출하므로 잠시 대기
        adapter.cancel_cmd_attempt()

        mock_api.navigate.assert_called_once()
        call_args = mock_api.navigate.call_args
        assert call_args[0][0] == 'AGV-001'  # robot_name
        assert call_args[0][1] == 1  # cmd_id


class TestRobotAdapterStop:
    """stop() 테스트."""

    def test_stop_clears_execution(self, adapter, mock_api):
        """Stop 호출 시 execution이 None이 된다."""
        execution = MagicMock()
        activity = MagicMock()
        execution.identifier.is_same.return_value = True
        adapter.execution = execution

        adapter.stop(activity)
        adapter.cancel_cmd_attempt()

        assert adapter.execution is None

    def test_stop_increments_cmd_id(self, adapter, mock_api):
        """Stop 호출 시 cmd_id가 증가한다."""
        execution = MagicMock()
        activity = MagicMock()
        execution.identifier.is_same.return_value = True
        adapter.execution = execution
        initial_cmd_id = adapter.cmd_id

        adapter.stop(activity)
        adapter.cancel_cmd_attempt()

        assert adapter.cmd_id == initial_cmd_id + 1

    def test_stop_ignores_different_activity(self, adapter, mock_api):
        """다른 activity에 대한 stop은 무시된다."""
        execution = MagicMock()
        activity = MagicMock()
        execution.identifier.is_same.return_value = False
        adapter.execution = execution

        adapter.stop(activity)

        assert adapter.execution is execution
        mock_api.stop.assert_not_called()


class TestRobotAdapterExecuteAction:
    """execute_action() 테스트."""

    def test_execute_action_increments_cmd_id(self, adapter):
        """execute_action 호출 시 cmd_id가 증가한다."""
        execution = MagicMock()
        initial_cmd_id = adapter.cmd_id

        adapter.execute_action('charge', {}, execution)
        adapter.cancel_cmd_attempt()

        assert adapter.cmd_id == initial_cmd_id + 1

    def test_execute_action_sets_execution(self, adapter):
        """execute_action 호출 시 execution이 설정된다."""
        execution = MagicMock()

        adapter.execute_action('teleop', {}, execution)
        adapter.cancel_cmd_attempt()

        assert adapter.execution is execution

    def test_execute_action_calls_api(self, adapter, mock_api):
        """execute_action이 API.start_activity를 호출한다."""
        execution = MagicMock()
        params = {'zone': 'A'}

        adapter.execute_action('clean', params, execution)
        adapter.cancel_cmd_attempt()

        mock_api.start_activity.assert_called_once()
        call_args = mock_api.start_activity.call_args
        assert call_args[0][0] == 'AGV-001'
        assert call_args[0][2] == 'clean'
        assert call_args[0][3] == {'zone': 'A'}


class TestRobotAdapterUpdate:
    """update() 테스트."""

    def test_update_completes_execution(self, adapter, mock_api):
        """명령 완료 시 execution.finished()가 호출된다."""
        execution = MagicMock()
        adapter.execution = execution
        adapter.cmd_id = 5
        adapter.update_handle = MagicMock()
        mock_api.is_command_completed.return_value = True

        state = MagicMock()
        data = RobotUpdateData(
            robot_name='AGV-001',
            map_name='map1',
            position=[1.0, 2.0, 0.0],
            battery_soc=0.85,
        )

        adapter.update(state, data)

        execution.finished.assert_called_once()
        assert adapter.execution is None

    def test_update_keeps_execution_if_not_completed(
        self, adapter, mock_api
    ):
        """명령 미완료 시 execution이 유지된다."""
        execution = MagicMock()
        adapter.execution = execution
        adapter.cmd_id = 5
        adapter.update_handle = MagicMock()
        mock_api.is_command_completed.return_value = False

        state = MagicMock()
        data = RobotUpdateData(
            robot_name='AGV-001',
            map_name='map1',
            position=[1.0, 2.0, 0.0],
            battery_soc=0.85,
        )

        adapter.update(state, data)

        execution.finished.assert_not_called()
        assert adapter.execution is execution

    def test_update_sends_state_to_rmf(self, adapter, mock_api):
        """update가 update_handle.update()를 호출한다."""
        adapter.update_handle = MagicMock()

        state = MagicMock()
        data = RobotUpdateData(
            robot_name='AGV-001',
            map_name='map1',
            position=[1.0, 2.0, 0.0],
            battery_soc=0.85,
        )

        adapter.update(state, data)

        adapter.update_handle.update.assert_called_once()

    def test_update_tracks_position(self, adapter, mock_api):
        """update가 position을 캐시한다."""
        adapter.update_handle = MagicMock()

        state = MagicMock()
        data = RobotUpdateData(
            robot_name='AGV-001',
            map_name='map1',
            position=[3.0, 4.0, 1.5],
            battery_soc=0.85,
        )

        adapter.update(state, data)

        assert adapter.position == [3.0, 4.0, 1.5]


class TestRobotAdapterRetry:
    """attempt_cmd_until_success() 테스트."""

    def test_retry_on_failure(self, adapter, mock_api):
        """API 실패 시 재시도한다."""
        call_count = 0

        def failing_then_success(*args):
            nonlocal call_count
            call_count += 1
            return call_count >= 2

        mock_api.navigate.side_effect = failing_then_success

        adapter.attempt_cmd_until_success(
            cmd=mock_api.navigate,
            args=('AGV-001', 1, [], [], 'map1'),
        )

        # 스레드가 자연스럽게 완료될 때까지 대기 (cancel하지 않음)
        if adapter._issue_cmd_thread is not None:
            adapter._issue_cmd_thread.join(timeout=5.0)

        assert call_count >= 2

    def test_cancel_stops_retry(self, adapter, mock_api):
        """cancel_cmd_attempt()가 재시도를 중지한다."""
        mock_api.navigate.return_value = False

        adapter.attempt_cmd_until_success(
            cmd=mock_api.navigate,
            args=('AGV-001', 1, [], [], 'map1'),
        )

        # 즉시 취소
        adapter.cancel_cmd_attempt()

        assert adapter._issue_cmd_thread is None
