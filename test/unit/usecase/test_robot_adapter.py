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

    def test_navigate_creates_new_order(self, adapter):
        """첫 navigate 시 _active_order_id가 생성된다."""
        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        dest.dock = None
        execution = MagicMock()

        assert adapter._active_order_id is None
        adapter.navigate(dest, execution)
        adapter.cancel_cmd_attempt()

        assert adapter._active_order_id is not None
        assert adapter._active_order_id.startswith('order_')
        assert adapter._order_update_id == 0
        assert adapter._final_destination is not None

    def test_navigate_updates_existing_order(self, adapter):
        """두 번째 navigate 시 같은 orderID, update_id 증가."""
        dest1 = MagicMock()
        dest1.position = [5.0, 0.0, 0.0]
        dest1.map = 'map1'
        dest1.dock = None
        exec1 = MagicMock()

        adapter.navigate(dest1, exec1)
        adapter.cancel_cmd_attempt()

        first_order_id = adapter._active_order_id
        first_final_dest = adapter._final_destination

        dest2 = MagicMock()
        dest2.position = [0.0, 0.0, 0.0]
        dest2.map = 'map1'
        dest2.dock = None
        exec2 = MagicMock()

        adapter.navigate(dest2, exec2)
        adapter.cancel_cmd_attempt()

        assert adapter._active_order_id == first_order_id
        assert adapter._order_update_id == 1
        assert adapter._final_destination == first_final_dest

    def test_navigate_passes_order_id_to_api(self, adapter, mock_api):
        """navigate가 order_id와 order_update_id를 API에 전달한다."""
        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        dest.dock = None
        execution = MagicMock()

        adapter.navigate(dest, execution)
        adapter.cancel_cmd_attempt()

        call_args = mock_api.navigate.call_args[0]
        # args: (name, cmd_id, nodes, edges, map, order_id, update_id)
        assert len(call_args) == 7
        assert call_args[5].startswith('order_')  # order_id
        assert call_args[6] == 0  # order_update_id


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

    def test_stop_resets_order_state(self, adapter, mock_api):
        """Stop 호출 시 order 상태가 초기화된다."""
        execution = MagicMock()
        activity = MagicMock()
        execution.identifier.is_same.return_value = True
        adapter.execution = execution
        adapter._active_order_id = 'order_1_abc'
        adapter._order_update_id = 2
        adapter._final_destination = 'wp3'

        adapter.stop(activity)
        adapter.cancel_cmd_attempt()

        assert adapter._active_order_id is None
        assert adapter._order_update_id == 0
        assert adapter._final_destination is None

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

    def test_completion_preserves_order_during_task(
        self, adapter, mock_api
    ):
        """명령 완료 시 task 중에는 order 상태가 유지된다."""
        execution = MagicMock()
        adapter.execution = execution
        adapter.cmd_id = 5
        mock_update_handle = MagicMock()
        mock_update_handle.more.return_value.current_task_id \
            .return_value = 'compose.dispatch-001'
        adapter.update_handle = mock_update_handle
        adapter._active_order_id = 'order_5_abc'
        adapter._order_update_id = 1
        adapter._final_destination = 'wp3'
        adapter._current_task_id = 'compose.dispatch-001'
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
        # Order state preserved (task still active)
        assert adapter._active_order_id == 'order_5_abc'

    def test_completion_resets_order_when_task_ends(
        self, adapter, mock_api
    ):
        """Task 완료 시 order 상태가 초기화된다."""
        execution = MagicMock()
        adapter.execution = execution
        adapter.cmd_id = 5
        mock_update_handle = MagicMock()
        # current_task_id returns empty string (task ended)
        mock_update_handle.more.return_value.current_task_id \
            .return_value = ''
        adapter.update_handle = mock_update_handle
        adapter._active_order_id = 'order_5_abc'
        adapter._order_update_id = 1
        adapter._final_destination = 'wp3'
        adapter._current_task_id = 'compose.dispatch-001'
        mock_api.is_command_completed.return_value = True

        state = MagicMock()
        data = RobotUpdateData(
            robot_name='AGV-001',
            map_name='map1',
            position=[1.0, 2.0, 0.0],
            battery_soc=0.85,
        )

        adapter.update(state, data)

        assert adapter._active_order_id is None
        assert adapter._order_update_id == 0
        assert adapter._final_destination is None

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
            if call_count >= 2:
                return RobotAPIResult.SUCCESS
            return RobotAPIResult.RETRY

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


@pytest.fixture
def mock_task_tracker():
    """Mock TaskTracker."""
    tracker = MagicMock()
    tracker.get_final_destination.return_value = None
    tracker.clear_booking = MagicMock()
    return tracker


@pytest.fixture
def adapter_with_tracker(
    mock_api, mock_node, mock_task_tracker,
    sample_nav_nodes, sample_nav_edges,
):
    """Create RobotAdapter with task_tracker."""
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
        task_tracker=mock_task_tracker,
    )
    robot.configuration = MagicMock()
    mock_update_handle = MagicMock()
    mock_update_handle.more.return_value.current_task_id \
        .return_value = 'compose.dispatch-001'
    robot.update_handle = mock_update_handle
    return robot


class TestTaskTrackerIntegration:
    """task_tracker를 사용한 navigate 테스트."""

    def test_uses_tracker_destination(
        self, adapter_with_tracker, mock_task_tracker
    ):
        """Task_tracker가 있을 때 최종 목적지를 사용한다."""
        mock_task_tracker.get_final_destination.return_value = 'wp4'

        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        execution = MagicMock()

        adapter_with_tracker.position = [0.0, 0.0, 0.0]
        adapter_with_tracker.navigate(dest, execution)
        adapter_with_tracker.cancel_cmd_attempt()

        assert adapter_with_tracker._final_destination == 'wp4'

    def test_falls_back_without_tracker_result(
        self, adapter_with_tracker, mock_task_tracker
    ):
        """Task_tracker가 None 반환 시 goal_node를 사용한다."""
        mock_task_tracker.get_final_destination.return_value = None

        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        execution = MagicMock()

        adapter_with_tracker.position = [0.0, 0.0, 0.0]
        adapter_with_tracker.navigate(dest, execution)
        adapter_with_tracker.cancel_cmd_attempt()

        assert adapter_with_tracker._final_destination == 'wp2'

    def test_backward_compat_without_tracker(self, adapter):
        """Task_tracker 없이도 동작한다."""
        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        execution = MagicMock()

        adapter.navigate(dest, execution)
        adapter.cancel_cmd_attempt()

        assert adapter._final_destination is not None


class TestOrderLifecycle:
    """Order 라이프사이클 관리 테스트."""

    def test_order_preserved_across_navigates(
        self, adapter_with_tracker, mock_api
    ):
        """같은 task 내에서 order가 유지된다."""
        adapter_with_tracker.position = [0.0, 0.0, 0.0]

        dest1 = MagicMock()
        dest1.position = [5.0, 0.0, 0.0]
        dest1.map = 'map1'
        exec1 = MagicMock()
        adapter_with_tracker.navigate(dest1, exec1)
        adapter_with_tracker.cancel_cmd_attempt()

        order_id = adapter_with_tracker._active_order_id

        # Command 완료 시뮬레이션
        mock_api.is_command_completed.return_value = True
        state = MagicMock()
        data = RobotUpdateData(
            robot_name='AGV-001', map_name='map1',
            position=[5.0, 0.0, 0.0], battery_soc=0.85,
        )
        adapter_with_tracker.update(state, data)

        # Order state 유지 (task 아직 active)
        assert adapter_with_tracker._active_order_id == order_id

    def test_order_update_id_increments(
        self, adapter_with_tracker, mock_api
    ):
        """같은 task 내 두 번째 navigate에서 update_id 증가."""
        adapter_with_tracker.position = [0.0, 0.0, 0.0]

        dest1 = MagicMock()
        dest1.position = [5.0, 0.0, 0.0]
        dest1.map = 'map1'
        exec1 = MagicMock()
        adapter_with_tracker.navigate(dest1, exec1)
        adapter_with_tracker.cancel_cmd_attempt()

        order_id = adapter_with_tracker._active_order_id
        assert adapter_with_tracker._order_update_id == 0

        dest2 = MagicMock()
        dest2.position = [5.0, 5.0, 0.0]
        dest2.map = 'map1'
        exec2 = MagicMock()
        adapter_with_tracker.navigate(dest2, exec2)
        adapter_with_tracker.cancel_cmd_attempt()

        assert adapter_with_tracker._active_order_id == order_id
        assert adapter_with_tracker._order_update_id == 1

    def test_stop_resets_order_and_clears_booking(
        self, adapter_with_tracker, mock_task_tracker
    ):
        """Stop 시 order 리셋 및 booking 캐시 정리."""
        execution = MagicMock()
        activity = MagicMock()
        execution.identifier.is_same.return_value = True
        adapter_with_tracker.execution = execution
        adapter_with_tracker._active_order_id = 'order_1_abc'
        adapter_with_tracker._current_task_id = 'compose.dispatch-001'

        adapter_with_tracker.stop(activity)
        adapter_with_tracker.cancel_cmd_attempt()

        assert adapter_with_tracker._active_order_id is None
        mock_task_tracker.clear_booking.assert_called_once_with(
            'compose.dispatch-001'
        )


class TestStartNodeFix:
    """Start_node 결정 로직 테스트."""

    def test_uses_current_position(
        self, adapter_with_tracker, mock_api
    ):
        """Navigate에서 현재 위치를 start_node로 사용한다."""
        adapter_with_tracker.position = [0.0, 0.0, 0.0]

        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        execution = MagicMock()
        adapter_with_tracker.navigate(dest, execution)
        adapter_with_tracker.cancel_cmd_attempt()

        call_args = mock_api.navigate.call_args[0]
        nodes = call_args[2]
        # First node should be wp1 (nearest to 0,0)
        if nodes:
            assert nodes[0].node_id == 'wp1'
