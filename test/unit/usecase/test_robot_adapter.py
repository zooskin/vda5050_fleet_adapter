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
    api.pause.return_value = RobotAPIResult.SUCCESS
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

    def test_stop_sends_pause_not_cancel(self, adapter, mock_api):
        """Stop 호출 시 startPause를 전송한다 (cancelOrder 아님)."""
        execution = MagicMock()
        activity = MagicMock()
        execution.identifier.is_same.return_value = True
        adapter.execution = execution

        adapter.stop(activity)
        adapter.cancel_cmd_attempt()

        mock_api.pause.assert_called_once()
        mock_api.stop.assert_not_called()

    def test_stop_sets_paused_for_negotiation(self, adapter, mock_api):
        """Stop 호출 시 _is_paused_for_negotiation이 True가 된다."""
        execution = MagicMock()
        activity = MagicMock()
        execution.identifier.is_same.return_value = True
        adapter.execution = execution

        adapter.stop(activity)
        adapter.cancel_cmd_attempt()

        assert adapter._is_paused_for_negotiation is True

    def test_stop_preserves_order_state(self, adapter, mock_api):
        """Stop 호출 시 order 상태가 유지된다 (negotiation 대비)."""
        execution = MagicMock()
        activity = MagicMock()
        execution.identifier.is_same.return_value = True
        adapter.execution = execution
        adapter._active_order_id = 'order_1_abc'
        adapter._order_update_id = 2
        adapter._final_destination = 'wp3'

        adapter.stop(activity)
        adapter.cancel_cmd_attempt()

        assert adapter._active_order_id == 'order_1_abc'
        assert adapter._order_update_id == 2
        assert adapter._final_destination == 'wp3'

    def test_stop_ignores_different_activity(self, adapter, mock_api):
        """다른 activity에 대한 stop은 무시된다."""
        execution = MagicMock()
        activity = MagicMock()
        execution.identifier.is_same.return_value = False
        adapter.execution = execution

        adapter.stop(activity)

        assert adapter.execution is execution
        mock_api.pause.assert_not_called()
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
def adapter_with_handle(
    mock_api, mock_node,
    sample_nav_nodes, sample_nav_edges,
):
    """Create RobotAdapter with update_handle."""
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
    mock_update_handle = MagicMock()
    mock_update_handle.more.return_value.current_task_id \
        .return_value = 'compose.dispatch-001'
    robot.update_handle = mock_update_handle
    return robot


class TestFinalNameIntegration:
    """destination.final_name을 사용한 navigate 테스트."""

    def test_uses_final_name_destination(
        self, adapter_with_handle, mock_api,
    ):
        """destination.final_name이 있을 때 최종 목적지를 사용한다."""
        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        dest.name = 'wp2'
        dest.final_name = 'wp4'
        execution = MagicMock()

        adapter_with_handle.position = [0.0, 0.0, 0.0]
        adapter_with_handle.navigate(dest, execution)
        adapter_with_handle.cancel_cmd_attempt()

        assert adapter_with_handle._final_destination == 'wp4'

    def test_falls_back_without_final_name(
        self, adapter_with_handle, mock_api,
    ):
        """destination.final_name이 없으면 goal_node를 사용한다."""
        dest = MagicMock(spec=[])
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        execution = MagicMock()

        adapter_with_handle.position = [0.0, 0.0, 0.0]
        adapter_with_handle.navigate(dest, execution)
        adapter_with_handle.cancel_cmd_attempt()

        assert adapter_with_handle._final_destination == 'wp2'

    def test_falls_back_when_final_name_empty(
        self, adapter_with_handle, mock_api,
    ):
        """destination.final_name이 빈 문자열이면 fallback한다."""
        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        dest.name = 'wp2'
        dest.final_name = ''
        execution = MagicMock()

        adapter_with_handle.position = [0.0, 0.0, 0.0]
        adapter_with_handle.navigate(dest, execution)
        adapter_with_handle.cancel_cmd_attempt()

        assert adapter_with_handle._final_destination == 'wp2'


class TestOrderLifecycle:
    """Order 라이프사이클 관리 테스트."""

    def test_order_preserved_across_navigates(
        self, adapter_with_handle, mock_api
    ):
        """같은 task 내에서 order가 유지된다."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        dest1 = MagicMock()
        dest1.position = [5.0, 0.0, 0.0]
        dest1.map = 'map1'
        exec1 = MagicMock()
        adapter_with_handle.navigate(dest1, exec1)
        adapter_with_handle.cancel_cmd_attempt()

        order_id = adapter_with_handle._active_order_id

        # Command 완료 시뮬레이션
        mock_api.is_command_completed.return_value = True
        state = MagicMock()
        data = RobotUpdateData(
            robot_name='AGV-001', map_name='map1',
            position=[5.0, 0.0, 0.0], battery_soc=0.85,
        )
        adapter_with_handle.update(state, data)

        # Order state 유지 (task 아직 active)
        assert adapter_with_handle._active_order_id == order_id

    def test_order_update_id_increments(
        self, adapter_with_handle, mock_api
    ):
        """같은 task 내 두 번째 navigate에서 update_id 증가."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        dest1 = MagicMock()
        dest1.position = [5.0, 0.0, 0.0]
        dest1.map = 'map1'
        exec1 = MagicMock()
        adapter_with_handle.navigate(dest1, exec1)
        adapter_with_handle.cancel_cmd_attempt()

        order_id = adapter_with_handle._active_order_id
        assert adapter_with_handle._order_update_id == 0

        dest2 = MagicMock()
        dest2.position = [5.0, 5.0, 0.0]
        dest2.map = 'map1'
        exec2 = MagicMock()
        adapter_with_handle.navigate(dest2, exec2)
        adapter_with_handle.cancel_cmd_attempt()

        assert adapter_with_handle._active_order_id == order_id
        assert adapter_with_handle._order_update_id == 1

    def test_stop_pauses_for_negotiation(
        self, adapter_with_handle,
    ):
        """Stop 시 pause 상태로 전환 (order 유지)."""
        execution = MagicMock()
        activity = MagicMock()
        execution.identifier.is_same.return_value = True
        adapter_with_handle.execution = execution
        adapter_with_handle._active_order_id = 'order_1_abc'
        adapter_with_handle._current_task_id = 'compose.dispatch-001'

        adapter_with_handle.stop(activity)
        adapter_with_handle.cancel_cmd_attempt()

        assert adapter_with_handle._is_paused_for_negotiation is True
        assert adapter_with_handle._active_order_id == 'order_1_abc'


class TestDestinationName:
    """destination.name 사용 테스트."""

    def test_uses_destination_name_when_available(
        self, adapter_with_handle, mock_api
    ):
        """destination.name이 nav_nodes에 있으면 사용한다."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        dest = MagicMock()
        dest.name = 'wp3'
        dest.position = [4.9, 4.9, 0.0]  # near wp3 anyway
        dest.map = 'map1'
        execution = MagicMock()

        adapter_with_handle.navigate(dest, execution)
        adapter_with_handle.cancel_cmd_attempt()

        # goal_node should be wp3 (from name, not find_nearest_node)
        assert adapter_with_handle._final_destination is not None

    def test_falls_back_to_nearest_node(
        self, adapter_with_handle, mock_api
    ):
        """destination.name이 없으면 find_nearest_node를 사용한다."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        dest = MagicMock(spec=[])  # no 'name' attribute
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        execution = MagicMock()

        adapter_with_handle.navigate(dest, execution)
        adapter_with_handle.cancel_cmd_attempt()

        assert adapter_with_handle._final_destination is not None

    def test_falls_back_when_name_not_in_nav_nodes(
        self, adapter_with_handle, mock_api
    ):
        """destination.name이 nav_nodes에 없으면 fallback한다."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        dest = MagicMock()
        dest.name = 'unknown_waypoint'
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        execution = MagicMock()

        adapter_with_handle.navigate(dest, execution)
        adapter_with_handle.cancel_cmd_attempt()

        # Should use find_nearest_node fallback (wp2)
        assert adapter_with_handle._final_destination is not None


class TestFinalNameReResolve:
    """Order update 시 final_destination 재조회 테스트."""

    def test_uses_final_name_on_update(
        self, adapter_with_handle, mock_api
    ):
        """두 번째 navigate에서 final_name으로 재조회한다."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        # First navigate: no final_name
        dest1 = MagicMock()
        dest1.name = 'wp2'
        dest1.final_name = ''
        dest1.position = [5.0, 0.0, 0.0]
        dest1.map = 'map1'
        exec1 = MagicMock()
        adapter_with_handle.navigate(dest1, exec1)
        adapter_with_handle.cancel_cmd_attempt()

        # fallback used
        assert adapter_with_handle._final_destination == 'wp2'

        # Second navigate: final_name available
        dest2 = MagicMock()
        dest2.name = 'wp3'
        dest2.final_name = 'wp4'
        dest2.position = [5.0, 5.0, 0.0]
        dest2.map = 'map1'
        exec2 = MagicMock()
        adapter_with_handle.navigate(dest2, exec2)
        adapter_with_handle.cancel_cmd_attempt()

        assert adapter_with_handle._final_destination == 'wp4'

    def test_keeps_destination_when_no_final_name(
        self, adapter_with_handle, mock_api
    ):
        """final_name이 없으면 기존 final_dest를 유지한다."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        dest1 = MagicMock()
        dest1.name = 'wp2'
        dest1.final_name = ''
        dest1.position = [5.0, 0.0, 0.0]
        dest1.map = 'map1'
        exec1 = MagicMock()
        adapter_with_handle.navigate(dest1, exec1)
        adapter_with_handle.cancel_cmd_attempt()

        first_dest = adapter_with_handle._final_destination

        # Second navigate, still no final_name
        dest2 = MagicMock()
        dest2.name = 'wp3'
        dest2.final_name = ''
        dest2.position = [5.0, 5.0, 0.0]
        dest2.map = 'map1'
        exec2 = MagicMock()
        adapter_with_handle.navigate(dest2, exec2)
        adapter_with_handle.cancel_cmd_attempt()

        # _final_destination keeps previous value (fallback)
        assert adapter_with_handle._final_destination == first_dest


class TestNegotiation:
    """Negotiation 처리 테스트."""

    @staticmethod
    def _wait_thread(adapter):
        """Retry 스레드가 완료될 때까지 대기한다."""
        if adapter._issue_cmd_thread is not None:
            adapter._issue_cmd_thread.join(timeout=5.0)

    def test_negotiation_sends_cancel_then_new_order(
        self, adapter_with_handle, mock_api
    ):
        """Negotiation 시 cancelOrder 후 새 orderID로 navigate한다."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        # 1. 첫 번째 navigate (정상)
        dest1 = MagicMock()
        dest1.name = 'wp2'
        dest1.final_name = 'wp4'
        dest1.position = [5.0, 0.0, 0.0]
        dest1.map = 'map1'
        exec1 = MagicMock()
        adapter_with_handle.navigate(dest1, exec1)
        adapter_with_handle.cancel_cmd_attempt()

        first_order_id = adapter_with_handle._active_order_id
        assert first_order_id is not None

        # 2. stop() → startPause (직접 호출)
        activity = MagicMock()
        exec1.identifier.is_same.return_value = True
        adapter_with_handle.stop(activity)

        assert adapter_with_handle._is_paused_for_negotiation is True
        mock_api.pause.assert_called_once()

        # 3. navigate() → composite: cancelOrder → 1s 대기 → 새 order
        dest2 = MagicMock()
        dest2.name = 'wp3'
        dest2.final_name = 'wp4'
        dest2.position = [5.0, 5.0, 0.0]
        dest2.map = 'map1'
        exec2 = MagicMock()
        adapter_with_handle.navigate(dest2, exec2)
        # 스레드 완료 대기 (cancel + 1s + navigate)
        self._wait_thread(adapter_with_handle)

        # cancelOrder 호출됨
        mock_api.stop.assert_called_once()
        # navigate도 호출됨 (첫 번째 + negotiation 후)
        assert mock_api.navigate.call_count == 2
        # 새 orderID
        assert adapter_with_handle._active_order_id != first_order_id
        assert adapter_with_handle._order_update_id == 0
        assert adapter_with_handle._is_paused_for_negotiation is False

    def test_negotiation_creates_new_order_id(
        self, adapter_with_handle, mock_api
    ):
        """Negotiation 후 orderID가 새로 생성된다."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        dest1 = MagicMock()
        dest1.position = [5.0, 0.0, 0.0]
        dest1.map = 'map1'
        exec1 = MagicMock()
        adapter_with_handle.navigate(dest1, exec1)
        adapter_with_handle.cancel_cmd_attempt()

        old_id = adapter_with_handle._active_order_id

        # stop → pause (직접 호출)
        activity = MagicMock()
        exec1.identifier.is_same.return_value = True
        adapter_with_handle.stop(activity)

        # navigate → cancel + new order
        dest2 = MagicMock()
        dest2.position = [0.0, 5.0, 0.0]
        dest2.map = 'map1'
        exec2 = MagicMock()
        adapter_with_handle.navigate(dest2, exec2)
        self._wait_thread(adapter_with_handle)

        new_id = adapter_with_handle._active_order_id
        assert new_id is not None
        assert new_id != old_id
        assert new_id.startswith('order_')

    def test_task_end_while_paused_sends_cancel(
        self, adapter_with_handle, mock_api
    ):
        """Pause 상태에서 task 종료 시 cancelOrder를 전송한다."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        dest = MagicMock()
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        exec1 = MagicMock()
        adapter_with_handle.navigate(dest, exec1)
        adapter_with_handle.cancel_cmd_attempt()

        # stop → pause (직접 호출)
        activity = MagicMock()
        exec1.identifier.is_same.return_value = True
        adapter_with_handle.stop(activity)

        assert adapter_with_handle._is_paused_for_negotiation is True

        # task 종료 시뮬레이션: update에서 task_id가 빈 문자열
        adapter_with_handle.update_handle.more.return_value \
            .current_task_id.return_value = ''
        state = MagicMock()
        data = RobotUpdateData(
            robot_name='AGV-001', map_name='map1',
            position=[5.0, 0.0, 0.0], battery_soc=0.85,
        )
        adapter_with_handle.update(state, data)
        adapter_with_handle.cancel_cmd_attempt()

        # cancelOrder가 _reset_order_state에서 호출됨
        mock_api.stop.assert_called_once()
        assert adapter_with_handle._is_paused_for_negotiation is False
        assert adapter_with_handle._active_order_id is None

    def test_negotiation_flow_sequence(
        self, adapter_with_handle, mock_api
    ):
        """전체 negotiation 시퀀스: pause → cancel → new order."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        # 1. 정상 navigate
        dest1 = MagicMock()
        dest1.name = 'wp2'
        dest1.final_name = 'wp4'
        dest1.position = [5.0, 0.0, 0.0]
        dest1.map = 'map1'
        exec1 = MagicMock()
        adapter_with_handle.navigate(dest1, exec1)
        adapter_with_handle.cancel_cmd_attempt()

        # 2. negotiation 발생 → stop (startPause 직접 호출)
        activity = MagicMock()
        exec1.identifier.is_same.return_value = True
        adapter_with_handle.stop(activity)

        # pause 직접 호출 확인
        assert mock_api.pause.call_count == 1
        assert mock_api.stop.call_count == 0

        # 3. 새 경로로 navigate (composite: cancel → wait → navigate)
        dest2 = MagicMock()
        dest2.name = 'wp3'
        dest2.final_name = 'wp4'
        dest2.position = [5.0, 5.0, 0.0]
        dest2.map = 'map1'
        exec2 = MagicMock()
        adapter_with_handle.navigate(dest2, exec2)
        self._wait_thread(adapter_with_handle)

        # cancel 호출 확인
        assert mock_api.stop.call_count == 1
        # 새 navigate 호출 확인 (총 2회: 첫 번째 + negotiation 후)
        assert mock_api.navigate.call_count == 2


class TestDetourPath:
    """경유지(goal_node)를 경유하는 경로 생성 테스트.

    RMF가 교통 관리를 위해 경유지를 지정한 경우,
    최단 경로가 아닌 경유지를 반드시 거치는 경로를 생성해야 한다.
    """

    @pytest.fixture
    def detour_adapter(self, mock_api, mock_node):
        """T자 그래프로 경유지 테스트용 어댑터 생성.

        Graph:
            A(0,0) --- B(5,0) --- C(10,0)
                         |
                       D(5,5)
        shortest A→C: A→B→C (D를 거치지 않음)
        """
        from vda5050_fleet_adapter.infra.nav_graph.graph_utils import (
            create_graph,
        )
        nodes = {
            'A': {'x': 0.0, 'y': 0.0, 'attributes': {}},
            'B': {'x': 5.0, 'y': 0.0, 'attributes': {}},
            'C': {'x': 10.0, 'y': 0.0, 'attributes': {}},
            'D': {'x': 5.0, 'y': 5.0, 'attributes': {}},
        }
        edges = {
            'e0': {'start': 'A', 'end': 'B', 'attributes': {}},
            'e1': {'start': 'B', 'end': 'A', 'attributes': {}},
            'e2': {'start': 'B', 'end': 'C', 'attributes': {}},
            'e3': {'start': 'C', 'end': 'B', 'attributes': {}},
            'e4': {'start': 'B', 'end': 'D', 'attributes': {}},
            'e5': {'start': 'D', 'end': 'B', 'attributes': {}},
        }
        graph = create_graph(nodes, edges)
        robot = RobotAdapter(
            name='AGV-001', api=mock_api, node=mock_node,
            fleet_handle=MagicMock(),
            nav_nodes=nodes, nav_edges=edges, nav_graph=graph,
        )
        robot.configuration = MagicMock()
        mock_handle = MagicMock()
        mock_handle.more.return_value.current_task_id \
            .return_value = 'task-001'
        robot.update_handle = mock_handle
        return robot

    def test_path_goes_through_goal_node(
        self, detour_adapter, mock_api
    ):
        """경유지 D를 반드시 거치는 경로 생성 (A→B→D→B→C)."""
        detour_adapter.position = [0.0, 0.0, 0.0]

        dest = MagicMock()
        dest.name = 'D'
        dest.final_name = 'C'
        dest.position = [5.0, 5.0, 0.0]
        dest.map = 'L1'
        execution = MagicMock()

        detour_adapter.navigate(dest, execution)
        detour_adapter.cancel_cmd_attempt()

        call_args = mock_api.navigate.call_args[0]
        nodes = call_args[2]
        node_ids = [n.node_id for n in nodes]

        # D를 반드시 거쳐야 함
        assert 'D' in node_ids
        # D가 C보다 먼저 나와야 함
        assert node_ids.index('D') < node_ids.index('C')
        # 경로: A → B → D → B → C
        assert node_ids == ['A', 'B', 'D', 'B', 'C']

    def test_base_horizon_split_with_detour(
        self, detour_adapter, mock_api
    ):
        """경유지까지 Base, 이후 Horizon으로 분리."""
        detour_adapter.position = [0.0, 0.0, 0.0]

        dest = MagicMock()
        dest.name = 'D'
        dest.final_name = 'C'
        dest.position = [5.0, 5.0, 0.0]
        dest.map = 'L1'
        execution = MagicMock()

        detour_adapter.navigate(dest, execution)
        detour_adapter.cancel_cmd_attempt()

        call_args = mock_api.navigate.call_args[0]
        nodes = call_args[2]

        # D(index=2)까지 Base, 이후 Horizon
        for i, node in enumerate(nodes):
            node_id = node.node_id
            if node_id == 'D':
                d_index = i
                break
        for i, node in enumerate(nodes):
            if i <= d_index:
                assert node.released is True, (
                    f'{node.node_id} should be base (released)'
                )
            else:
                assert node.released is False, (
                    f'{node.node_id} should be horizon (not released)'
                )

    def test_direct_path_when_goal_is_final(
        self, detour_adapter, mock_api
    ):
        """경유지 == 최종목적지면 최단 경로 사용."""
        detour_adapter.position = [0.0, 0.0, 0.0]

        dest = MagicMock()
        dest.name = 'C'
        dest.final_name = 'C'
        dest.position = [10.0, 0.0, 0.0]
        dest.map = 'L1'
        execution = MagicMock()

        detour_adapter.navigate(dest, execution)
        detour_adapter.cancel_cmd_attempt()

        call_args = mock_api.navigate.call_args[0]
        nodes = call_args[2]
        node_ids = [n.node_id for n in nodes]

        # 최단 경로: A → B → C (D를 거치지 않음)
        assert node_ids == ['A', 'B', 'C']
        assert 'D' not in node_ids


class TestStartNodeFix:
    """Start_node 결정 로직 테스트."""

    def test_uses_current_position(
        self, adapter_with_handle, mock_api
    ):
        """Navigate에서 현재 위치를 start_node로 사용한다."""
        adapter_with_handle.position = [0.0, 0.0, 0.0]

        dest = MagicMock()
        dest.name = 'wp2'
        dest.position = [5.0, 0.0, 0.0]
        dest.map = 'map1'
        execution = MagicMock()
        adapter_with_handle.navigate(dest, execution)
        adapter_with_handle.cancel_cmd_attempt()

        call_args = mock_api.navigate.call_args[0]
        nodes = call_args[2]
        # First node should be wp1 (nearest to 0,0)
        if nodes:
            assert nodes[0].node_id == 'wp1'
