"""TaskApiListener 유닛 테스트.

ROS 2 런타임 없이 콜백 로직을 직접 테스트한다.
"""

import json
from unittest.mock import MagicMock, patch

import pytest

from vda5050_fleet_adapter.infra.ros.task_api_listener import (
    TaskApiListener,
)


@pytest.fixture
def listener():
    """Create listener with mocked ROS subscriptions."""
    with patch.object(TaskApiListener, '_setup_subscriptions'):
        return TaskApiListener(
            MagicMock(),
            fleet_name='tinyRobot',
            index_to_name=['wp1', 'wp2', 'charger1', 'wp3'],
        )


class TestExtractDestination:
    """_extract_destination() 테스트."""

    def test_go_to_place(self, listener):
        """Go_to_place request에서 목적지를 추출한다."""
        payload = {
            'type': 'dispatch_task_request',
            'request': {
                'category': 'compose',
                'description': {
                    'category': 'go_to_place',
                    'phases': [{
                        'activity': {
                            'category': 'go_to_place',
                            'description': {
                                'one_of': [{'waypoint': 'pantry'}],
                            },
                        },
                    }],
                },
            },
        }
        assert listener._extract_destination(payload) == 'pantry'

    def test_go_to_place_multiple_one_of(self, listener):
        """Multiple one_of에서 마지막 waypoint를 추출한다."""
        payload = {
            'type': 'dispatch_task_request',
            'request': {
                'category': 'compose',
                'description': {
                    'category': 'go_to_place',
                    'phases': [{
                        'activity': {
                            'category': 'go_to_place',
                            'description': {
                                'one_of': [
                                    {'waypoint': 'kitchen'},
                                    {'waypoint': 'pantry'},
                                ],
                            },
                        },
                    }],
                },
            },
        }
        assert listener._extract_destination(payload) == 'pantry'

    def test_patrol(self, listener):
        """Patrol request에서 마지막 place를 추출한다."""
        payload = {
            'type': 'dispatch_task_request',
            'request': {
                'category': 'patrol',
                'description': {
                    'places': ['wp1', 'wp2', 'pantry'],
                    'rounds': 1,
                },
            },
        }
        assert listener._extract_destination(payload) == 'pantry'

    def test_unknown_category(self, listener):
        """알 수 없는 카테고리는 None을 반환한다."""
        payload = {
            'request': {
                'category': 'unknown',
                'description': {},
            },
        }
        assert listener._extract_destination(payload) is None

    def test_empty_payload(self, listener):
        """빈 payload는 None을 반환한다."""
        assert listener._extract_destination({}) is None


class TestOnTaskRequest:
    """_on_task_request() 콜백 테스트."""

    def test_caches_destination(self, listener):
        """Request 수신 시 destination을 캐시한다."""
        msg = MagicMock()
        msg.request_id = 'direct_test-uuid'
        msg.json_msg = json.dumps({
            'type': 'dispatch_task_request',
            'request': {
                'category': 'compose',
                'description': {
                    'category': 'go_to_place',
                    'phases': [{
                        'activity': {
                            'category': 'go_to_place',
                            'description': {
                                'one_of': [{'waypoint': 'pantry'}],
                            },
                        },
                    }],
                },
            },
        })

        listener._on_task_request(msg)

        assert listener._request_destinations['direct_test-uuid'] == (
            'pantry'
        )

    def test_handles_invalid_json(self, listener):
        """잘못된 JSON은 예외 없이 처리된다."""
        msg = MagicMock()
        msg.request_id = 'bad-req'
        msg.json_msg = 'not valid json{'

        listener._on_task_request(msg)

        assert 'bad-req' not in listener._request_destinations


class TestOnTaskResponse:
    """_on_task_response() 콜백 테스트."""

    def test_correlates_booking_id(self, listener):
        """Response에서 booking_id -> destination 매핑을 생성한다."""
        listener._request_destinations['direct_abc'] = 'pantry'

        msg = MagicMock()
        msg.type = 2  # TYPE_RESPONDING
        msg.request_id = 'direct_abc'
        msg.json_msg = json.dumps({
            'success': True,
            'state': {
                'booking': {'id': 'compose.dispatch-123'},
            },
        })

        listener._on_task_response(msg)

        assert listener.get_final_destination(
            'compose.dispatch-123'
        ) == 'pantry'
        assert 'direct_abc' not in listener._request_destinations

    def test_ignores_acknowledge_type(self, listener):
        """TYPE_ACKNOWLEDGE 응답은 무시한다."""
        listener._request_destinations['req-1'] = 'kitchen'

        msg = MagicMock()
        msg.type = 1  # TYPE_ACKNOWLEDGE
        msg.request_id = 'req-1'
        msg.json_msg = '{}'

        listener._on_task_response(msg)

        assert not listener._booking_destinations

    def test_ignores_unsuccessful(self, listener):
        """success=false 응답은 무시한다."""
        listener._request_destinations['req-1'] = 'kitchen'

        msg = MagicMock()
        msg.type = 2
        msg.request_id = 'req-1'
        msg.json_msg = json.dumps({'success': False})

        listener._on_task_response(msg)

        assert not listener._booking_destinations

    def test_handles_missing_request(self, listener):
        """Request 캐시에 없는 response도 에러 없이 처리된다."""
        msg = MagicMock()
        msg.type = 2
        msg.request_id = 'orphan-req'
        msg.json_msg = json.dumps({
            'success': True,
            'state': {'booking': {'id': 'compose.dispatch-999'}},
        })

        listener._on_task_response(msg)

        assert not listener._booking_destinations


class TestGetFinalDestination:
    """get_final_destination() 테스트."""

    def test_returns_none_without_mapping(self, listener):
        """매핑이 없으면 None을 반환한다."""
        assert listener.get_final_destination('unknown-id') is None

    def test_returns_cached_destination(self, listener):
        """캐시된 목적지를 반환한다."""
        listener._booking_destinations['task-1'] = 'pantry'
        assert listener.get_final_destination('task-1') == 'pantry'


class TestClearBooking:
    """clear_booking() 테스트."""

    def test_removes_entry(self, listener):
        """Booking 캐시를 정리한다."""
        listener._booking_destinations['task-1'] = 'pantry'
        listener.clear_booking('task-1')
        assert listener.get_final_destination('task-1') is None

    def test_clear_nonexistent_is_safe(self, listener):
        """존재하지 않는 booking 정리는 안전하다."""
        listener.clear_booking('nonexistent')


class TestOnFleetState:
    """_on_fleet_state() 콜백 테스트."""

    def test_extracts_destination_from_path(self, listener):
        """Fleet state의 path[-1].index에서 목적지를 추출한다."""
        last_loc = MagicMock()
        last_loc.index = 2  # index 2 → 'charger1'

        robot_state = MagicMock()
        robot_state.name = 'AGV-001'
        robot_state.task_id = 'ParkRobot-abc123'
        robot_state.path = [MagicMock(), last_loc]

        msg = MagicMock()
        msg.name = 'tinyRobot'
        msg.robots = [robot_state]

        listener._on_fleet_state(msg)

        assert listener.get_final_destination(
            'ParkRobot-abc123'
        ) == 'charger1'

    def test_ignores_different_fleet(self, listener):
        """다른 fleet의 메시지는 무시한다."""
        robot_state = MagicMock()
        robot_state.name = 'AGV-001'
        robot_state.task_id = 'task-1'
        robot_state.path = [MagicMock()]

        msg = MagicMock()
        msg.name = 'otherFleet'
        msg.robots = [robot_state]

        listener._on_fleet_state(msg)

        assert listener.get_final_destination('task-1') is None

    def test_ignores_robot_without_task(self, listener):
        """task_id가 없는 로봇은 무시한다."""
        robot_state = MagicMock()
        robot_state.name = 'AGV-001'
        robot_state.task_id = ''
        robot_state.path = [MagicMock()]

        msg = MagicMock()
        msg.name = 'tinyRobot'
        msg.robots = [robot_state]

        listener._on_fleet_state(msg)

        assert not listener._booking_destinations

    def test_ignores_robot_without_path(self, listener):
        """path가 없는 로봇은 무시한다."""
        robot_state = MagicMock()
        robot_state.name = 'AGV-001'
        robot_state.task_id = 'task-1'
        robot_state.path = []

        msg = MagicMock()
        msg.name = 'tinyRobot'
        msg.robots = [robot_state]

        listener._on_fleet_state(msg)

        assert not listener._booking_destinations

    def test_ignores_out_of_range_index(self, listener):
        """index가 범위를 초과하면 무시한다."""
        last_loc = MagicMock()
        last_loc.index = 99  # out of range

        robot_state = MagicMock()
        robot_state.name = 'AGV-001'
        robot_state.task_id = 'task-1'
        robot_state.path = [last_loc]

        msg = MagicMock()
        msg.name = 'tinyRobot'
        msg.robots = [robot_state]

        listener._on_fleet_state(msg)

        assert not listener._booking_destinations

    def test_does_not_overwrite_existing_mapping(self, listener):
        """이미 매핑된 task_id는 덮어쓰지 않는다."""
        listener._booking_destinations['task-1'] = 'pantry'

        last_loc = MagicMock()
        last_loc.index = 2  # 'charger1'

        robot_state = MagicMock()
        robot_state.name = 'AGV-001'
        robot_state.task_id = 'task-1'
        robot_state.path = [last_loc]

        msg = MagicMock()
        msg.name = 'tinyRobot'
        msg.robots = [robot_state]

        listener._on_fleet_state(msg)

        # task_api correlation takes priority
        assert listener.get_final_destination('task-1') == 'pantry'
