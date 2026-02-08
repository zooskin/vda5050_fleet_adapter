"""Vda5050RobotAPI 유닛 테스트."""

import json
from unittest.mock import MagicMock

import pytest

from vda5050_fleet_adapter.domain.entities.battery import BatteryState
from vda5050_fleet_adapter.domain.entities.edge import Edge
from vda5050_fleet_adapter.domain.entities.node import Node
from vda5050_fleet_adapter.domain.enums import ActionStatus
from vda5050_fleet_adapter.domain.value_objects.position import (
    AgvPosition,
    NodePosition,
)
from vda5050_fleet_adapter.infra.mqtt.vda5050_robot_api import (
    Vda5050RobotAPI,
)
from vda5050_fleet_adapter.usecase.ports.robot_api import RobotAPIResult


@pytest.fixture
def mock_mqtt():
    """Mock MqttClient."""
    mqtt = MagicMock()
    mqtt.is_connected = True
    return mqtt


@pytest.fixture
def api(mock_mqtt):
    """Vda5050RobotAPI instance with mock MQTT."""
    return Vda5050RobotAPI(
        mqtt_client=mock_mqtt,
        prefix='uagv/v2/TestCo',
        manufacturer='TestCo',
    )


class TestNavigate:
    """navigate() 테스트."""

    def test_navigate_publishes_order(self, api, mock_mqtt):
        """navigate가 MQTT로 Order를 발행한다."""
        nodes = [
            Node(
                node_id='wp1', sequence_id=0, released=True,
                node_position=NodePosition(x=0.0, y=0.0, map_id='map1'),
            ),
            Node(
                node_id='wp2', sequence_id=2, released=True,
                node_position=NodePosition(x=5.0, y=0.0, map_id='map1'),
            ),
        ]
        edges = [
            Edge(
                edge_id='e1', sequence_id=1, released=True,
                start_node_id='wp1', end_node_id='wp2',
            ),
        ]

        result = api.navigate('AGV-001', 1, nodes, edges, 'map1')

        assert result == RobotAPIResult.SUCCESS
        mock_mqtt.publish.assert_called_once()
        topic = mock_mqtt.publish.call_args[0][0]
        assert topic == 'uagv/v2/TestCo/AGV-001/order'
        payload = mock_mqtt.publish.call_args[0][1]
        data = json.loads(payload)
        assert data['orderId'].startswith('order_1_')
        assert len(data['nodes']) == 2
        assert len(data['edges']) == 1

    def test_navigate_uses_qos_0(self, api, mock_mqtt):
        """navigate가 QoS 0을 사용한다."""
        nodes = [
            Node(
                node_id='wp1', sequence_id=0, released=True,
                node_position=NodePosition(x=0.0, y=0.0, map_id='map1'),
            ),
        ]
        api.navigate('AGV-001', 1, nodes, [], 'map1')

        assert mock_mqtt.publish.call_args[1].get('qos', 0) == 0

    def test_navigate_retries_when_disconnected(self, api, mock_mqtt):
        """MQTT 미연결 시 RETRY를 반환한다."""
        mock_mqtt.is_connected = False
        result = api.navigate('AGV-001', 1, [], [], 'map1')
        assert result == RobotAPIResult.RETRY

    def test_navigate_stores_cmd_order_mapping(self, api, mock_mqtt):
        """navigate가 cmd_id → order_id 매핑을 저장한다."""
        nodes = [
            Node(
                node_id='wp1', sequence_id=0, released=True,
                node_position=NodePosition(x=0.0, y=0.0, map_id='map1'),
            ),
        ]
        api.navigate('AGV-001', 42, nodes, [], 'map1')
        assert 42 in api._cmd_order_map.get('AGV-001', {})

    def test_navigate_uses_provided_order_id(self, api, mock_mqtt):
        """제공된 order_id를 사용한다."""
        nodes = [
            Node(
                node_id='wp1', sequence_id=0, released=True,
                node_position=NodePosition(x=0.0, y=0.0, map_id='map1'),
            ),
        ]
        result = api.navigate(
            'AGV-001', 1, nodes, [], 'map1',
            order_id='custom_order_123',
        )

        assert result == RobotAPIResult.SUCCESS
        payload = mock_mqtt.publish.call_args[0][1]
        data = json.loads(payload)
        assert data['orderId'] == 'custom_order_123'

    def test_navigate_auto_generates_order_id_when_empty(self, api, mock_mqtt):
        """order_id가 빈 문자열이면 자동 생성한다."""
        nodes = [
            Node(
                node_id='wp1', sequence_id=0, released=True,
                node_position=NodePosition(x=0.0, y=0.0, map_id='map1'),
            ),
        ]
        api.navigate('AGV-001', 5, nodes, [], 'map1', order_id='')

        payload = mock_mqtt.publish.call_args[0][1]
        data = json.loads(payload)
        assert data['orderId'].startswith('order_5_')

    def test_navigate_uses_provided_order_update_id(self, api, mock_mqtt):
        """order_update_id가 payload에 반영된다."""
        nodes = [
            Node(
                node_id='wp1', sequence_id=0, released=True,
                node_position=NodePosition(x=0.0, y=0.0, map_id='map1'),
            ),
        ]
        api.navigate(
            'AGV-001', 1, nodes, [], 'map1',
            order_id='order_1_abc', order_update_id=3,
        )

        payload = mock_mqtt.publish.call_args[0][1]
        data = json.loads(payload)
        assert data['orderUpdateId'] == 3


class TestStop:
    """stop() 테스트."""

    def test_stop_publishes_cancel_order(self, api, mock_mqtt):
        """stop이 cancelOrder instant action을 발행한다."""
        result = api.stop('AGV-001', 1)

        assert result == RobotAPIResult.SUCCESS
        mock_mqtt.publish.assert_called_once()
        topic = mock_mqtt.publish.call_args[0][0]
        assert topic == 'uagv/v2/TestCo/AGV-001/instantActions'
        payload = mock_mqtt.publish.call_args[0][1]
        data = json.loads(payload)
        assert data['actions'][0]['actionType'] == 'cancelOrder'

    def test_stop_retries_when_disconnected(self, api, mock_mqtt):
        """MQTT 미연결 시 RETRY를 반환한다."""
        mock_mqtt.is_connected = False
        result = api.stop('AGV-001', 1)
        assert result == RobotAPIResult.RETRY


class TestStartActivity:
    """start_activity() 테스트."""

    def test_start_activity_publishes_action(self, api, mock_mqtt):
        """start_activity가 instant action을 발행한다."""
        result = api.start_activity(
            'AGV-001', 1, 'charge', {'station': 'cs1'}
        )

        assert result == RobotAPIResult.SUCCESS
        mock_mqtt.publish.assert_called_once()
        topic = mock_mqtt.publish.call_args[0][0]
        assert topic == 'uagv/v2/TestCo/AGV-001/instantActions'
        payload = mock_mqtt.publish.call_args[0][1]
        data = json.loads(payload)
        assert data['actions'][0]['actionType'] == 'charge'
        params = data['actions'][0]['actionParameters']
        assert any(p['key'] == 'station' for p in params)


class TestGetData:
    """get_data() 테스트."""

    def test_get_data_returns_none_without_state(self, api):
        """상태 미수신 시 None을 반환한다."""
        assert api.get_data('AGV-001') is None

    def test_get_data_returns_cached_state(self, api):
        """캐시된 상태를 반환한다."""
        state = MagicMock()
        state.agv_position = AgvPosition(
            x=1.0, y=2.0, theta=0.5, map_id='map1'
        )
        state.battery_state = BatteryState(battery_charge=80.0)
        api._state_cache['AGV-001'] = state

        data = api.get_data('AGV-001')
        assert data is not None
        assert data.robot_name == 'AGV-001'
        assert data.map_name == 'map1'
        assert data.position == [1.0, 2.0, 0.5]
        assert abs(data.battery_soc - 0.8) < 0.01


class TestIsCommandCompleted:
    """is_command_completed() 테스트."""

    def test_returns_false_without_state(self, api):
        """상태 미수신 시 False를 반환한다."""
        assert not api.is_command_completed('AGV-001', 1)

    def test_order_completed_when_no_nodes_and_not_driving(self, api):
        """nodeStates가 비고 driving=False이면 완료."""
        order_id = 'order_1_abc'
        api._cmd_order_map['AGV-001'] = {1: order_id}

        state = MagicMock()
        state.order_id = order_id
        state.node_states = []
        state.driving = False
        state.action_states = []
        api._state_cache['AGV-001'] = state

        assert api.is_command_completed('AGV-001', 1)

    def test_order_not_completed_when_driving(self, api):
        """driving=True이면 미완료."""
        order_id = 'order_1_abc'
        api._cmd_order_map['AGV-001'] = {1: order_id}

        state = MagicMock()
        state.order_id = order_id
        state.node_states = []
        state.driving = True
        state.action_states = []
        api._state_cache['AGV-001'] = state

        assert not api.is_command_completed('AGV-001', 1)

    def test_order_not_completed_when_nodes_remain(self, api):
        """남은 노드가 있으면 미완료."""
        order_id = 'order_1_abc'
        api._cmd_order_map['AGV-001'] = {1: order_id}

        state = MagicMock()
        state.order_id = order_id
        state.node_states = [MagicMock()]
        state.driving = False
        state.action_states = []
        api._state_cache['AGV-001'] = state

        assert not api.is_command_completed('AGV-001', 1)

    def test_action_completed_when_finished(self, api):
        """Action status가 FINISHED이면 완료."""
        action_id = 'charge_1_abc'
        api._cmd_order_map['AGV-001'] = {1: action_id}

        action_state = MagicMock()
        action_state.action_id = action_id
        action_state.action_status = ActionStatus.FINISHED

        state = MagicMock()
        state.order_id = ''
        state.node_states = []
        state.driving = False
        state.action_states = [action_state]
        api._state_cache['AGV-001'] = state

        assert api.is_command_completed('AGV-001', 1)

    def test_action_completed_when_failed(self, api):
        """Action status가 FAILED이면 완료."""
        action_id = 'charge_1_abc'
        api._cmd_order_map['AGV-001'] = {1: action_id}

        action_state = MagicMock()
        action_state.action_id = action_id
        action_state.action_status = ActionStatus.FAILED

        state = MagicMock()
        state.order_id = ''
        state.node_states = []
        state.driving = False
        state.action_states = [action_state]
        api._state_cache['AGV-001'] = state

        assert api.is_command_completed('AGV-001', 1)


class TestOnStateMessage:
    """_on_state_message() 테스트."""

    def test_state_message_updates_cache(self, api):
        """State 메시지 수신 시 캐시가 업데이트된다."""
        state_json = json.dumps({
            'headerId': 1,
            'timestamp': '2026-01-01T00:00:00.000Z',
            'version': '2.0.0',
            'manufacturer': 'TestCo',
            'serialNumber': 'AGV-001',
            'orderId': 'order-001',
            'orderUpdateId': 0,
            'lastNodeId': 'wp1',
            'lastNodeSequenceId': 0,
            'driving': True,
            'operatingMode': 'AUTOMATIC',
            'nodeStates': [],
            'edgeStates': [],
            'actionStates': [],
            'errors': [],
            'safetyState': {'eStop': 'NONE', 'fieldViolation': False},
            'batteryState': {'batteryCharge': 80.0, 'charging': False},
            'agvPosition': {
                'x': 1.0, 'y': 2.0, 'theta': 0.5,
                'mapId': 'map1', 'positionInitialized': True,
            },
        })

        api._on_state_message('AGV-001', state_json.encode('utf-8'))

        assert 'AGV-001' in api._state_cache
        cached = api._state_cache['AGV-001']
        assert cached.order_id == 'order-001'
        assert cached.driving is True


class TestSubscribeRobot:
    """subscribe_robot() 테스트."""

    def test_subscribes_state_and_connection(self, api, mock_mqtt):
        """로봇 구독 시 state와 connection 토픽을 구독한다."""
        api.subscribe_robot('AGV-001')

        assert mock_mqtt.subscribe.call_count == 2
        topics = [c[0][0] for c in mock_mqtt.subscribe.call_args_list]
        assert 'uagv/v2/TestCo/AGV-001/state' in topics
        assert 'uagv/v2/TestCo/AGV-001/connection' in topics


class TestBuildTopic:
    """_build_topic() 테스트."""

    def test_build_topic_format(self, api):
        """토픽 형식이 올바르다."""
        topic = api._build_topic('AGV-001', 'order')
        assert topic == 'uagv/v2/TestCo/AGV-001/order'

    def test_build_topic_state(self, api):
        """State 토픽 형식이 올바르다."""
        topic = api._build_topic('AGV-001', 'state')
        assert topic == 'uagv/v2/TestCo/AGV-001/state'
