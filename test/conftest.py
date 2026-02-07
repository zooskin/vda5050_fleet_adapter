"""공통 테스트 fixture."""

import pytest

from vda5050_fleet_adapter.domain.entities.action import (
    Action,
    ActionParameter,
    ActionState,
)
from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.entities.battery import BatteryState
from vda5050_fleet_adapter.domain.entities.connection import Connection
from vda5050_fleet_adapter.domain.entities.edge import Edge
from vda5050_fleet_adapter.domain.entities.error import AgvError, ErrorReference
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.entities.load import Load
from vda5050_fleet_adapter.domain.entities.map_state import AgvMap
from vda5050_fleet_adapter.domain.entities.node import Node, NodeState
from vda5050_fleet_adapter.domain.entities.order import Order
from vda5050_fleet_adapter.domain.entities.safety import SafetyState
from vda5050_fleet_adapter.domain.enums import (
    ActionStatus,
    BlockingType,
    ConnectionState,
    ErrorLevel,
    EStopType,
    MapStatus,
    OperatingMode,
)
from vda5050_fleet_adapter.domain.value_objects.position import (
    AgvPosition,
    NodePosition,
    Velocity,
)
from vda5050_fleet_adapter.usecase.ports.config_port import (
    AdapterConfig,
    AppConfig,
    MqttConfig,
    Vda5050Config,
)


@pytest.fixture
def sample_header():
    return Header(
        version="2.0.0",
        manufacturer="TestCo",
        serial_number="AGV-001",
    )


@pytest.fixture
def sample_node_position():
    return NodePosition(x=1.0, y=2.0, map_id="map1", theta=0.5)


@pytest.fixture
def sample_nodes(sample_node_position):
    return [
        Node(
            node_id="n0",
            sequence_id=0,
            released=True,
            node_position=sample_node_position,
        ),
        Node(
            node_id="n2",
            sequence_id=2,
            released=True,
            node_position=NodePosition(x=5.0, y=6.0, map_id="map1"),
        ),
        Node(
            node_id="n4",
            sequence_id=4,
            released=False,
            node_position=NodePosition(x=10.0, y=12.0, map_id="map1"),
        ),
    ]


@pytest.fixture
def sample_edges():
    return [
        Edge(
            edge_id="e1",
            sequence_id=1,
            released=True,
            start_node_id="n0",
            end_node_id="n2",
            max_speed=2.0,
        ),
        Edge(
            edge_id="e3",
            sequence_id=3,
            released=False,
            start_node_id="n2",
            end_node_id="n4",
        ),
    ]


@pytest.fixture
def sample_order(sample_header, sample_nodes, sample_edges):
    return Order(
        header=sample_header,
        order_id="order-001",
        order_update_id=0,
        nodes=sample_nodes,
        edges=sample_edges,
    )


@pytest.fixture
def sample_agv_position():
    return AgvPosition(
        x=1.5, y=2.5, theta=0.3, map_id="map1",
        position_initialized=True, localization_score=0.95,
    )


@pytest.fixture
def sample_agv_state(sample_header, sample_agv_position):
    return AgvState(
        header=sample_header,
        order_id="order-001",
        order_update_id=0,
        last_node_id="n0",
        last_node_sequence_id=0,
        driving=True,
        operating_mode=OperatingMode.AUTOMATIC,
        agv_position=sample_agv_position,
        velocity=Velocity(vx=1.0),
        battery_state=BatteryState(battery_charge=85.0, charging=False),
        safety_state=SafetyState(e_stop=EStopType.NONE),
        node_states=[
            NodeState(node_id="n2", sequence_id=2, released=True),
        ],
        action_states=[
            ActionState(
                action_id="a1",
                action_type="pick",
                action_status=ActionStatus.WAITING,
            ),
        ],
    )


@pytest.fixture
def sample_config():
    return AppConfig(
        mqtt=MqttConfig(broker_host="localhost", broker_port=1883),
        vda5050=Vda5050Config(
            interface_name="uagv",
            protocol_version="v2",
            manufacturer="TestCo",
        ),
        adapter=AdapterConfig(fleet_name="test_fleet"),
    )
