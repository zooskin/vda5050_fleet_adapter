"""주문 처리 유스케이스.

RMF로부터 수신한 내비게이션 명령을 VDA5050 Order로 변환하여
AGV에 전송하는 핵심 로직을 담당한다.
"""

from vda5050_fleet_adapter.domain.entities.edge import Edge
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.entities.node import Node
from vda5050_fleet_adapter.domain.entities.order import Order
from vda5050_fleet_adapter.domain.events.agv_events import (
    OrderReceivedEvent,
    OrderUpdateEvent,
)
from vda5050_fleet_adapter.domain.exceptions import (
    AgvNotFoundError,
    OrderValidationError,
)
from vda5050_fleet_adapter.domain.value_objects.position import NodePosition
from vda5050_fleet_adapter.usecase.ports.agv_gateway import AgvGateway
from vda5050_fleet_adapter.usecase.ports.config_port import AppConfig
from vda5050_fleet_adapter.usecase.ports.event_publisher import EventPublisher
from vda5050_fleet_adapter.usecase.ports.state_repository import StateRepository


class ProcessOrder:
    """주문 처리 유스케이스.

    RMF 내비게이션 명령 → VDA5050 Order 변환 → AGV 전송.

    Args:
        agv_gateway: AGV 통신 포트.
        state_repo: 상태 저장소.
        event_publisher: 이벤트 발행자.
        config: 애플리케이션 설정.
    """

    def __init__(
        self,
        agv_gateway: AgvGateway,
        state_repo: StateRepository,
        event_publisher: EventPublisher,
        config: AppConfig,
    ) -> None:
        self._agv_gateway = agv_gateway
        self._state_repo = state_repo
        self._event_publisher = event_publisher
        self._config = config

    def send_new_order(
        self,
        agv_id: str,
        order_id: str,
        waypoints: list[tuple[float, float, float, str]],
    ) -> Order:
        """새 주문을 생성하여 AGV에 전송한다.

        Args:
            agv_id: 대상 AGV 식별자.
            order_id: 주문 ID.
            waypoints: 경유지 목록 (x, y, theta, map_id).

        Returns:
            생성된 Order 객체.

        Raises:
            AgvNotFoundError: AGV가 등록되어 있지 않을 때.
            OrderValidationError: 주문 유효성 검증 실패 시.
        """
        state = self._state_repo.get_state(agv_id)
        if state is None:
            raise AgvNotFoundError(f"AGV [{agv_id}]가 등록되어 있지 않습니다.")

        nodes, edges = self._build_graph(waypoints)

        order = Order(
            header=self._build_header(agv_id),
            order_id=order_id,
            order_update_id=0,
            nodes=nodes,
            edges=edges,
        )
        order.validate()

        self._agv_gateway.send_order(agv_id, order)
        self._state_repo.save_order(agv_id, order)
        self._event_publisher.publish(
            OrderReceivedEvent(
                agv_id=agv_id,
                order_id=order_id,
                order_update_id=0,
            )
        )

        return order

    def update_order(
        self,
        agv_id: str,
        additional_waypoints: list[tuple[float, float, float, str]],
    ) -> Order:
        """기존 주문에 새 경유지를 추가(업데이트)한다.

        Args:
            agv_id: 대상 AGV 식별자.
            additional_waypoints: 추가 경유지 목록 (x, y, theta, map_id).

        Returns:
            업데이트된 Order 객체.

        Raises:
            AgvNotFoundError: AGV가 등록되어 있지 않을 때.
            OrderValidationError: 현재 주문이 없거나 유효성 검증 실패 시.
        """
        current_order = self._state_repo.get_current_order(agv_id)
        if current_order is None:
            raise OrderValidationError(
                f"AGV [{agv_id}]에 업데이트할 현재 주문이 없습니다."
            )

        stitching_node = current_order.last_base_node
        if stitching_node is None:
            raise OrderValidationError("현재 주문에 Base 노드가 없습니다.")

        start_seq = stitching_node.sequence_id
        nodes, edges = self._build_graph(
            additional_waypoints, start_sequence_id=start_seq
        )
        # stitching node를 새 그래프의 첫 노드로 삽입
        nodes.insert(0, stitching_node)

        new_update_id = current_order.order_update_id + 1
        updated_order = Order(
            header=self._build_header(agv_id),
            order_id=current_order.order_id,
            order_update_id=new_update_id,
            nodes=nodes,
            edges=edges,
        )
        updated_order.validate()

        self._agv_gateway.send_order(agv_id, updated_order)
        self._state_repo.save_order(agv_id, updated_order)
        self._event_publisher.publish(
            OrderUpdateEvent(
                agv_id=agv_id,
                order_id=updated_order.order_id,
                order_update_id=new_update_id,
            )
        )

        return updated_order

    def _build_graph(
        self,
        waypoints: list[tuple[float, float, float, str]],
        start_sequence_id: int = 0,
    ) -> tuple[list[Node], list[Edge]]:
        """경유지 목록으로 노드/엣지 그래프를 생성한다."""
        nodes: list[Node] = []
        edges: list[Edge] = []

        for i, (x, y, theta, map_id) in enumerate(waypoints):
            node_seq = start_sequence_id + (i * 2)
            node = Node(
                node_id=f"node_{node_seq}",
                sequence_id=node_seq,
                released=True,
                node_position=NodePosition(
                    x=x, y=y, theta=theta, map_id=map_id
                ),
            )
            nodes.append(node)

            if i > 0:
                prev_node = nodes[-2]
                edge_seq = node_seq - 1
                edge = Edge(
                    edge_id=f"edge_{edge_seq}",
                    sequence_id=edge_seq,
                    released=True,
                    start_node_id=prev_node.node_id,
                    end_node_id=node.node_id,
                )
                edges.append(edge)

        return nodes, edges

    def _build_header(self, agv_id: str) -> Header:
        """메시지 헤더를 생성한다."""
        return Header(
            version="2.0.0",
            manufacturer=self._config.vda5050.manufacturer,
            serial_number=agv_id,
        )
