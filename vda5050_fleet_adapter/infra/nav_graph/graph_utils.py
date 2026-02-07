"""내비게이션 그래프 파싱 및 좌표 변환 유틸리티.

RMF nav graph 파싱, 좌표 변환, 경로 탐색,
VDA5050 Node/Edge 생성 기능을 제공한다.
"""

from __future__ import annotations

import logging
import math
from typing import Any
import uuid

import networkx as nx
import nudged
from vda5050_fleet_adapter.domain.entities.edge import Edge
from vda5050_fleet_adapter.domain.entities.node import Node
from vda5050_fleet_adapter.domain.value_objects.position import NodePosition
import yaml

logger = logging.getLogger(__name__)


def parse_nav_graph(
    nav_graph_path: str, level: str = 'L1'
) -> tuple[dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    """RMF 내비게이션 그래프 파일을 파싱한다.

    Args:
        nav_graph_path: nav graph YAML 파일 경로.
        level: 사용할 레벨 이름.

    Returns:
        (nodes, edges) 튜플.
        nodes: {name: {x, y, attributes}} 형태.
        edges: {name: {start, end, attributes}} 형태.
    """
    with open(nav_graph_path, 'r') as f:
        nav_graph = yaml.safe_load(f)

    vertices = nav_graph['levels'][level]['vertices']
    lanes = nav_graph['levels'][level]['lanes']

    nodes: dict[str, dict[str, Any]] = {}
    for i, vertex in enumerate(vertices):
        name = vertex[2].get('name', f'node{i}') if len(vertex) > 2 else f'node{i}'
        nodes[name] = {
            'x': float(vertex[0]),
            'y': float(vertex[1]),
            'attributes': vertex[2] if len(vertex) > 2 else {},
        }

    edges: dict[str, dict[str, Any]] = {}
    node_names = list(nodes.keys())
    for i, lane in enumerate(lanes):
        start_node = node_names[lane[0]]
        end_node = node_names[lane[1]]
        attrs = lane[2] if len(lane) > 2 else {}
        edge_name_a = f'edge{i}_a'
        edges[edge_name_a] = {
            'start': start_node,
            'end': end_node,
            'attributes': attrs,
        }
        edge_name_b = f'edge{i}_b'
        edges[edge_name_b] = {
            'start': end_node,
            'end': start_node,
            'attributes': attrs,
        }

    logger.info(
        'Parsed nav graph: %d nodes, %d edges', len(nodes), len(edges)
    )
    return nodes, edges


def compute_transforms(
    rmf_coords: list[list[float]],
    robot_coords: list[list[float]],
) -> nudged.Transform:
    """RMF ↔ 로봇 좌표계 변환을 계산한다.

    Args:
        rmf_coords: RMF 좌표 기준점 [[x,y], ...].
        robot_coords: 로봇 좌표 기준점 [[x,y], ...].

    Returns:
        nudged Transform 객체 (rmf → robot).
    """
    tf = nudged.estimate(rmf_coords, robot_coords)
    mse = nudged.estimate_error(tf, rmf_coords, robot_coords)
    logger.info('Coordinate transform MSE: %.6f', mse)
    return tf


def transform_coords(
    x: float, y: float, theta: float,
    rotation: float, scale: float, translation: list[float],
) -> tuple[float, float, float]:
    """RMF 좌표를 로봇 좌표로 변환한다.

    Args:
        x: RMF x 좌표.
        y: RMF y 좌표.
        theta: RMF 방향 (rad).
        rotation: 회전 각도 (rad).
        scale: 스케일 팩터.
        translation: [tx, ty] 이동 벡터.

    Returns:
        (x', y', theta') 변환된 좌표.
    """
    cos_r = math.cos(rotation)
    sin_r = math.sin(rotation)
    x_rot = x * cos_r - y * sin_r
    y_rot = x * sin_r + y * cos_r
    x_out = x_rot * scale + translation[0]
    y_out = y_rot * scale + translation[1]
    theta_out = theta + rotation
    return x_out, y_out, theta_out


def inverse_transform_coords(
    x: float, y: float, theta: float,
    rotation: float, scale: float, translation: list[float],
) -> tuple[float, float, float]:
    """로봇 좌표를 RMF 좌표로 역변환한다.

    Args:
        x: 로봇 x 좌표.
        y: 로봇 y 좌표.
        theta: 로봇 방향 (rad).
        rotation: 회전 각도 (rad).
        scale: 스케일 팩터.
        translation: [tx, ty] 이동 벡터.

    Returns:
        (x', y', theta') RMF 좌표.
    """
    x_t = x - translation[0]
    y_t = y - translation[1]
    x_s = x_t / scale if scale != 0 else x_t
    y_s = y_t / scale if scale != 0 else y_t
    cos_r = math.cos(-rotation)
    sin_r = math.sin(-rotation)
    x_out = x_s * cos_r - y_s * sin_r
    y_out = x_s * sin_r + y_s * cos_r
    theta_out = theta - rotation
    return x_out, y_out, theta_out


def apply_transformations(
    nodes: dict[str, dict[str, Any]],
    rotation: float,
    scale: float,
    translation: list[float],
) -> None:
    """노드 좌표에 변환을 적용한다 (in-place).

    Args:
        nodes: 노드 dict (parse_nav_graph 반환값).
        rotation: 회전 각도 (rad).
        scale: 스케일 팩터.
        translation: [tx, ty] 이동 벡터.
    """
    for name, node in nodes.items():
        x_new, y_new, _ = transform_coords(
            node['x'], node['y'], 0.0, rotation, scale, translation
        )
        node['x'] = x_new
        node['y'] = y_new
        logger.debug(
            'Transformed node %s: x=%.3f, y=%.3f', name, x_new, y_new
        )


def create_graph(
    nodes: dict[str, dict[str, Any]],
    edges: dict[str, dict[str, Any]],
) -> nx.Graph:
    """Networkx 그래프를 생성한다.

    Args:
        nodes: 노드 dict.
        edges: 엣지 dict.

    Returns:
        가중치 그래프.
    """
    graph = nx.Graph()
    for edge_name, edge in edges.items():
        start = edge['start']
        end = edge['end']
        dx = nodes[start]['x'] - nodes[end]['x']
        dy = nodes[start]['y'] - nodes[end]['y']
        weight = math.sqrt(dx * dx + dy * dy)
        graph.add_edge(start, end, weight=weight)
    return graph


def find_nearest_node(
    nodes: dict[str, dict[str, Any]], x: float, y: float
) -> str | None:
    """가장 가까운 노드를 찾는다.

    Args:
        nodes: 노드 dict.
        x: x 좌표.
        y: y 좌표.

    Returns:
        가장 가까운 노드 이름 또는 None.
    """
    nearest: str | None = None
    min_dist = float('inf')
    for name, node in nodes.items():
        dx = x - node['x']
        dy = y - node['y']
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < min_dist:
            min_dist = dist
            nearest = name
    return nearest


def compute_path(
    graph: nx.Graph, start: str, goal: str
) -> list[str] | None:
    """최단 경로를 계산한다.

    Args:
        graph: networkx 그래프.
        start: 시작 노드 이름.
        goal: 목표 노드 이름.

    Returns:
        노드 이름 리스트 또는 경로 없으면 None.
    """
    try:
        return nx.shortest_path(graph, source=start, target=goal, weight='weight')
    except nx.NetworkXNoPath:
        logger.warning('No path from %s to %s', start, goal)
        return None
    except nx.NodeNotFound as e:
        logger.warning('Node not found: %s', e)
        return None


def find_edge_name(
    edges: dict[str, dict[str, Any]],
    start_node: str,
    end_node: str,
) -> str | None:
    """두 노드 사이의 엣지 이름을 찾는다.

    Args:
        edges: 엣지 dict.
        start_node: 시작 노드.
        end_node: 종료 노드.

    Returns:
        엣지 이름 또는 None.
    """
    for edge_name, edge in edges.items():
        if edge['start'] == start_node and edge['end'] == end_node:
            return edge_name
    return None


def build_vda5050_nodes_edges(
    path: list[str],
    nodes: dict[str, dict[str, Any]],
    map_id: str,
    seq_start: int = 0,
) -> tuple[list[Node], list[Edge]]:
    """경로에서 VDA5050 Node/Edge 목록을 생성한다.

    Args:
        path: 노드 이름 리스트 (경로).
        nodes: 노드 dict (좌표 포함).
        map_id: 맵 ID.
        seq_start: 시작 시퀀스 번호.

    Returns:
        (vda_nodes, vda_edges) 튜플.
    """
    vda_nodes: list[Node] = []
    vda_edges: list[Edge] = []

    for i, node_name in enumerate(path):
        seq = seq_start + i * 2
        nd = nodes[node_name]
        vda_nodes.append(
            Node(
                node_id=node_name,
                sequence_id=seq,
                released=True,
                node_position=NodePosition(
                    x=nd['x'],
                    y=nd['y'],
                    map_id=map_id,
                ),
            )
        )

    for i in range(len(path) - 1):
        seq = seq_start + i * 2 + 1
        edge_id = f'{path[i]}_{path[i + 1]}_{uuid.uuid4().hex[:8]}'
        vda_edges.append(
            Edge(
                edge_id=edge_id,
                sequence_id=seq,
                released=True,
                start_node_id=path[i],
                end_node_id=path[i + 1],
            )
        )

    return vda_nodes, vda_edges
