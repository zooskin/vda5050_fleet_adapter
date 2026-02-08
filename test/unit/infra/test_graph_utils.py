"""graph_utils 유닛 테스트."""

from vda5050_fleet_adapter.infra.nav_graph.graph_utils import (
    apply_transformations,
    build_vda5050_nodes_edges,
    compute_path,
    compute_transforms,
    create_graph,
    find_edge_name,
    find_nearest_node,
    inverse_transform_coords,
    parse_nav_graph,
    transform_coords,
)
import yaml


class TestParseNavGraph:
    """parse_nav_graph() 테스트."""

    def test_parse_basic_graph(self, tmp_path):
        """기본 nav graph를 파싱한다."""
        nav_data = {
            'levels': {
                'L1': {
                    'vertices': [
                        [0.0, 0.0, {'name': 'wp1'}],
                        [5.0, 0.0, {'name': 'wp2'}],
                        [5.0, 5.0, {'name': 'wp3'}],
                    ],
                    'lanes': [
                        [0, 1, {}],
                        [1, 0, {}],
                        [1, 2, {}],
                        [2, 1, {}],
                    ],
                }
            }
        }
        path = tmp_path / 'nav.yaml'
        with open(path, 'w') as f:
            yaml.dump(nav_data, f)

        nodes, edges = parse_nav_graph(str(path))

        assert len(nodes) == 3
        assert 'wp1' in nodes
        assert 'wp2' in nodes
        assert 'wp3' in nodes
        assert nodes['wp1']['x'] == 0.0
        assert nodes['wp2']['x'] == 5.0
        # lane 1개당 edge 1개 (이미 방향별 분리)
        assert len(edges) == 4

    def test_parse_unnamed_vertices(self, tmp_path):
        """이름 없는 정점은 node0, node1 등으로 생성된다."""
        nav_data = {
            'levels': {
                'L1': {
                    'vertices': [
                        [0.0, 0.0, {}],
                        [1.0, 0.0, {}],
                    ],
                    'lanes': [[0, 1, {}]],
                }
            }
        }
        path = tmp_path / 'nav.yaml'
        with open(path, 'w') as f:
            yaml.dump(nav_data, f)

        nodes, edges = parse_nav_graph(str(path))

        assert 'node0' in nodes
        assert 'node1' in nodes

    def test_parse_empty_name_vertices(self, tmp_path):
        """빈 이름('') 정점은 node{i}로 대체된다."""
        nav_data = {
            'levels': {
                'L1': {
                    'vertices': [
                        [0.0, 0.0, {'name': 'wp1'}],
                        [1.0, 0.0, {'name': ''}],
                        [2.0, 0.0, {'name': ''}],
                    ],
                    'lanes': [
                        [0, 1, {}],
                        [1, 2, {}],
                    ],
                }
            }
        }
        path = tmp_path / 'nav.yaml'
        with open(path, 'w') as f:
            yaml.dump(nav_data, f)

        nodes, edges = parse_nav_graph(str(path))

        assert len(nodes) == 3
        assert 'wp1' in nodes
        assert 'node1' in nodes
        assert 'node2' in nodes
        # lane 인덱스로 정상 참조 가능
        assert len(edges) == 2


class TestCreateGraph:
    """create_graph() 테스트."""

    def test_create_graph_from_fixtures(
        self, sample_nav_nodes, sample_nav_edges
    ):
        """fixture에서 그래프를 생성한다."""
        graph = create_graph(sample_nav_nodes, sample_nav_edges)

        assert graph.number_of_nodes() == 4
        assert graph.number_of_edges() >= 4


class TestComputePath:
    """compute_path() 테스트."""

    def test_direct_path(self, sample_nav_nodes, sample_nav_edges):
        """직접 연결된 노드 간 경로를 찾는다."""
        graph = create_graph(sample_nav_nodes, sample_nav_edges)
        path = compute_path(graph, 'wp1', 'wp2')

        assert path is not None
        assert path[0] == 'wp1'
        assert path[-1] == 'wp2'

    def test_multi_hop_path(self, sample_nav_nodes, sample_nav_edges):
        """여러 홉을 거치는 경로를 찾는다."""
        graph = create_graph(sample_nav_nodes, sample_nav_edges)
        path = compute_path(graph, 'wp1', 'wp3')

        assert path is not None
        assert path[0] == 'wp1'
        assert path[-1] == 'wp3'
        assert len(path) >= 2

    def test_no_path_returns_none(self, sample_nav_nodes):
        """경로가 없으면 None을 반환한다."""
        import networkx as nx
        graph = nx.Graph()
        graph.add_node('isolated')
        graph.add_node('wp1')
        path = compute_path(graph, 'wp1', 'isolated')
        assert path is None

    def test_node_not_found_returns_none(
        self, sample_nav_nodes, sample_nav_edges
    ):
        """존재하지 않는 노드는 None을 반환한다."""
        graph = create_graph(sample_nav_nodes, sample_nav_edges)
        path = compute_path(graph, 'wp1', 'nonexistent')
        assert path is None


class TestFindNearestNode:
    """find_nearest_node() 테스트."""

    def test_exact_match(self, sample_nav_nodes):
        """정확한 위치의 노드를 찾는다."""
        result = find_nearest_node(sample_nav_nodes, 0.0, 0.0)
        assert result == 'wp1'

    def test_nearest_to_wp2(self, sample_nav_nodes):
        """wp2에 가장 가까운 노드를 찾는다."""
        result = find_nearest_node(sample_nav_nodes, 4.9, 0.1)
        assert result == 'wp2'

    def test_empty_nodes(self):
        """빈 노드 목록에서 None을 반환한다."""
        result = find_nearest_node({}, 0.0, 0.0)
        assert result is None


class TestBuildVda5050NodesEdges:
    """build_vda5050_nodes_edges() 테스트."""

    def test_single_node_path(self, sample_nav_nodes):
        """단일 노드 경로를 생성한다."""
        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            ['wp1'], sample_nav_nodes, 'map1'
        )

        assert len(vda_nodes) == 1
        assert len(vda_edges) == 0
        assert vda_nodes[0].node_id == 'wp1'
        assert vda_nodes[0].sequence_id == 0

    def test_two_node_path(self, sample_nav_nodes):
        """2노드 경로를 생성한다."""
        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            ['wp1', 'wp2'], sample_nav_nodes, 'map1'
        )

        assert len(vda_nodes) == 2
        assert len(vda_edges) == 1
        assert vda_nodes[0].sequence_id == 0
        assert vda_nodes[1].sequence_id == 2
        assert vda_edges[0].sequence_id == 1
        assert vda_edges[0].start_node_id == 'wp1'
        assert vda_edges[0].end_node_id == 'wp2'

    def test_node_positions_are_set(self, sample_nav_nodes):
        """노드 위치가 설정된다."""
        vda_nodes, _ = build_vda5050_nodes_edges(
            ['wp1'], sample_nav_nodes, 'map1'
        )

        pos = vda_nodes[0].node_position
        assert pos is not None
        assert pos.x == 0.0
        assert pos.y == 0.0
        assert pos.map_id == 'map1'

    def test_all_nodes_released(self, sample_nav_nodes):
        """base_end_index 미지정 시 모든 노드가 released=True이다."""
        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            ['wp1', 'wp2', 'wp3'], sample_nav_nodes, 'map1'
        )

        for node in vda_nodes:
            assert node.released is True
        for edge in vda_edges:
            assert edge.released is True

    def test_base_horizon_split(self, sample_nav_nodes):
        """base_end_index로 Base/Horizon이 올바르게 분리된다."""
        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            ['wp1', 'wp2', 'wp3', 'wp4'], sample_nav_nodes, 'map1',
            base_end_index=1,
        )

        # 노드: index 0,1 → Base, index 2,3 → Horizon
        assert vda_nodes[0].released is True   # wp1 (index 0)
        assert vda_nodes[1].released is True   # wp2 (index 1)
        assert vda_nodes[2].released is False  # wp3 (index 2)
        assert vda_nodes[3].released is False  # wp4 (index 3)

        # 엣지: index 0 → Base (0 < 1), index 1,2 → Horizon
        assert vda_edges[0].released is True   # wp1→wp2 (i=0)
        assert vda_edges[1].released is False  # wp2→wp3 (i=1)
        assert vda_edges[2].released is False  # wp3→wp4 (i=2)

    def test_base_end_index_none_defaults_all_base(self, sample_nav_nodes):
        """base_end_index=None이면 전부 Base이다 (기존 동작 보장)."""
        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            ['wp1', 'wp2', 'wp3'], sample_nav_nodes, 'map1',
            base_end_index=None,
        )

        for node in vda_nodes:
            assert node.released is True
        for edge in vda_edges:
            assert edge.released is True

    def test_base_end_index_at_last_node(self, sample_nav_nodes):
        """base_end_index가 마지막 노드이면 전부 Base이다."""
        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            ['wp1', 'wp2', 'wp3'], sample_nav_nodes, 'map1',
            base_end_index=2,
        )

        for node in vda_nodes:
            assert node.released is True
        for edge in vda_edges:
            assert edge.released is True

    def test_seq_start_offset(self, sample_nav_nodes):
        """시작 시퀀스 오프셋이 적용된다."""
        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            ['wp1', 'wp2'], sample_nav_nodes, 'map1', seq_start=4
        )

        assert vda_nodes[0].sequence_id == 4
        assert vda_edges[0].sequence_id == 5
        assert vda_nodes[1].sequence_id == 6


class TestTransformCoords:
    """좌표 변환 테스트."""

    def test_identity_transform(self):
        """항등 변환."""
        x, y, theta = transform_coords(1.0, 2.0, 0.5, 0.0, 1.0, [0.0, 0.0])
        assert abs(x - 1.0) < 1e-10
        assert abs(y - 2.0) < 1e-10
        assert abs(theta - 0.5) < 1e-10

    def test_translation_only(self):
        """이동 변환만."""
        x, y, theta = transform_coords(
            1.0, 2.0, 0.0, 0.0, 1.0, [10.0, 20.0]
        )
        assert abs(x - 11.0) < 1e-10
        assert abs(y - 22.0) < 1e-10

    def test_scale_only(self):
        """스케일 변환만."""
        x, y, theta = transform_coords(
            1.0, 2.0, 0.0, 0.0, 2.0, [0.0, 0.0]
        )
        assert abs(x - 2.0) < 1e-10
        assert abs(y - 4.0) < 1e-10

    def test_inverse_transform_roundtrip(self):
        """정변환 → 역변환 시 원래 좌표로 복원된다."""
        x0, y0, theta0 = 3.0, 4.0, 1.0
        rotation = 0.5
        scale = 1.5
        translation = [10.0, 20.0]

        x1, y1, theta1 = transform_coords(
            x0, y0, theta0, rotation, scale, translation
        )
        x2, y2, theta2 = inverse_transform_coords(
            x1, y1, theta1, rotation, scale, translation
        )

        assert abs(x2 - x0) < 1e-10
        assert abs(y2 - y0) < 1e-10
        assert abs(theta2 - theta0) < 1e-10


class TestComputeTransforms:
    """compute_transforms() 테스트."""

    def test_identity_transform(self):
        """동일 좌표에 대해 항등 변환을 계산한다."""
        rmf = [[0, 0], [1, 0], [0, 1], [1, 1]]
        robot = [[0, 0], [1, 0], [0, 1], [1, 1]]
        tf = compute_transforms(rmf, robot)

        # 동일 좌표이므로 변환이 거의 항등
        assert abs(tf.get_rotation()) < 1e-10
        assert abs(tf.get_scale() - 1.0) < 1e-10


class TestFindEdgeName:
    """find_edge_name() 테스트."""

    def test_find_existing_edge(self, sample_nav_edges):
        """존재하는 엣지를 찾는다."""
        result = find_edge_name(sample_nav_edges, 'wp1', 'wp2')
        assert result is not None

    def test_find_nonexistent_edge(self, sample_nav_edges):
        """존재하지 않는 엣지에 대해 None을 반환한다."""
        result = find_edge_name(sample_nav_edges, 'wp1', 'wp3')
        assert result is None


class TestApplyTransformations:
    """apply_transformations() 테스트."""

    def test_apply_translation(self):
        """이동 변환이 모든 노드에 적용된다."""
        nodes = {
            'a': {'x': 0.0, 'y': 0.0, 'attributes': {}},
            'b': {'x': 1.0, 'y': 0.0, 'attributes': {}},
        }
        apply_transformations(nodes, 0.0, 1.0, [10.0, 20.0])

        assert abs(nodes['a']['x'] - 10.0) < 1e-10
        assert abs(nodes['a']['y'] - 20.0) < 1e-10
        assert abs(nodes['b']['x'] - 11.0) < 1e-10
        assert abs(nodes['b']['y'] - 20.0) < 1e-10
