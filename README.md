# VDA5050 Fleet Adapter

VDA5050 표준을 따르는 AGV와 Open-RMF를 연결하는 Fleet Adapter.
AGV와는 MQTT(VDA5050)로, RMF와는 `rmf_fleet_adapter_python`(`rmf_easy`) API로 통신한다.

| 항목 | 내용 |
|------|------|
| Python | 3.12+ |
| ROS 2 | Jazzy Jalisco (Ubuntu 24.04) |
| 아키텍처 | Clean Architecture (4 레이어) |
| VDA5050 | v2.0 |
| RMF API | `rmf_easy` (FleetConfiguration, RobotCallbacks) |

## 디렉토리 구조

```
vda5050_fleet_adapter/
│
├── CLAUDE.md                              # Claude Code 프로젝트 규칙
├── CODING_RULES.md                        # 코딩 규칙 & 아키텍처 가이드
├── VDA5050_PROTOCOL.md                    # VDA5050 프로토콜 정의서
├── package.xml                            # ROS 2 패키지 매니페스트
├── setup.py
│
├── vda5050_fleet_adapter/                 # 메인 패키지
│   │
│   ├── domain/                            # ① 도메인 (순수 Python, 외부 의존성 없음)
│   │   ├── enums.py                       #    StrEnum 9개 (OperatingMode, ActionStatus 등)
│   │   ├── exceptions.py                  #    도메인 예외
│   │   ├── value_objects/                 #    불변 값 객체 (frozen dataclass)
│   │   │   ├── position.py                #      NodePosition, AgvPosition, Velocity
│   │   │   ├── physical.py                #      BoundingBoxReference, LoadDimensions, Corridor
│   │   │   └── trajectory.py              #      ControlPoint, Trajectory (NURBS)
│   │   └── entities/                      #    도메인 엔티티
│   │       ├── header.py                  #      공통 메시지 헤더
│   │       ├── action.py                  #      Action, ActionState (상태 전이 머신)
│   │       ├── node.py                    #      Node, NodeState
│   │       ├── edge.py                    #      Edge, EdgeState
│   │       ├── order.py                   #      Order (validate, Base/Horizon 분리)
│   │       ├── agv_state.py               #      AgvState (전체 AGV 상태 통합)
│   │       ├── error.py                   #      AgvError, AgvInformation
│   │       ├── battery.py                 #      BatteryState
│   │       ├── safety.py                  #      SafetyState
│   │       ├── load.py                    #      Load
│   │       ├── map_state.py               #      AgvMap
│   │       └── connection.py              #      Connection
│   │
│   ├── usecase/                           # ② 유스케이스
│   │   ├── robot_adapter.py               #    RMF↔VDA5050 브릿지 (navigate, stop, action)
│   │   └── ports/                         #    포트 인터페이스 (ABC)
│   │       ├── robot_api.py               #      RobotAPI (navigate, stop, start_activity 등)
│   │       └── config_port.py             #      ConfigPort + MqttConfig, FleetManagerConfig
│   │
│   ├── infra/                             # ③ 인프라 (외부 의존성 격리)
│   │   ├── mqtt/                          #    MQTT/VDA5050 통신
│   │   │   ├── message_serializer.py      #      JSON ↔ 도메인 (camelCase 변환)
│   │   │   ├── mqtt_client.py             #      paho-mqtt 래퍼 (재연결, Last Will)
│   │   │   └── vda5050_robot_api.py       #      RobotAPI 구현체 (Order/InstantActions 발행)
│   │   ├── nav_graph/                     #    내비게이션 그래프
│   │   │   └── graph_utils.py             #      파싱, 경로 탐색, 좌표 변환
│   │   └── config/
│   │       └── yaml_config_loader.py      #      ConfigPort 구현체
│   │
│   ├── presentation/                      # ④ 프레젠테이션 (진입점)
│   │   └── main.py                        #    Entry point (rmf_easy 패턴)
│   │
│   └── config/
│       └── config.yaml                    #    기본 설정 (mrceki 형식)
│
└── test/                                  # 테스트 (135 tests)
    ├── conftest.py                        #    공통 fixture
    └── unit/
        ├── domain/                        #    도메인 테스트
        ├── usecase/                       #    RobotAdapter 테스트
        └── infra/                         #    RobotAPI, graph_utils, serializer 테스트
```

## 아키텍처

### Clean Architecture 의존성 규칙

```
presentation ──→ infra ──→ usecase ──→ domain
```

| 레이어 | 역할 | 의존성 규칙 |
|--------|------|------------|
| **domain** | 엔티티, 값 객체, 예외 | 순수 Python만 사용. 외부 라이브러리 import 금지 |
| **usecase** | RobotAdapter, 포트(ABC) 정의 | domain만 import. rclpy, paho 금지 |
| **infra** | 포트 구현체 (MQTT, nav_graph, config) | usecase/ports의 ABC를 구현 |
| **presentation** | 진입점, DI 조립 | 모든 구현체를 생성하고 usecase에 주입 |

### Port-Adapter 매핑

| Port (ABC) | Adapter (구현체) | 외부 의존성 |
|------------|-----------------|------------|
| `RobotAPI` | `Vda5050RobotAPI` | paho-mqtt |
| `ConfigPort` | `YamlConfigLoader` | pyyaml |

### 데이터 흐름

```
  ┌─────────┐     MQTT      ┌──────────────────────────────────┐    rmf_easy    ┌─────────┐
  │         │  ◄── state ──  │          Fleet Adapter           │  ── position → │         │
  │         │  ◄── conn ───  │                                  │  ── battery →  │         │
  │   AGV   │               │  ┌────────────────────────────┐  │               │  Open-  │
  │ (VDA5050)│  ── order ──→  │  │  RobotAdapter   (usecase)  │  │  ◄─ navigate │   RMF   │
  │         │  ── action ─→  │  │  Vda5050RobotAPI (infra)    │  │  ◄─ stop     │         │
  │         │               │  │  graph_utils     (infra)    │  │  ◄─ action   │         │
  └─────────┘               │  └────────────────────────────┘  │               └─────────┘
                             └──────────────────────────────────┘
```

### 핵심 컴포넌트

- **RobotAdapter** (`usecase/robot_adapter.py`): RMF의 `RobotCallbacks`(navigate, stop, execute_action)을 받아 VDA5050 AGV에 명령을 전달하는 브릿지. 명령 재시도 로직 포함.
- **Vda5050RobotAPI** (`infra/mqtt/vda5050_robot_api.py`): MQTT로 VDA5050 Order/InstantActions를 발행하고, AGV State를 구독하여 캐시.
- **graph_utils** (`infra/nav_graph/graph_utils.py`): RMF nav graph 파싱, networkx 최단 경로 탐색, nudged 좌표 변환, VDA5050 Node/Edge 생성.
- **main.py** (`presentation/main.py`): rmf_demos_fleet_adapter + mrceki 패턴의 진입점. FleetConfiguration 로드, Adapter 생성, 로봇별 RobotAdapter 생성, 주기적 상태 업데이트 루프 실행.

## 설치 및 빌드

### 의존성

```bash
# ROS 2 의존성
sudo apt install ros-jazzy-rmf-fleet-adapter-python

# Python 의존성
pip install paho-mqtt pyyaml networkx nudged
```

### 빌드

```bash
cd ~/rmf_ws
colcon build --packages-select vda5050_fleet_adapter
source install/setup.bash
```

## 실행

```bash
ros2 run vda5050_fleet_adapter fleet_adapter \
  -c /path/to/config.yaml \
  -n /path/to/nav_graph.yaml
```

### 시뮬레이션 시간 사용

```bash
ros2 run vda5050_fleet_adapter fleet_adapter \
  -c /path/to/config.yaml \
  -n /path/to/nav_graph.yaml \
  -sim
```

### 설정 파일

설정은 `config.yaml`에 정의한다. Open-RMF의 `FleetConfiguration.from_config_files()`가 읽을 수 있는 mrceki/vda5050_fleet_adapter 형식을 사용한다.

```yaml
# Fleet 설정 (Open-RMF FleetConfiguration 호환)
rmf_fleet:
  name: "vda5050_fleet"
  limits:
    linear: [0.5, 0.75]       # velocity, acceleration
    angular: [0.6, 2.0]
  profile:
    footprint: 0.3
    vicinity: 0.5
  reversible: True
  battery_system:
    voltage: 12.0
    capacity: 24.0
    charging_current: 5.0
  robots:
    AGV-001:
      charger: "charger_1"
  robot_state_update_frequency: 10.0

# MQTT / VDA5050 설정
fleet_manager:
  ip: "127.0.0.1"
  prefix: "uagv/v2/manufacturer"   # MQTT 토픽 prefix
  port: 1883

# 좌표 변환 (선택사항)
# reference_coordinates:
#   L1:
#     rmf: [[0, 0], [1, 0], [0, 1], [1, 1]]
#     robot: [[0, 0], [1, 0], [0, 1], [1, 1]]
```

## 테스트

```bash
cd ~/rmf_ws/src/vda5050_fleet_adapter

# 전체 테스트 (135 tests)
python3 -m pytest test/ -v

# 단위 테스트만 (132 tests)
python3 -m pytest test/unit/ -v

# colcon 테스트
cd ~/rmf_ws
colcon build --packages-select vda5050_fleet_adapter
colcon test --packages-select vda5050_fleet_adapter
colcon test-result --verbose
```

## 참조

| 문서/레포 | 설명 |
|-----------|------|
| [rmf_demos_fleet_adapter](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_fleet_adapter) | RMF 공식 fleet adapter 예제 (rmf_easy 패턴) |
| [mrceki/vda5050_fleet_adapter](https://github.com/mrceki/vda5050_fleet_adapter) | VDA5050 MQTT fleet adapter 참조 구현 |
| [CODING_RULES.md](CODING_RULES.md) | 코딩 규칙, 네이밍 컨벤션, 아키텍처 상세 |
| [VDA5050_PROTOCOL.md](VDA5050_PROTOCOL.md) | VDA5050 v2.0 통신 프로토콜 전체 정의 |
| [VDA5050 공식 스펙](https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md) | VDA5050 원본 스펙 문서 |
