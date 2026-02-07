# VDA5050 Fleet Adapter

VDA5050 표준을 따르는 AGV와 Open-RMF를 연결하는 Fleet Adapter.
AGV와는 MQTT로, RMF와는 ROS 2로 이중 통신한다.

| 항목 | 내용 |
|------|------|
| Python | 3.12+ |
| ROS 2 | Jazzy Jalisco (Ubuntu 24.04) |
| 아키텍처 | Clean Architecture (4 레이어) |
| VDA5050 | v2.0 |

## 디렉토리 구조

```
vda5050_fleet_adapter/
│
├── CLAUDE.md                              # Claude Code 대전제 규칙
├── CODING_RULES.md                        # 코딩 규칙 & 아키텍처 가이드
├── VDA5050_PROTOCOL.md                    # VDA5050 프로토콜 정의서
├── package.xml                            # ROS 2 패키지 매니페스트
├── setup.py
│
├── vda5050_fleet_adapter/                 # 메인 패키지
│   │
│   ├── domain/                            # ① 도메인 (순수 Python, 외부 의존성 없음)
│   │   ├── enums.py                       #    StrEnum 9개 (OperatingMode, ActionStatus 등)
│   │   ├── exceptions.py                  #    도메인 예외 6개
│   │   ├── value_objects/                 #    불변 값 객체 (frozen dataclass)
│   │   │   ├── position.py                #      NodePosition, AgvPosition, Velocity
│   │   │   ├── physical.py                #      BoundingBoxReference, LoadDimensions, Corridor
│   │   │   └── trajectory.py              #      ControlPoint, Trajectory (NURBS)
│   │   ├── entities/                      #    도메인 엔티티
│   │   │   ├── header.py                  #      공통 메시지 헤더
│   │   │   ├── action.py                  #      Action, ActionState (상태 전이 머신)
│   │   │   ├── node.py                    #      Node, NodeState
│   │   │   ├── edge.py                    #      Edge, EdgeState
│   │   │   ├── order.py                   #      Order (validate, Base/Horizon 분리)
│   │   │   ├── agv_state.py               #      AgvState (전체 AGV 상태 통합)
│   │   │   ├── error.py                   #      AgvError, AgvInformation
│   │   │   ├── battery.py                 #      BatteryState
│   │   │   ├── safety.py                  #      SafetyState
│   │   │   ├── load.py                    #      Load
│   │   │   ├── map_state.py               #      AgvMap
│   │   │   └── connection.py              #      Connection
│   │   └── events/                        #    도메인 이벤트
│   │       └── agv_events.py              #      9개 이벤트 클래스
│   │
│   ├── usecase/                           # ② 유스케이스
│   │   ├── ports/                         #    포트 인터페이스 (ABC)
│   │   │   ├── agv_gateway.py             #      AGV ↔ MQTT 통신
│   │   │   ├── fleet_gateway.py           #      RMF Fleet 통신
│   │   │   ├── state_repository.py        #      상태 저장소
│   │   │   ├── event_publisher.py         #      이벤트 발행/구독
│   │   │   └── config_port.py             #      설정 로더 + Config 데이터 클래스
│   │   ├── process_order.py               #    RMF 명령 → VDA5050 Order 전송
│   │   ├── update_agv_state.py            #    AGV State 수신 → 저장 → RMF 보고
│   │   └── handle_action.py               #    즉시 액션 (pause/cancel/charge 등)
│   │
│   ├── infra/                             # ③ 인프라 (외부 의존성 격리)
│   │   ├── mqtt/                          #    AgvGateway 구현
│   │   │   ├── message_serializer.py      #      JSON ↔ 도메인 (camelCase 변환)
│   │   │   ├── mqtt_client.py             #      paho-mqtt 래퍼 (재연결, Last Will)
│   │   │   └── vda5050_mqtt_adapter.py    #      AgvGateway 구현체
│   │   ├── ros2/                          #    FleetGateway 구현
│   │   │   ├── fleet_adapter_handle.py    #      NavigationHandle/TaskHandle 구현체
│   │   │   └── ros2_fleet_gateway.py      #      FleetGateway 구현체
│   │   ├── config/
│   │   │   └── yaml_config_loader.py      #    ConfigPort 구현체
│   │   ├── repository/
│   │   │   └── in_memory_state_repository.py  # StateRepository 구현체
│   │   └── event/
│   │       └── in_memory_event_publisher.py   # EventPublisher 구현체
│   │
│   ├── presentation/                      # ④ 프레젠테이션 (진입점)
│   │   ├── fleet_adapter_node.py          #    ROS 2 노드 + DI 조립
│   │   └── main.py                        #    Entry point
│   │
│   └── config/
│       └── default_params.yaml            #    기본 설정
│
└── test/                                  # 테스트 (121 tests)
    ├── conftest.py                        #    공통 fixture
    └── unit/
        ├── domain/                        #    44 tests
        ├── usecase/                       #    22 tests (Mock 기반)
        └── infra/                         #    55 tests
```

## 아키텍처

### Clean Architecture 의존성 규칙

```
presentation ──→ infra ──→ usecase ──→ domain
```

| 레이어 | 역할 | 의존성 규칙 |
|--------|------|------------|
| **domain** | 엔티티, 값 객체, 이벤트, 예외 | 순수 Python만 사용. 외부 라이브러리 import 금지 |
| **usecase** | 애플리케이션 로직, 포트(ABC) 정의 | domain만 import. rclpy, paho 금지 |
| **infra** | 포트 구현체 (MQTT, ROS 2, 저장소) | usecase/ports의 ABC를 구현 |
| **presentation** | 진입점, DI 조립 | 모든 구현체를 생성하고 usecase에 주입 |

### Port-Adapter 매핑

| Port (ABC) | Adapter (구현체) | 외부 의존성 |
|------------|-----------------|------------|
| `AgvGateway` | `Vda5050MqttAdapter` | paho-mqtt |
| `FleetGateway` | `Ros2FleetGateway` | rclpy, rmf_fleet_adapter |
| `ConfigPort` | `YamlConfigLoader` | pyyaml |
| `StateRepository` | `InMemoryStateRepository` | - (threading.Lock) |
| `EventPublisher` | `InMemoryEventPublisher` | - |

### 데이터 흐름

```
  ┌─────────┐     MQTT      ┌──────────────────────────────────┐     ROS 2     ┌─────────┐
  │         │  ◄── state ──  │          Fleet Adapter           │  ── position →│         │
  │         │  ◄── conn ───  │                                  │  ── battery → │         │
  │   AGV   │               │  ┌────────────────────────────┐  │               │  Open-  │
  │         │  ── order ──→  │  │  UpdateAgvState (usecase)  │  │  ◄─ navigate │   RMF   │
  │         │  ── action ─→  │  │  ProcessOrder   (usecase)  │  │  ◄─ stop     │         │
  │         │               │  │  HandleAction   (usecase)  │  │  ◄─ action   │         │
  └─────────┘               │  └────────────────────────────┘  │               └─────────┘
                             └──────────────────────────────────┘
```

## 설치 및 빌드

### 의존성

```bash
# ROS 2 의존성
sudo apt install ros-jazzy-rmf-fleet-adapter-python

# Python 의존성
pip install paho-mqtt pyyaml
```

### 빌드

```bash
cd ~/rmf_ws
colcon build --packages-select vda5050_fleet_adapter
source install/setup.bash
```

## 실행

### 기본 실행

```bash
ros2 run vda5050_fleet_adapter fleet_adapter \
  --ros-args -p agv_ids:="['AGV-001','AGV-002']"
```

### 파라미터 오버라이드

```bash
ros2 run vda5050_fleet_adapter fleet_adapter \
  --ros-args \
  -p fleet_name:=my_fleet \
  -p mqtt.broker_host:=192.168.1.100 \
  -p mqtt.broker_port:=1883 \
  -p vda5050.manufacturer:=MyRobotCo \
  -p agv_ids:="['AGV-001','AGV-002']"
```

### 설정 파일

기본 설정은 `vda5050_fleet_adapter/config/default_params.yaml`에 정의되어 있다.
ROS 2 파라미터로 개별 값을 오버라이드할 수 있다.

```yaml
vda5050_fleet_adapter:
  ros__parameters:
    fleet_name: "vda5050_fleet"
    mqtt:
      broker_host: "localhost"
      broker_port: 1883
      keepalive_sec: 60
      reconnect_max_delay_sec: 60
    vda5050:
      interface_name: "uagv"
      protocol_version: "v2"
      manufacturer: "default_manufacturer"
    adapter:
      state_publish_rate_hz: 1.0
      order_timeout_sec: 30.0
      max_retry_count: 3
```

## 테스트

```bash
cd ~/rmf_ws/src/vda5050_fleet_adapter

# 전체 단위 테스트 (121 tests)
python3 -m pytest test/unit/ -v

# 레이어별 실행
python3 -m pytest test/unit/domain/ -v      # 44 tests
python3 -m pytest test/unit/usecase/ -v     # 22 tests
python3 -m pytest test/unit/infra/ -v       # 55 tests

# 커버리지
python3 -m pytest test/unit/ --cov=vda5050_fleet_adapter --cov-report=term-missing

# colcon 테스트
cd ~/rmf_ws
colcon test --packages-select vda5050_fleet_adapter
colcon test-result --verbose
```

## 관련 문서

| 문서 | 설명 |
|------|------|
| [CODING_RULES.md](CODING_RULES.md) | 코딩 규칙, 네이밍 컨벤션, 아키텍처 상세 |
| [VDA5050_PROTOCOL.md](VDA5050_PROTOCOL.md) | VDA5050 v2.0 통신 프로토콜 전체 정의 |
| [CLAUDE.md](CLAUDE.md) | Claude Code용 프로젝트 규칙 |
| [VDA5050 공식 스펙](https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md) | VDA5050 원본 스펙 문서 |
