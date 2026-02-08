# CLAUDE.md - VDA5050 Fleet Adapter Project Rules

## Project Overview
VDA5050 Fleet Adapter: ROS 2 (Jazzy) + Open-RMF fleet adapter bridging VDA5050 AGVs via MQTT.

## Reference Implementation
- Fleet adapter 구현 시 [rmf_demos_fleet_adapter](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_fleet_adapter)를 참조할 것
- Fleet adapter 구현 시 [vda5050_fleet_adapter](https://github.com/mrceki/vda5050_fleet_adapter/tree/main/vda5050_fleet_adapter)를 참조할 것 

## Tech Stack
- Python 3.12+ / ROS 2 Jazzy Jalisco (Ubuntu 24.04)
- `rmf_fleet_adapter_python`, `rclpy`, MQTT (paho-mqtt)
- Build: `ament_python` / `colcon`

## Architecture: Clean Architecture
```
vda5050_fleet_adapter/
  domain/         # Entities, Value Objects, Domain Events (no external deps)
  usecase/        # Application logic, Port interfaces (ABC)
  infra/          # Adapters: MQTT client, ROS 2 nodes, config loaders
  presentation/   # Entry points, CLI, ROS 2 launch integration
```
- Dependencies flow inward only: infra -> usecase -> domain
- Domain layer MUST NOT import rclpy, paho-mqtt, or any framework
- Use Port/Adapter (ABC interfaces in usecase/, implementations in infra/)

## Coding Rules

### Style
- PEP 8 strict, enforced by flake8
- Line length: 99 characters max
- Docstrings: Google style, PEP 257 compliant
- Type hints required on all public functions/methods
- snake_case for functions/variables, PascalCase for classes, UPPER_SNAKE for constants

### Naming Conventions
- ROS 2 nodes: `vda5050_` prefix (e.g., `vda5050_fleet_adapter_node`)
- MQTT topics: follow VDA5050 spec (`/interfaceName/version/manufacturer/serialNumber/topic`)
- Config keys: snake_case in YAML

### Error Handling
- Never use bare `except:`; always catch specific exceptions
- ROS 2 logging (`self.get_logger()`) for node-level logging
- Python `logging` module for non-ROS components
- Log levels: DEBUG for protocol details, INFO for state changes, WARN for recoverable errors, ERROR for failures

### Testing
- pytest for unit tests, launch_testing for integration
- Unit tests MUST NOT depend on ROS 2 runtime or MQTT broker
- Mock external dependencies at Port boundaries
- Test file naming: `test_<module_name>.py`

### VDA5050 Specific
- All VDA5050 message models defined as dataclasses in domain/
- Strict validation of VDA5050 JSON schemas on send/receive
- AGV state machine transitions must be explicit and logged

## Build & Test Commands
```bash
# Build
cd ~/rmf_ws && colcon build --packages-select vda5050_fleet_adapter

# Test
colcon test --packages-select vda5050_fleet_adapter
colcon test-result --verbose

# Lint
cd ~/rmf_ws/src/vda5050_fleet_adapter && python -m flake8
python -m pytest test/
```

## Workflow
- 작업이 완료되면 자동으로 git commit을 수행할 것 (사용자가 별도로 요청하지 않아도)

## Key Constraints
- Do NOT add ROS 2 message type definitions here; use existing rmf_fleet_msgs
- MQTT QoS: use QoS 1 for state/visualization, QoS 2 for orders/instant actions
- All config values must be loadable from YAML parameters, no hardcoded values
- Thread safety: use `threading.Lock` for shared state between MQTT callbacks and ROS 2 callbacks

## VDA5050 order 생성시 지켜야할 규칙
- order의 단위 : rmf의 한개의 Task가 하나의 order단위다 되어야한다. 예를 들어 rmf에 go_to_place명령이 들어올경우 로봇으로 전달 되는 orderID는 하나여야 한다.
- order의 단위 예외사항 : rmf core의 기능을 통해 로봇이 다른 로봇 및 특정상황에서 기존 경로가 변경이 일어날경우 기존 order를 cancel 시키고 다른 orderid로 새로 내린다. 
- order의 생성 범위 : order에 rmf core에서 내려온 Destination까지를 Base로 만들고 나머지 최종 목적지까지의 경로를 horizon으로 만든다. 
- order update 시 : rmf core에서 새로운 Destination이 내려오면 해당 Destination까지 Base로 만들고 나머지 최종 목적지까지의 경로를 horizon으로 만든다. 
- 최종 목적지 란 : 최종목적지는, rmf core에 명령 넣은 로봇의 최종 도착지 이다. 예를 들어 ros2 run rmf_demos_tasks dispatch_go_to_place -p pantry 이렇게 터미널에서 명령을 주면 "pantry"가 최종 목적지가 된다. 
- negotiation 발생 시 규칙 : rmf에서 negotiation이 발생하면 아래 순서를 따른다
  1. 발생 로봇에 startPause instantAction을 내린다.
  2. 이후 새로운 Destination이 내려오면 로봇에 cancelOrder instantAction을 내린다.
  3. 이후 위 order 생성 규칙을 통해 order를 생성하여 로봇을 움직인다. 단 이경우 새로운 orderID로 명령을 내린다.