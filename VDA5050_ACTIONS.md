# VDA5050 Actions Definition

## Overview

VDA5050 fleet adapter에서 지원하는 action 목록과 정의를 관리하는 문서.
Action은 3가지 방식으로 AGV에 전달된다:

| 전달 방식 | 설명 | 사용 시점 |
|-----------|------|----------|
| **nodeAction** | Order의 Node에 첨부 | 특정 위치 도착 시 실행 |
| **edgeAction** | Order의 Edge에 첨부 | 이동 중 실행 |
| **instantAction** | Order와 독립적으로 전송 | 즉시 실행 (활성 order 없을 때) |

---

## Action 공통 구조

```json
{
  "actionType": "string",
  "actionId": "string (UUID)",
  "blockingType": "NONE | SOFT | HARD",
  "actionDescription": "string (optional)",
  "actionParameters": [
    { "key": "string", "value": "string | number | boolean | array" }
  ]
}
```

### BlockingType 정의

| Type | 설명 |
|------|------|
| `NONE` | 주행과 병렬 실행 가능 |
| `SOFT` | 다른 action과 병렬 가능, 주행은 차단 |
| `HARD` | 모든 것을 차단 (독점 실행) |

---

## 현재 구현된 Action

### System Actions (내부 사용)

| actionType | blockingType | 전달 방식 | 설명 | 비고 |
|------------|-------------|----------|------|------|
| `cancelOrder` | HARD | instantAction | 현재 order 취소 | negotiation 시 사용 |
| `startPause` | HARD | instantAction | AGV 일시 정지 | negotiation 시 사용 |
| `stopPause` | HARD | instantAction | 일시 정지 해제 | - |

### RMF Task Actions

| actionType | blockingType | 전달 방식 | 설명 | actionParameters |
|------------|-------------|----------|------|-----------------|
| `teleop` | HARD | instantAction | 원격 조종 | - |
| `pick` | HARD | nodeAction / instantAction | 화물 적재 | `loadType`, `loadID` |
| `drop` | HARD | nodeAction / instantAction | 화물 하역 | `stationName` |

#### `pick` Action 상세

카트/화물 적재 action. RMF Task 생성 시 매개변수로 전달받는다.

```json
{
  "actionType": "pick",
  "actionId": "pick-patrol.dispatch-0",
  "blockingType": "HARD",
  "actionParameters": [
    { "key": "loadType", "value": "Tool" },
    { "key": "loadID", "value": "SP4ECTR002" }
  ]
}
```

| Parameter | Type | 설명 | 예시 | 소스 |
|-----------|------|------|------|------|
| `loadType` | string | 적재물 타입 종류 | `"Tool"`, `"Pallet"`, `"Box"` | Task 매개변수 |
| `loadID` | string | 카트(적재물) 고유 ID | `"SP4ECTR002"` | Task 매개변수 |

#### `drop` Action 상세

카트/화물 하역 action. RMF Task 생성 시 매개변수로 전달받는다.

```json
{
  "actionType": "drop",
  "actionId": "drop-patrol.dispatch-0",
  "blockingType": "HARD",
  "actionParameters": [
    { "key": "stationName", "value": "1004" }
  ]
}
```

| Parameter | Type | 설명 | 예시 | 소스 |
|-----------|------|------|------|------|
| `stationName` | string | 하역할 스테이션 이름 | `"1004"` | Task 매개변수 |

#### Cart Delivery (pick/drop) 시퀀스

Cart delivery는 pickup → dropoff 2개의 phase로 구성된다.
**전체 과정이 1개의 orderID**로 처리되며, 각 phase마다 `order_update_id`가 증가한다.

```
RMF Task: cart_delivery (wp1 → wp2[pickup] → wp3[dropoff])

Phase 1: navigate(dest=wp2, final=wp2)
  → 새 orderID 생성, order_update_id=0
  → VDA5050 Order 전송
  ┌──────────────────────────────────────────────┐
  │ Order (orderID=X, updateId=0)                │
  │   wp1(base) → wp2(base)                      │
  └──────────────────────────────────────────────┘
  → 거리 기반 도착 감지 → execution.finished()

Phase 2: execute_action('pick', {loadType, loadID})
  → active order 존재 → Order Update로 전송 (instantAction 아님)
  → order_update_id++ (=1)
  → 현재 노드(wp2)에 pick nodeAction(HARD) 부착
  ┌──────────────────────────────────────────────┐
  │ Order Update (orderID=X, updateId=1)         │
  │   wp2(base, actions=[pick{HARD}])            │
  │   ↳ track_action_id로 완료 추적              │
  └──────────────────────────────────────────────┘
  → AGV의 actionStates에서 action_id FINISHED 감지 → execution.finished()

Phase 3: navigate(dest=wp3, final=wp3)
  → 같은 orderID, order_update_id++ (=2)
  ┌──────────────────────────────────────────────┐
  │ Order Update (orderID=X, updateId=2)         │
  │   wp2(base) → wp3(base)                      │
  │   ↳ sequenceId는 이전 stitching point에서 연속 │
  └──────────────────────────────────────────────┘
  → 거리 기반 도착 감지 → execution.finished()

Phase 4: execute_action('drop', {stationName})
  → Order Update (nodeAction)
  → order_update_id++ (=3)
  ┌──────────────────────────────────────────────┐
  │ Order Update (orderID=X, updateId=3)         │
  │   wp3(base, actions=[drop{HARD}])            │
  │   ↳ track_action_id로 완료 추적              │
  └──────────────────────────────────────────────┘
  → AGV의 actionStates에서 action_id FINISHED 감지 → execution.finished()

Task 완료 → _reset_order_state()
```

핵심 구현 사항:
- `execute_action()` 호출 시 `_active_order_id`가 존재하면 `_execute_action_as_order_update()`로 처리
- 현재 위치의 nearest node에 action을 `BlockingType.HARD`로 첨부
- `track_action_id`로 action 완료를 모니터링 (order 완료가 아닌 action 단위)
- sequenceId는 `_last_stitch_seq_id`로 order update 간 연속성 유지

### Charging Actions

| actionType | blockingType | 전달 방식 | 설명 | actionParameters |
|------------|-------------|----------|------|-----------------|
| `startCharging` | HARD | nodeAction | 충전 시작 (charger 앞 노드에 부착) | `stationName` |
| `stopCharging` | HARD | instantAction | 충전 중지 (SOC 도달 시 자동 전송) | - |

#### `startCharging` Action 상세

RMF 자동 충전 시 charger 앞 노드에 nodeAction으로 부착된다.
Adapter가 `destination.dock`을 감지하여 자동 생성하므로 config 등록 불필요.

```json
{
  "actionType": "startCharging",
  "actionId": "startCharging_5_a1b2c3d4",
  "blockingType": "HARD",
  "actionParameters": [
    { "key": "stationName", "value": "charger_1" }
  ]
}
```

| Parameter | Type | 설명 | 예시 | 소스 |
|-----------|------|------|------|------|
| `stationName` | string | 충전 스테이션 이름 (charger 노드 이름) | `"charger_1"` | `destination.dock` |

#### `stopCharging` Action 상세

SOC가 `recharge_soc` 이상에 도달하면 instantAction으로 자동 전송된다.
파라미터 없음.

```json
{
  "actionType": "stopCharging",
  "actionId": "stopCharging_7",
  "blockingType": "HARD",
  "actionParameters": []
}
```

#### Charging 시퀀스

RMF가 `battery_soc <= recharge_threshold`를 감지하면 자동으로 충전 task를 생성한다.
충전 task에서 RMF는 `navigate(destination)` 호출 시 `destination.dock = "charger_name"`을 설정한다.

```
전제: config에 finishing_request: "park" 설정
      battery_soc <= recharge_threshold → RMF가 자동 충전 task 생성

경로 예시: wp1 → wp2 → wp3 → charger_1

Phase 1: navigate(dest=charger_1, dock="charger_1")
  → dock 감지 → _is_charging_pending = True
  → 경로 계산: [wp1, wp2, wp3, charger_1]
  → charger 노드 제거: [wp1, wp2, wp3]
  → wp3(pre-charger)에 startCharging nodeAction 부착
  → _navigate_target_position = wp3 좌표 (charger가 아닌 pre-charger)
  ┌──────────────────────────────────────────────────────────────┐
  │ Order                                                        │
  │   wp1(base) → wp2(base) → wp3(base,                         │
  │     actions=[startCharging{HARD, stationName:"charger_1"}])  │
  │   ↳ charger_1 노드는 order에 미포함                          │
  └──────────────────────────────────────────────────────────────┘

Phase 2: AGV 이동 → pre-charger(wp3) 도착
  → update()에서 거리 기반 도착 감지 (wp3까지)
  → _is_charging_pending → _is_charging 전환
  → execution.finished() 호출하지 않음 (충전 계속)
  → AGV가 wp3 도착 후 startCharging nodeAction 자동 실행
  → AGV가 charger_1로 자율 도킹하여 충전 시작

Phase 3: SOC 모니터링 (update() 루프)
  → _is_charging == True 상태에서 battery_soc 감시
  → battery_soc < recharge_soc → 대기 (finished 미호출)
  → battery_soc >= recharge_soc:
    ┌─────────────────────────────────────────────┐
    │ InstantAction: stopCharging                 │
    │   → api.start_activity('stopCharging', {})  │
    └─────────────────────────────────────────────┘
    → execution.finished() 호출
    → 충전 상태 초기화

Task 완료 → _reset_order_state()
```

핵심 구현 사항:
- `destination.dock`이 문자열이면 충전 task로 판단
- 경로에서 charger 노드(마지막 노드)를 제거하고, 바로 앞 노드에 `startCharging` 부착
- `_navigate_target_position`을 pre-charger 좌표로 변경하여 도착 판정 기준 조정
- pre-charger 도착 시 `_is_charging = True`로 전환, `execution.finished()` 미호출
- `update()` 루프에서 `data.battery_soc >= self.recharge_soc` 조건 충족 시 `stopCharging` 전송
- negotiation 발생 시 (`stop()` 호출) 충전 중이면 `stopCharging` 먼저 전송 후 `startPause`
- `recharge_soc`는 config YAML의 `rmf_fleet.recharge_soc`에서 로드 (기본값 1.0)

예외 처리:
- 충전 중 negotiation → `stopCharging` → `startPause` → `cancelOrder` → 새 경로
- `_reset_order_state()` 호출 시 모든 충전 상태 초기화

---

## 추가 예정 Action

### Navigation / Positioning

| actionType | blockingType | 전달 방식 | 설명 | actionParameters | 구현 상태 |
|------------|-------------|----------|------|-----------------|----------|
| `initPosition` | HARD | instantAction | AGV 위치 초기화 | `x`, `y`, `theta`, `mapId`, `lastNodeId` | [ ] |
| `finePositioning` | HARD | nodeAction | 정밀 위치 조정 | `stationType`, `stationName` | [ ] |

### Cargo Handling

> `pick`/`drop`은 RMF Task Actions로 이동됨 (위 섹션 참조)

### Map Management

| actionType | blockingType | 전달 방식 | 설명 | actionParameters | 구현 상태 |
|------------|-------------|----------|------|-----------------|----------|
| `downloadMap` | NONE | instantAction | 맵 다운로드 | `mapId`, `mapDownloadUrl` | [ ] |
| `enableMap` | NONE | instantAction | 맵 활성화 | `mapId` | [ ] |
| `deleteMap` | NONE | instantAction | 맵 삭제 | `mapId` | [ ] |

### Trigger / Wait

| actionType | blockingType | 전달 방식 | 설명 | actionParameters | 구현 상태 |
|------------|-------------|----------|------|-----------------|----------|
| `waitForTrigger` | HARD | nodeAction | 외부 트리거 대기 | `triggerType`, `timeout` | [ ] |

### Information

| actionType | blockingType | 전달 방식 | 설명 | actionParameters | 구현 상태 |
|------------|-------------|----------|------|-----------------|----------|
| `factsheetRequest` | NONE | instantAction | AGV factsheet 요청 | - | [ ] |

### Custom Actions (프로젝트 특화)

| actionType | blockingType | 전달 방식 | 설명 | actionParameters | 구현 상태 |
|------------|-------------|----------|------|-----------------|----------|
| | | | | | [ ] |

---

## Action 구현 체크리스트

새로운 action 추가 시 아래 항목을 모두 완료해야 한다:

- [ ] `domain/entities/action.py` - 필요시 action 관련 상수/헬퍼 추가
- [ ] `config/config.yaml` - `actions` 목록에 actionType 등록
- [ ] `usecase/robot_adapter.py` - execute_action 콜백에서 처리 로직 구현
- [ ] `infra/mqtt/vda5050_robot_api.py` - 전송 로직 (nodeAction / instantAction)
- [ ] `infra/mqtt/message_serializer.py` - 직렬화/역직렬화 확인
- [ ] `test/` - 단위 테스트 작성
- [ ] 이 문서(`VDA5050_ACTIONS.md`) 업데이트

---

## Config 등록 방법

`config.yaml`의 `actions` 목록에 actionType을 추가:

```yaml
rmf_fleet:
  actions: ["teleop", "pick", "drop", "NEW_ACTION_TYPE"]
```

---

## Action 흐름도

```
RMF Task
  │
  ├─ navigate() ──────────────────────────► Order (nodeAction/edgeAction)
  │                                              │
  ├─ execute_action() ─┬─ active order 있음 ──► Order Update (nodeAction)
  │                    └─ active order 없음 ──► InstantAction
  │
  └─ stop() ──────────────────────────────► InstantAction (startPause)

                         ┌──────────────────┐
                         │   AGV (VDA5050)   │
                         └────────┬─────────┘
                                  │
                         State (actionStates)
                                  │
                    ┌─────────────▼──────────────┐
                    │ WAITING → INITIALIZING →    │
                    │ RUNNING → FINISHED/FAILED   │
                    └─────────────────────────────┘
```
