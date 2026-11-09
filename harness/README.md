# Codex Harness

Python Codex runtime → MCP `observe` → ZMQ 서버 → observation → 제어신호 JSON 출력.
기본 모드는 서버의 현재 관측을 사용합니다. 한 번 실행하며, 실패 기록·목표 평가·로봇 명령 전송은 포함하지 않습니다.

```bash
# 터미널 1: 통신 서버
uv run --project server server

# 터미널 2: observation을 보고 제어신호 출력
uv run --project harness harness-run \
  --robot-id kiwi-01 '현재 센서를 확인하고 100ms 대기하는 제어신호를 출력해줘.'
```

저장된 observation으로 시험 (서버/ORIN 불필요):

```bash
uv run --project harness harness-run \
  --snapshot TEMP/zmq_server/captures/20261109-125911/message-0001/frame-00.json \
  '관측한 상태를 확인하고 100ms 대기 신호를 출력해줘.'
```

`--snapshot`은 파일을 MCP `observe`로 전달합니다. 결과에는 기록된 데이터임을
명시하며, 원래 관측 timestamp와 센서값은 유지합니다.

- Codex 로그인 필요. 공식 `openai-codex` Python SDK와 번들 runtime 사용.
- `--server tcp://127.0.0.1:5556`, `--model`로 연결 주소와 모델 지정 가능.
- `runtime.py`: Codex 실행, MCP 연결, JSON 출력. 모델 미지정 시 Codex 기본 설정 사용.
- `tools.py`: `observe(robot_id)`와 `execute(robot_id, x, y, theta, duration_ms)` 노출. JPEG는 MCP 이미지로 반환. snapshot 모드에는 observe만 등록합니다.
- `client.py`: 독립된 ZMQ 서버에 요청. 다른 프로젝트의 Python 코드를 import하지 않음.
- MCP 관측 성공을 확인하지 못하면 제어신호 출력을 거절함.

ORIN은 `move`, `wait`을 지원합니다. `harness-run`은 기존대로 observe만 허용하는 제어신호 출력 모드이며, 현재 출력 스키마는 wait 전용입니다. 실제 주행은 아래 execute MCP를 직접 호출합니다.

```json
{"robot_id":"kiwi-01","command":{"action":"wait","duration_ms":100},"reason":"관측에 기반한 설명"}
```

MCP만 실행: `uv run --project harness harness --server tcp://127.0.0.1:5556`

테스트: `uv run --project harness python -m unittest discover -s harness/tests -v`

SDK 문서: https://learn.chatgpt.com/docs/codex-sdk

## 차체 속도 기반 execute MCP

```json
{"robot_id":"kiwi-01","x":0.2,"y":0.0,"theta":0.0,"duration_ms":500}
```

위 호출은 전진 0.2 m/s를 500 ms 동안 명령합니다. `x`는 전진(+)/후진(-), `y`는 왼쪽(+)/오른쪽(-) 평행이동(m/s), `theta`는 반시계(+)/시계(-) 회전(deg/s)입니다. 평행이동과 회전을 함께 지정할 수 있습니다. 평행이동 속도 `hypot(x,y)`는 최대 0.4 m/s, 회전은 ±150 deg/s, 시간은 1–1000 ms입니다. 모두 0인 속도도 허용합니다.

MCP는 서버에 `command: {action: "move", x, y, theta, duration_ms}`를 보내며, ORIN이 바퀴 ID 7·8·9의 속도로 변환합니다. 실행 종료 시 정지·토크 해제를 수행한 뒤 observation을 반환합니다. 완료는 명령 전송과 정지 처리의 성공을 의미하며, 실제 이동 거리 도달을 검증하는 방식은 아닙니다. 통신 오류나 시간 초과는 실행 결과가 불확실할 수 있어 자동 재시도하지 않습니다.

## 폴더에서 직접 실행

observe/execute MCP 서버(stdio)를 시작합니다. Codex 제어신호 출력은 기존 `harness-run` 명령을 사용합니다.

```bash
cd harness
uv run python main.py
```

`uv run`이 의존성과 프로젝트 설치를 준비합니다. 이미 설치된 Python 환경에서는 `python main.py`로 실행할 수 있습니다. 기존 CLI 옵션도 그대로 사용할 수 있습니다.

## 팔·그리퍼 제어

execute MCP 예시(절대 목표값):

```json
{"robot_id":"kiwi-01","action":"move","joints":{"wrist_roll":10,"gripper":60},"duration_ms":1000}
```

관절명은 shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper입니다. 팔은 도(deg), 그리퍼는 보정된 개폐율(%)입니다. 먼저 observe의 `ego.joints`와 `ego.limits`를 확인합니다. 생략한 관절은 건드리지 않습니다. 이동 중 속도는 관절 30deg/s, 그리퍼 50%/s 이하, 실행 시간은 100–1000ms입니다. 베이스와 팔을 한 요청에서 동시에 제어하지 않습니다.

팔은 목표 도달 피드백을 확인한 후 완료를 반환합니다. `ego.last_motion`에 목표·측정값·허용오차가 포함됩니다. 팔 자세 유지 토크는 자동으로 해제하지 않습니다. 관절 도달과 물체 잡기 성공은 서로 다르며, Cartesian IK·자동 파지 경로 계획은 포함하지 않습니다.

베이스와 팔을 동시에 움직이려면 execute에 `action="move"`, `x/y/theta`, `joints`, 공통 `duration_ms`(100–1000)를 전달합니다. 각 축의 기존 범위·속도 검증을 적용하며, ORIN runtime의 동시 제어 지원 버전이 필요합니다. 정상 실행 후 팔은 자세를 유지하지만 런타임 종료 시에는 팔·바퀴 토크를 해제합니다.

### Action 규칙

MCP 도구는 `observe`, `execute` 두 개이며, execute의 action은 `move`, `wait` 두 개입니다.
`move`는 베이스 속도(x/y/theta, 생략 시 0)와 팔 관절 목표(joints, 생략 시 팔 유지)를 선택적으로 받습니다. 둘 다 지정하면 동시에 움직입니다. `wait`에는 이동 목표를 넣지 않습니다.

`observe`와 `execute` 응답에는 `observation.imu`도 포함됩니다. 가속도·각속도·자세 및 초기 위치 기준 추정 변위(m)를 반환합니다. `available`, `estimate_valid`, `age_s`, `origin_id`를 확인하고, 동일 원점의 유효한 추정값끼리만 비교합니다. IMU 적분 변위는 누적 오차가 있는 추정치입니다. 도구와 action은 추가되지 않습니다.
