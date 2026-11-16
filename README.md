# Lekiwi

여러 Lekiwi 로봇의 observation을 ZeroMQ로 수신하고, Codex가 관측을 바탕으로 제어신호를 생성하는 프로젝트입니다.

기본 harness는 observation 조회와 `wait` 제안 JSON 출력까지만 수행합니다. `harness-run --execute`는 관찰 → 한 동작 실행 → 새 관찰 → 재판단을 반복하는 별도 실행 모드입니다. 런타임은 베이스 속도·팔 관절 목표를 받는 `move`와 `wait`을 지원합니다. [에이전트 루프와 복구 안내](docs/agent-loop.md)를 참고하세요.

화면에서 지시와 진행 상황을 보려면 서버·runtime 실행 후 `uv run --project harness harness-gui`를 실행하고 http://127.0.0.1:8765 를 엽니다. 터미널 실행과 같은 루프의 계획·판단 근거·카메라 전후 이미지·실행 결과를 표시합니다. 로봇·모델 없이 화면만 확인하려면 `uv run --project harness harness-gui --demo --port 8766`을 사용합니다. [GUI 실행 안내](harness/README.md#gui에서-실행)

## 구성

| 폴더 | 역할 |
| --- | --- |
| [server](server/README.md) | 로봇별 observe/execute 요청과 응답을 중계하는 ZeroMQ 서버 |
| [harness](harness/README.md) | Codex 판단, MCP observe/execute, 제안 모드와 관찰 기반 실행 루프 |
| [lekiwi_runtime](lekiwi_runtime/README.md) | ORIN에서 실행하는 DEALER 클라이언트와 센서 드라이버 |
| `contracts/` | 예약된 빈 폴더 |
| `utility/` | 예약된 빈 폴더 |
| `TEMP/` | 로컬 참고 코드와 observation 캡처. Git 관리 제외 |

각 패키지는 독립적으로 설치하며 서로의 Python 코드를 import하지 않습니다. 빈 폴더는 Git에 저장되지 않습니다.

```text
Codex Python runtime
        │ MCP observe / execute
        ▼
     harness
        │ REQ → tcp://127.0.0.1:5556
        ▼
      server
        │ ROUTER ← DEALER (tcp://SERVER_IP:5555)
        ▼
  lekiwi_runtime (ORIN)
        │
        └─ 카메라 / 레이더 / 관절 observation
```

## 준비

- 서버·harness: Python 3.13 이상, uv
- ORIN runtime: Python 3.10 이상; 센서 드라이버와 설정은 runtime에 포함
- Codex runtime 실행: Codex 로그인

아래 명령은 저장소 루트에서 실행합니다.

```bash
uv sync --project server
uv sync --project harness
```

## 실행

### 1. 서버 PC

```bash
uv run --project server server
```

로봇 연결은 5555 포트, harness 호출은 로컬 5556 포트를 사용합니다.

### 2. ORIN

runtime과 드라이버 의존성을 설치하고 실행합니다.

```bash
uv sync --project lekiwi_runtime
uv run --project lekiwi_runtime lekiwi-runtime \
  --server tcp://SERVER_IP:5555 \
  --robot-id kiwi-01
```

`SERVER_IP`는 서버 PC 주소입니다. 각 로봇은 서로 다른 `--robot-id`를 사용합니다.
외부 ORIN 폴더 없이 실행됩니다. 장비별 설정은 `--config /etc/lekiwi/robot.json`으로 지정하고, 하드웨어 없이 확인하려면 `--mock`을 사용합니다. 기본 레이더 프로필은 TDM 펌웨어용이며, 보정값은 제공된 follower 로봇 기준입니다. 상세 설정은 [runtime 문서](lekiwi_runtime/README.md)를 참고하세요.

### 3. 관측 기반 제어신호 출력

서버 PC의 다른 터미널에서 실행합니다.

```bash
uv run --project harness harness-run \
  --robot-id kiwi-01 \
  '현재 센서를 확인하고 100ms 대기하는 제어신호를 출력해줘.'
```

출력 형식:

```json
{
  "robot_id": "kiwi-01",
  "command": {"action": "wait", "duration_ms": 100},
  "reason": "관측에 기반한 설명"
}
```

## 저장된 observation으로 실행

서버와 로봇 없이 JSON 캡처를 MCP observation으로 전달할 수 있습니다.

```bash
uv run --project harness harness-run \
  --snapshot /path/to/frame-00.json \
  --robot-id kiwi-01 \
  '관측한 상태를 확인하고 100ms 대기 신호를 출력해줘.'
```

파일은 `{"status":"ok","observation":{...}}` 응답 형식이어야 합니다. 같은 폴더에 `schema.json`이 있으면 기록된 로봇 identity도 검사합니다. 결과에는 기록된 데이터임을 명시합니다. `TEMP/`의 캡처는 Git에서 제외되므로 새 checkout에는 별도로 준비해야 합니다.

## 테스트

```bash
uv run --project harness python -m unittest discover -s harness/tests -v
uv run --project lekiwi_runtime python -m unittest discover -s lekiwi_runtime/tests -v
```

모의 로봇 라우팅, MCP 관측, snapshot 처리와 runtime 응답을 검증합니다. 실물 로봇이나 Codex 모델 호출 없이 실행됩니다.

## 폴더별 main.py 실행

각 폴더 안에서 `uv run python main.py`를 실행합니다. 이미 해당 패키지를 설치한 Python 환경에서는 `python main.py`도 가능합니다.

| 폴더 | 실행되는 프로그램 | 기본 연결 |
| --- | --- | --- |
| `server` | ZeroMQ 중계 서버 | 로봇 5555, API 5556 |
| `harness` | observe/execute MCP 서버(stdio) | `tcp://127.0.0.1:5556` |
| `lekiwi_runtime` | 로봇 센서·주행 클라이언트 | `tcp://127.0.0.1:5555` |

서로 다른 컴퓨터에서 실행할 때는 harness/runtime에 `--server tcp://SERVER_IP:PORT`를 지정합니다. MCP 서버는 터미널 대화형 UI가 아니라 MCP 클라이언트와 stdio로 연결하는 프로세스입니다.
