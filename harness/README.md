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
- `tools.py`: `observe(robot_id)` 하나만 노출. JPEG는 MCP 이미지로 반환.
- `client.py`: 독립된 ZMQ 서버에 요청. 다른 프로젝트의 Python 코드를 import하지 않음.
- MCP 관측 성공을 확인하지 못하면 제어신호 출력을 거절함.

현재 ORIN이 지원하는 액션은 `wait`뿐입니다. 이동·관절 명령이 필요한 요청은
`wait, duration_ms=0`과 미지원 사유를 반환합니다. 모터 명령 형식이 확정되면
`runtime.py`의 출력 스키마와 지시문을 변경하면 됩니다.

```json
{"robot_id":"kiwi-01","command":{"action":"wait","duration_ms":100},"reason":"관측에 기반한 설명"}
```

MCP만 실행: `uv run --project harness harness --server tcp://127.0.0.1:5556`

테스트: `uv run --project harness python -m unittest discover -s harness/tests -v`

SDK 문서: https://learn.chatgpt.com/docs/codex-sdk
