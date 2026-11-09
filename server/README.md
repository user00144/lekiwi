# ZeroMQ Server

Lekiwi-Server간 통신용
ZeroMQ 프로토콜 기반 서버 코드

기능
- Lekiwi와의 데이터 수신 : observe()
- Lekiwi에 제어명령 송신 : execute()
- Lekiwi 제어 후 보고(데이터) 수신 : execute 응답(REP)

## 실행

```bash
uv run --project server server
```

- 로봇: `tcp://*:5555` / ROUTER ↔ DEALER
- harness: `tcp://127.0.0.1:5556` / ROUTER ↔ REQ
- 호출: `{"robot_id":"kiwi-01","op":"observe"}`
- 실행: `{"robot_id":"kiwi-01","op":"execute","command":{"action":"wait","duration_ms":100}}`

`RobotServer`는 로봇 통신, `ServerService`는 호출 인터페이스만 담당합니다.
응답은 ORIN의 JSON 그대로 반환합니다. 같은 로봇에 진행 중인 요청이 있으면
새 요청은 거절합니다. timeout 시 실행 결과는 미확정이며 자동 재전송하지 않습니다.
시각화·MCP·하드웨어·실패 기록은 포함하지 않습니다.
기존 TEMP 대시보드 서버와 5555 포트를 동시에 사용할 수 없습니다.
