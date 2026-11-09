# Lekiwi Runtime

ORIN에서 실행하는 DEALER 클라이언트. server/harness 패키지에 의존하지 않습니다.

- `RobotClient`: 요청 수신 → observe/execute → JSON 응답.
- `Robot`: 교체할 하드웨어의 인터페이스 (Protocol).
- `OrinRobot`: 기존 ORIN/client의 센서 드라이버와 액션을 사용하는 Adapter.

```bash
uv run --project lekiwi_runtime lekiwi-runtime \
  --server tcp://SERVER_IP:5555 --robot-id kiwi-01 \
  --client-dir /path/to/ORIN/client
```

`--client-dir`는 `hardware.py`, `handlers.py`가 있는 기존 ORIN 폴더입니다.
하드웨어 코드와 설정을 중복 복사하지 않으며 ORIN의 teleop/radar 폴더 배치를 유지합니다.
기존 드라이버의 OpenCV/numpy/pyserial/scservo_sdk 등은 실행 환경에 설치되어 있어야 합니다.
드라이버 의존성이 기존 Python 환경에 있다면 그 환경에 `pip install ./lekiwi_runtime` 후
`lekiwi-runtime ...`으로 실행합니다. `uv run`의 격리 환경에는 별도 설치가 필요합니다.

요청: `{"request_id":"1","op":"observe"}`
실행: `{"request_id":"2","op":"execute","action":"wait","duration_ms":100}`

observe → `status: ok`, execute 성공 → `status: completed`와 실행 후 observation.
미지원 요청 → `rejected`, 하드웨어 예외 → `error`.
현재 액션은 기존 ORIN과 같은 `wait`만 지원합니다.

```bash
uv run --project lekiwi_runtime python -m unittest discover -s lekiwi_runtime/tests -v
```
