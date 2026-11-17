"""Interactive, hand-guided range calibration; never drives a joint or writes EEPROM."""
from __future__ import annotations

import argparse
from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import select
import shutil
import statistics
import sys
import tempfile
from uuid import uuid4

from .config import load_config
from .drivers.arm import ARM_JOINTS, ArmController, load_calibration


MIN_SPAN_TICKS = 32
JOINT_NAMES = [name for name, _ in ARM_JOINTS]
# The gripper keeps its existing calibration unless it is requested explicitly.
DEFAULT_JOINTS = [name for name in JOINT_NAMES if name != 'gripper']


class RangeCapture:
    """Use three-sample medians, rejecting encoder wrap rather than inventing limits."""

    def __init__(self):
        self.window = deque(maxlen=3)
        self.minimum = self.maximum = None
        self.samples = 0

    def add(self, raw):
        if type(raw) is not int or not 0 <= raw <= 4095:
            raise ValueError(f"유효하지 않은 encoder 값: {raw!r}")
        if self.window and abs(raw - self.window[-1]) > 2048:
            raise ValueError("Encoder가 0/4095 경계를 넘거나 급변했습니다. 이 도구는 영점/EEPROM을 변경하지 않습니다.")
        self.window.append(raw)
        self.samples += 1
        if len(self.window) == 3:
            value = int(statistics.median(self.window))
            self.minimum = value if self.minimum is None else min(self.minimum, value)
            self.maximum = value if self.maximum is None else max(self.maximum, value)

    def limits(self, margin_ticks=0):
        if type(margin_ticks) is not int or not 0 <= margin_ticks <= 1024:
            raise ValueError("margin_ticks는 0..1024 정수여야 합니다.")
        if self.samples < 6 or self.minimum is None or self.maximum is None:
            raise ValueError("측정 샘플이 부족합니다. 양 끝 위치를 잠시 유지하세요.")
        low, high = self.minimum + margin_ticks, self.maximum - margin_ticks
        if high - low < MIN_SPAN_TICKS:
            raise ValueError(f"유효 가동범위가 {MIN_SPAN_TICKS} ticks 미만입니다. 양 끝을 다시 측정하세요.")
        return low, high


def read_offsets(arm):
    offsets = {}
    for name, mid in ARM_JOINTS:
        if arm._read(mid, 33, 1) != 0:
            raise ValueError(f"{name}: 위치 제어 모드가 아닙니다. 모드를 자동 변경하지 않습니다.")
        raw = arm._read(mid, 31)
        if type(raw) is not int or not 0 <= raw <= 4095:
            raise ValueError(f"{name}: 유효하지 않은 homing offset")
        offsets[name] = -(raw & 0x7ff) if raw & 0x800 else raw
    return offsets


def release_arm(arm):
    """Only write torque OFF to IDs 1..6, and verify each acknowledgement/state."""
    for name, mid in ARM_JOINTS:
        try:
            arm._write(mid, 40, 0, 1)
            if arm._read(mid, 40, 1) != 0:
                raise OSError('토크 레지스터가 0이 아닙니다.')
        except OSError as exc:
            raise OSError(f"{name}: 일부 관절의 토크 해제를 확인하지 못했습니다. "
                          f"팔을 지지하고 손으로 움직이지 마세요. {exc}") from exc


def build_calibration(original, captures, offsets, margin_ticks=0):
    if not captures or not set(captures) <= set(JOINT_NAMES):
        raise ValueError("측정한 관절이 없습니다.")
    result = deepcopy(original)
    for name in JOINT_NAMES:
        if name not in captures:
            if offsets[name] != original[name].get('homing_offset'):
                raise ValueError(f"{name}: 기존 보정의 homing_offset({original[name].get('homing_offset')})과 "
                                 f"하드웨어 영점({offsets[name]})이 다릅니다. 기존 범위를 그대로 유지할 수 없으니 "
                                 f"해당 관절도 측정해야 합니다.")
            continue
        low, high = captures[name].limits(margin_ticks)
        result[name].update(range_min=low, range_max=high, homing_offset=offsets[name])
    return result


def save_calibration(path, calibration, *, expected_previous):
    """Validate, back up exact old bytes, and atomically replace the chosen file."""
    path = Path(path).expanduser().resolve()
    current = path.read_bytes() if path.exists() else None
    if current != expected_previous:
        raise RuntimeError("측정 도중 출력 파일이 변경되었습니다. 덮어쓰지 않았습니다.")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    backup = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.' + path.name + '.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(calibration, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        load_calibration(str(temporary))
        if current is not None:
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            backup = path.with_name(path.name + f'.backup-{stamp}-{uuid4().hex[:8]}')
            with backup.open('xb') as stream:
                stream.write(current)
                stream.flush()
                os.fsync(stream.fileno())
            shutil.copymode(path, temporary)
        # Do not overwrite a concurrently updated calibration after the backup step.
        if (path.read_bytes() if path.exists() else None) != expected_previous:
            raise RuntimeError("출력 파일이 변경되어 저장을 취소했습니다.")
        os.replace(temporary, path)
        temporary = None
        return backup
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def capture_joint(arm, name, mid, margin_ticks):
    capture = RangeCapture()
    print(f"\n[{name} / ID {mid}] 손으로 사용할 최소~최대 위치까지 천천히 움직이세요.")
    print("각 끝에서 잠시 유지하세요. Enter: 다음 관절, r+Enter: 다시 측정, q+Enter: 취소")
    while True:
        # Any unconfirmed sample aborts the session; no partial range is silently saved.
        raw = arm._read(mid, 56)
        capture.add(raw)
        print(f"\r현재 {raw:4d} | 최소 {str(capture.minimum):>4} | 최대 {str(capture.maximum):>4} | 샘플 {capture.samples:<6}",
              end='', flush=True)
        if select.select([sys.stdin], [], [], .05)[0]:
            command = sys.stdin.readline()
            if not command or command.strip().lower() == 'q':
                raise KeyboardInterrupt
            if command.strip().lower() == 'r':
                capture = RangeCapture()
                print('\n범위 측정을 초기화했습니다.')
                continue
            if command.strip():
                continue
            try:
                capture.limits(margin_ticks)
            except ValueError as exc:
                print(f"\n{exc}")
                continue
            print()
            return capture


def run_calibration(config, output, joints, margin_ticks, *, arm_factory=ArmController,
                    prompt=input, capture_fn=capture_joint):
    output = Path(output).expanduser().resolve()
    previous = output.read_bytes() if output.exists() else None
    original = load_calibration(config['arm']['calib_path'])
    if not config['arm']['enabled']:
        raise ValueError('arm.enabled가 false입니다.')
    if not joints or len(set(joints)) != len(joints) or not set(joints) <= set(JOINT_NAMES):
        raise ValueError('관절 선택이 잘못되었습니다.')
    if type(margin_ticks) is not int or not 0 <= margin_ticks <= 1024:
        raise ValueError('margin-ticks는 0..1024 정수여야 합니다.')
    print(f"저장할 보정 파일: {output}")
    print('runtime과 teleop을 종료하세요. 팔 전체의 토크를 해제하므로 팔을 손으로 지지하세요.')
    print('자동 이동은 하지 않습니다. 손으로 사용하는 범위를 측정하고 JSON에 저장합니다.')
    print('기계적 끝을 억지로 밀지 마세요. 종료 후에도 토크는 꺼진 상태입니다.')
    if prompt('준비 후 Enter, 취소하려면 q: ').strip():
        return None
    arm = arm_factory(config['arm'])
    try:
        arm.bus.connect()
        offsets = read_offsets(arm)
        release_arm(arm)
        captures = {name: capture_fn(arm, name, mid, margin_ticks)
                    for name, mid in ARM_JOINTS if name in joints}
        if read_offsets(arm) != offsets:
            raise RuntimeError('측정 도중 하드웨어 영점이 변경되었습니다. 저장하지 않습니다.')
        result = build_calibration(original, captures, offsets, margin_ticks)
        print('\n저장 전 검토 (raw encoder ticks):')
        for name in JOINT_NAMES:
            old, new = original[name], result[name]
            label = '측정' if name in captures else '유지'
            print(f"  {name:15s} {old['range_min']:4d}..{old['range_max']:4d} -> "
                  f"{new['range_min']:4d}..{new['range_max']:4d} [{label}] offset={new['homing_offset']}")
        print('관절 각도의 0도는 새 범위의 중간값이 됩니다. 기존 자세·XYZ 매핑을 재확인하세요.')
        print('측정 관절의 homing_offset은 하드웨어에서 읽었으며 EEPROM에는 쓰지 않았습니다.')
        if prompt('저장하려면 save 입력, 그 외 취소: ').strip().lower() != 'save':
            print('저장하지 않았습니다.')
            return None
        backup = save_calibration(output, result, expected_previous=previous)
        print(f'저장 완료: {output}')
        calibration_id = hashlib.sha256(json.dumps(result, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
        print(f'calibration_id (XYZ 매핑 연결용): {calibration_id}')
        if backup:
            print(f'이전 파일 백업: {backup}')
        print('팔을 새 범위 안의 자세로 놓은 뒤 runtime을 다시 시작하세요.')
        if output != Path(config['arm']['calib_path']).resolve():
            print('runtime 설정의 arm.calib_path를 다음 경로로 지정하세요: ' + str(output))
        return output
    finally:
        # Never re-enable torque or run a motion/hold on exit.
        arm.bus.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description='LeKiwi 팔의 수동 최소·최대 가동범위 측정')
    parser.add_argument('--config', type=Path, help='runtime과 같은 로봇 설정 JSON')
    parser.add_argument('--port', help='서보 포트 (예: /dev/ttyACM0). 생략하면 runtime 설정 사용')
    parser.add_argument('--output', type=Path, help='저장 경로. 기본값은 runtime의 arm.calib_path')
    parser.add_argument('--joints', nargs='+', choices=JOINT_NAMES, default=DEFAULT_JOINTS,
                        help='측정할 관절. 기본값은 그리퍼를 제외한 팔 5관절이며 그리퍼는 기존 보정을 유지합니다. '
                             '그리퍼도 측정하려면 gripper를 명시하세요.')
    parser.add_argument('--margin-ticks', type=int, default=0,
                        help='측정 범위 양 끝에서 제외할 여유 ticks (기본 0, 예: 10)')
    args = parser.parse_args(argv)
    if not sys.stdin.isatty():
        parser.error('실물 캘리브레이션은 대화형 터미널에서 실행하세요.')
    try:
        config = load_config(args.config)
        if args.port:
            config['arm']['port'] = args.port
        run_calibration(config, args.output or config['arm']['calib_path'], args.joints, args.margin_ticks)
    except (KeyboardInterrupt, EOFError):
        print('\n캘리브레이션 취소: 파일을 저장하지 않았습니다. 팔을 지지하세요.', file=sys.stderr)
        return 130
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'캘리브레이션 실패: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
