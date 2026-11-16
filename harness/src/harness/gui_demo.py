"""A fixed, clearly simulated episode for viewing the console without a robot/model."""
import asyncio
import base64
import json
from pathlib import Path
import struct
import time
from uuid import uuid4
import zlib

from mcp.types import ImageContent, TextContent

from .episode import EpisodeLimits, EpisodeRunner, atomic_json


def camera_png(view, angle):
    """Draw a synthetic grid, target and gripper; no external images or dependencies."""
    width, height = 480, 300
    pixels = bytearray(bytes((29, 43, 49)) * width * height)

    def box(x, y, w, h, color):
        for row in range(max(0, y), min(height, y + h)):
            left, right = max(0, x), min(width, x + w)
            if right > left:
                start = (row * width + left) * 3
                pixels[start:start + (right - left) * 3] = bytes(color) * (right - left)

    for x in range(0, width, 40):
        box(x, 0, 1, height, (44, 61, 68))
    for y in range(0, height, 40):
        box(0, y, width, 1, (44, 61, 68))
    box(20, 20, 9, 9, (111, 202, 177))
    if view == 'front':
        box(0, 205, width, 95, (49, 64, 68))
        box(276, 159, 45, 46, (237, 160, 91))
        shift = round(angle * 3)
        box(130 + shift, 94, 32, 90, (151, 172, 177))
        box(119 + shift, 178, 54, 15, (198, 213, 214))
        box(119 + shift, 190, 10, 30, (198, 213, 214))
        box(163 + shift, 190, 10, 30, (198, 213, 214))
    else:
        box(211, 133, 58, 56, (237, 160, 91))
        shift = round(angle * 3)
        box(120 + shift, 77, 110, 14, (198, 213, 214))
        box(120 + shift, 90, 15, 76, (151, 172, 177))
        box(215 + shift, 90, 15, 76, (151, 172, 177))
    box(width // 2 - 7, height // 2, 15, 1, (111, 202, 177))
    box(width // 2, height // 2 - 7, 1, 15, (111, 202, 177))

    def chunk(kind, data):
        return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind + data))

    rows = b''.join(b'\0' + pixels[row * width * 3:(row + 1) * width * 3] for row in range(height))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', width, height, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))


class DemoTools:
    def __init__(self, delay=.4):
        self.delay = delay
        self.runtime_id = 'demo-' + uuid4().hex
        self.sequence = 0
        self.observation_id = None
        self.joints = dict(shoulder_pan=0., shoulder_lift=-15., elbow_flex=35.,
                           wrist_flex=-20., wrist_roll=0., gripper=65.)

    def observation(self):
        self.sequence += 1
        self.observation_id = f'{self.runtime_id}-obs-{self.sequence}'
        now = time.time()
        packet = {
            'status': 'ok', 'capabilities': {'guarded_execution': 1},
            'action_context': {'runtime_id': self.runtime_id, 'observation_id': self.observation_id,
                               'movement_allowed': True, 'max_age_s': 60},
            'observation': {
                'runtime_id': self.runtime_id, 'observation_id': self.observation_id,
                'timestamp': now, 'simulated': True,
                'cameras': {name: {'available': True, 'timestamp': now, 'age_s': 0,
                                   'width': 480, 'height': 300, 'simulated': True}
                            for name in ('front', 'wrist')},
                'ego': {'available': True, 'timestamp': now, 'simulated': True,
                        'joints': dict(self.joints),
                        'limits': {name: {'min': 0 if name == 'gripper' else -180,
                                         'max': 100 if name == 'gripper' else 180,
                                         'max_speed': 50 if name == 'gripper' else 30}
                                   for name in self.joints}},
                'radar': {'available': True, 'simulated': True, 'timestamp': now, 'count': 3,
                          'points_xyzd': [[.2, .4, .1, 0], [-.3, .6, .1, 0], [.1, .8, .1, 0]]},
                'imu': {'available': False, 'simulated': True, 'error': '데모에서는 IMU를 생성하지 않습니다.'},
            },
        }
        images = []
        for name in ('front', 'wrist'):
            images.extend([TextContent(type='text', text=name), ImageContent(
                type='image', mimeType='image/png',
                data=base64.b64encode(camera_png(name, self.joints['wrist_roll'])).decode())])
        return packet, images

    async def observe(self, robot_id):
        await asyncio.sleep(self.delay)
        packet, images = self.observation()
        return [TextContent(type='text', text=json.dumps(packet)), *images]

    async def execute(self, robot_id, **command):
        if command['runtime_id'] != self.runtime_id or command['observation_id'] != self.observation_id:
            raise ValueError('Demo command uses a different observation')
        await asyncio.sleep(self.delay)
        self.joints.update(command.get('joints') or {})
        finished = time.time()
        await asyncio.sleep(self.delay)
        packet, images = self.observation()
        packet.update(status='completed', execution={
            'action_id': command['action_id'], 'outcome': 'completed', 'command_sent': True,
            'arm_reached': True, 'base_stop_command_sent': None, 'base_stop_verified': None,
            'motion_finished_at': finished, 'simulated': True,
        })
        return [TextContent(type='text', text=json.dumps(packet)), *images]


class DemoPlanner:
    def __init__(self, delay=1.8):
        self.delay = delay
        self.step = 0

    async def decide(self, context, images):
        await asyncio.sleep(self.delay)
        self.step += 1
        choice = 'observe' if self.step == 1 else 'execute' if self.step <= 3 else 'task_complete'
        command = None
        if choice == 'execute':
            command = {'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': 500,
                       'joints': {name: float((self.step - 1) * 5) if name == 'wrist_roll' else None
                                  for name in context['current']['observation']['ego']['joints']}}
        reasons = {
            1: '고정 시연입니다. 합성 전방·손목 이미지를 다시 관찰한 뒤 작은 관절 동작을 표시합니다.',
            2: '합성 관절값에서 손목 회전이 0°입니다. 5° 목표를 한 번 실행하고 변화량을 비교합니다.',
            3: '직전 시연에서 손목 회전이 5° 증가했습니다. 다음 10° 목표도 별도 동작으로 실행합니다.',
            4: '합성 피드백에서 손목 회전 10°를 확인했습니다. 새 관찰로 시연 완료 주장을 다시 확인합니다.',
            5: '재관찰에서도 합성 목표값이 유지됩니다. 시연을 종료하며 실제 작업 성공을 뜻하지는 않습니다.',
        }
        return {
            'robot_id': context['robot_id'],
            'observation_id': context['current']['observation']['observation_id'],
            'tool': choice, 'plan': ['합성 관찰 확인', '손목 회전 두 단계 시연', '측정 변화 비교', '재관찰 후 종료'],
            'reason': reasons.get(self.step, reasons[5]),
            'expected': '손목 회전 목표와 합성 이미지가 함께 변합니다.' if command else '추가 관찰에서 합성 상태를 확인합니다.',
            'feedback': 'initial' if self.step < 3 else 'progress',
            'evidence': '고정된 모의 데이터와 스크립트 판단입니다. 실제 모델·로봇은 사용하지 않았습니다.',
            'command': command,
        }


async def run_demo(server, model, robot_id, instruction, *, snapshot=None, run_dir=None, limits=None, _delay=1.):
    """Match run_episode's interface while never creating network or model clients."""
    if snapshot is not None:
        raise ValueError('Demo does not use snapshot inputs')
    if run_dir is None:
        raise ValueError('Demo requires an explicit run directory')
    runner = EpisodeRunner(DemoPlanner(1.8 * _delay), DemoTools(.4 * _delay), robot_id,
                           instruction, run_dir, limits or EpisodeLimits())
    path = Path(run_dir)
    metadata = json.loads((path / 'run.json').read_text())
    atomic_json(path / 'run.json', {**metadata, 'mode': 'demo', 'simulated': True})
    runner.record.event('demo_started', {'simulated': True, 'model_called': False,
                                       'note': '입력한 작업과 무관한 고정 시연입니다. 실제 로봇·모델에 연결하지 않습니다.'})
    result = await runner.run()
    result.update(simulated=True, demo=True)
    atomic_json(path / 'result.json', result)
    return result
