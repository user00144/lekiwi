"""Robot memory for a real, non-resettable robot: verified lessons and measured body corrections.

Two stores per robot under ``<memory_dir>/``:

* ``lessons-<robot>.json`` (:class:`LessonMemory`): short lessons written by the planner. A new
  lesson is only a *hypothesis*. It is credited or blamed when a **later** episode explicitly applies
  it (``lessons_applied``): +support when that episode completes and no action that applied it
  failed, +contradict when an action applying it failed or the episode failed. Infrastructure stops
  (uncertain execution, interruption, host errors) change nothing. Lessons become ``verified`` or
  ``retired`` from these counts, so what the planner is told first has been confirmed on this robot.
* ``embodiment-<robot>.json`` (:class:`EmbodimentMemory`): numeric corrections the host measures
  itself, without any model judgement:
  - drive gain = object shift measured by two ``locate`` calls around base moves / commanded
    speed*time (1.0 = the base travels exactly as commanded);
  - locate offset = where an object was released (arm FK) minus where ``locate`` later saw it.
  Medians over recent samples are reported to the planner; the locate offset is applied to new
  ``locate`` results only once enough consistent samples exist.

Nothing here replays actions; it only changes what the planner is told and how locate is corrected.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics

from .episode import atomic_json

MAX_LESSONS = 300
LESSONS_PER_EPISODE = 4
RECALL_VERIFIED = 8
RECALL_HYPOTHESES = 8
SIMILAR_LESSON = 0.45
INFRASTRUCTURE_STOPS = frozenset({
    'uncertain_execution', 'interrupted', 'host_error', 'startup_error', 'observation_failed',
    'observation_invalid', 'post_motion_observation_failed', 'runtime_changed', 'guarded_execution_unavailable',
})
DRIVE_MIN_COMMANDED_M = 0.03
DRIVE_MAX_RANGE_M = 0.5  # beyond this the floor locate is too sensitive to a few pixels
DRIVE_MAX_SENSITIVITY_M = 0.02
DRIVE_GAIN_BOUNDS = (0.4, 1.6)
LOCATE_MAX_OFFSET_M = 0.03
LOCATE_MAX_SPREAD_M = 0.006
READY_SAMPLES = 3
KEEP_SAMPLES = 60


def _now():
    return datetime.now(timezone.utc).isoformat()


def _safe_name(robot_id):
    return ''.join(ch if ch.isalnum() or ch in '-_.' else '_' for ch in str(robot_id)) or 'robot'


def _load(path, default):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (FileNotFoundError, json.JSONDecodeError):
        return default


def _bigrams(text):
    compact = ''.join(str(text).lower().split())
    return {compact[i:i + 2] for i in range(len(compact) - 1)} or {compact}


def similarity(a, b):
    left, right = _bigrams(a), _bigrams(b)
    return len(left & right) / max(1, len(left | right))


def failure_signature(tool, status, error=None, detail=None):
    """Short, stable name of how a skill ended; lessons are keyed by (skill, signature)."""
    if status in ('ok', 'completed'):
        return 'ok'
    codes = (detail or {}).get('ik_reason_codes') or []
    if codes:
        return f'ik:{codes[0]}'
    text = str(error or '').lower()
    for needle, name in (('no longer between the jaws', 'object_slipped'), ('nothing between', 'object_slipped'),
                         ('ik not executable', 'ik_not_executable'), ('wrist_roll', 'roll_limit'),
                         ('not calibrated', 'camera_uncalibrated'), ('not sent', 'rejected'),
                         ('rejected', 'rejected'), ('not found', 'not_found')):
        if needle in text:
            return name
    return 'failed'


def select_lessons(candidates, limit=LESSONS_PER_EPISODE):
    """Pick an episode's lessons: conclusions first, then failures, then the most recent; no near-duplicates.

    Each candidate is {'text', 'skill', 'signature', 'final': bool}; order is chronological.
    """
    ranked = sorted(enumerate(candidates), key=lambda item: (item[1].get('final', False),
                                                            item[1].get('signature') not in ('ok', None), item[0]),
                    reverse=True)
    chosen = []
    for _, candidate in ranked:
        text = ' '.join(str(candidate.get('text') or '').split())
        if len(text) < 8 or any(similarity(text, other['text']) >= SIMILAR_LESSON for other in chosen):
            continue
        chosen.append({**candidate, 'text': text})
        if len(chosen) >= limit:
            break
    return chosen


class LessonMemory:
    def __init__(self, memory_dir, robot_id):
        self.path = Path(memory_dir) / f'lessons-{_safe_name(robot_id)}.json'
        self.robot_id = robot_id
        data = _load(self.path, {})
        self.lessons = list(data.get('lessons') or [])
        self.next_id = int(data.get('next_id') or len(self.lessons) + 1)

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        live = [lesson for lesson in self.lessons if lesson['status'] != 'retired']
        retired = [lesson for lesson in self.lessons if lesson['status'] == 'retired']
        self.lessons = live[-MAX_LESSONS:] + retired[-max(0, MAX_LESSONS - len(live)):]
        atomic_json(self.path, {'version': 1, 'robot_id': self.robot_id, 'next_id': self.next_id,
                                'lessons': self.lessons})

    def get(self, lesson_id):
        return next((lesson for lesson in self.lessons if lesson['id'] == lesson_id), None)

    def propose(self, text, *, skill, signature, run, source=None):
        """Store a hypothesis, or count a re-proposal of a similar existing lesson."""
        text = ' '.join(str(text or '').split())[:400]
        if len(text) < 8:
            return None
        for lesson in self.lessons:
            if (lesson['status'] != 'retired' and lesson['skill'] == skill and lesson['signature'] == signature
                    and similarity(lesson['text'], text) >= SIMILAR_LESSON):
                if lesson.get('source_run') != run and run not in lesson.get('proposed_in', []):
                    lesson['proposals'] = lesson.get('proposals', 1) + 1
                    lesson['proposed_in'] = (lesson.get('proposed_in', []) + [run])[-10:]
                return lesson['id']
        lesson = {'id': f'L{self.next_id:03d}', 'text': text, 'skill': skill, 'signature': signature,
                  'status': 'hypothesis', 'support': 0, 'contradict': 0, 'proposals': 1,
                  'source_run': run, 'proposed_in': [run], 'source': source or {}, 'created_at': _now(),
                  'uses': []}
        self.next_id += 1
        self.lessons.append(lesson)
        return lesson['id']

    @staticmethod
    def _status(lesson):
        support, contradict = lesson['support'], lesson['contradict']
        if contradict >= 2 and contradict > support:
            return 'retired'
        if support >= 2 and support >= 2 * contradict:
            return 'verified'
        return 'hypothesis'

    def credit(self, applied, failed, episode_status, run):
        """One update per applied lesson per episode; returns {lesson_id: 'support'|'contradict'}."""
        if episode_status in INFRASTRUCTURE_STOPS:
            return {}
        episode_ok = episode_status == 'completion_claim'
        changes = {}
        for lesson_id in sorted(set(applied)):
            lesson = self.get(lesson_id)
            if lesson is None or lesson.get('source_run') == run:
                continue  # a lesson is never confirmed by the episode that wrote it
            outcome = 'support' if episode_ok and lesson_id not in failed else 'contradict'
            lesson[outcome] += 1
            lesson['uses'] = (lesson.get('uses', []) + [{'run': run, 'outcome': outcome, 'episode': episode_status,
                                                          'at': _now()}])[-12:]
            lesson['status'] = self._status(lesson)
            changes[lesson_id] = outcome
        return changes

    def recall(self):
        """Lessons for the planner: verified first, then the most re-proposed / recent hypotheses."""
        verified = sorted((lesson for lesson in self.lessons if lesson['status'] == 'verified'),
                          key=lambda lesson: (lesson['support'] - lesson['contradict'], lesson['support']),
                          reverse=True)[:RECALL_VERIFIED]
        hypotheses = [lesson for lesson in self.lessons if lesson['status'] == 'hypothesis']
        hypotheses = sorted(hypotheses, key=lambda lesson: (lesson['signature'] != 'ok', lesson.get('proposals', 1),
                                                            lesson['created_at']), reverse=True)[:RECALL_HYPOTHESES]
        return [{'id': lesson['id'], 'status': lesson['status'], 'skill': lesson['skill'],
                 'when': lesson['signature'], 'text': lesson['text'], 'support': lesson['support'],
                 'contradict': lesson['contradict']} for lesson in verified + hypotheses]

    def counts(self):
        counts = {'verified': 0, 'hypothesis': 0, 'retired': 0}
        for lesson in self.lessons:
            counts[lesson['status']] = counts.get(lesson['status'], 0) + 1
        return counts


def camera_key(camera_config):
    """Locate offsets belong to one camera extrinsic calibration."""
    transform = (camera_config or {}).get('T_base_camera')
    return hashlib.sha256(json.dumps(transform).encode()).hexdigest()[:12] if transform else None


class EmbodimentMemory:
    def __init__(self, memory_dir, robot_id):
        self.path = Path(memory_dir) / f'embodiment-{_safe_name(robot_id)}.json'
        self.robot_id = robot_id
        data = _load(self.path, {})
        self.drive_samples = list(data.get('drive_samples') or [])
        self.locate_samples = list(data.get('locate_samples') or [])

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        atomic_json(self.path, {'version': 1, 'robot_id': self.robot_id,
                                'drive_samples': self.drive_samples[-KEEP_SAMPLES:],
                                'locate_samples': self.locate_samples[-KEEP_SAMPLES:]})

    # -- drive gain -----------------------------------------------------------------
    def observe_drive(self, before_xyz, after_xyz, commanded_xy, *, run, sensitivity=(None, None), label=None):
        """Object shift between two locates vs commanded base travel; returns the sample or a rejection."""
        cx, cy = (float(value) for value in commanded_xy)
        commanded = math.hypot(cx, cy)
        reason = None
        if commanded < DRIVE_MIN_COMMANDED_M:
            reason = f'commanded travel {commanded:.3f} m < {DRIVE_MIN_COMMANDED_M} m'
        elif max(math.hypot(*before_xyz[:2]), math.hypot(*after_xyz[:2])) > DRIVE_MAX_RANGE_M:
            reason = f'object farther than {DRIVE_MAX_RANGE_M} m; locate too sensitive'
        elif any(value is not None and value > DRIVE_MAX_SENSITIVITY_M for value in sensitivity):
            reason = f'locate sensitivity above {DRIVE_MAX_SENSITIVITY_M} m per 3 px'
        if reason:
            return {'accepted': False, 'reason': reason}
        # The object moves opposite to the base in arm_base: shift = -gain * commanded.
        dx, dy = after_xyz[0] - before_xyz[0], after_xyz[1] - before_xyz[1]
        gain = -(dx * cx + dy * cy) / (commanded ** 2)
        if not DRIVE_GAIN_BOUNDS[0] <= gain <= DRIVE_GAIN_BOUNDS[1]:
            return {'accepted': False, 'reason': f'implausible gain {gain:.2f}', 'gain': round(gain, 3)}
        sample = {'at': _now(), 'run': run, 'label': label, 'commanded_m': [round(cx, 4), round(cy, 4)],
                  'measured_shift_m': [round(dx, 4), round(dy, 4)], 'gain': round(gain, 3),
                  'axis': 'forward' if abs(cx) >= 2 * abs(cy) else 'lateral' if abs(cy) >= 2 * abs(cx) else 'mixed'}
        self.drive_samples.append(sample)
        return {'accepted': True, **sample}

    def drive_gain(self):
        gains = [sample['gain'] for sample in self.drive_samples[-20:]]
        if not gains:
            return {'gain': None, 'samples': 0, 'ready': False}
        median = statistics.median(gains)
        spread = statistics.median(abs(value - median) for value in gains)
        return {'gain': round(median, 3), 'spread': round(spread, 3), 'samples': len(gains),
                'ready': len(gains) >= READY_SAMPLES}

    # -- locate offset ------------------------------------------------------------------
    def observe_locate(self, expected_xy, located_xy, *, run, camera, label=None):
        offset = [expected_xy[0] - located_xy[0], expected_xy[1] - located_xy[1]]
        if math.hypot(*offset) > 3 * LOCATE_MAX_OFFSET_M:
            return {'accepted': False, 'reason': f'offset {math.hypot(*offset):.3f} m is implausible'}
        sample = {'at': _now(), 'run': run, 'label': label, 'camera': camera,
                  'expected_xy_m': [round(v, 4) for v in expected_xy], 'located_xy_m': [round(v, 4) for v in located_xy],
                  'offset_m': [round(v, 4) for v in offset]}
        self.locate_samples.append(sample)
        return {'accepted': True, **sample}

    def locate_offset(self, camera):
        samples = [sample['offset_m'] for sample in self.locate_samples[-20:] if sample.get('camera') == camera]
        if not samples:
            return {'offset_m': None, 'samples': 0, 'ready': False}
        median = [statistics.median(axis) for axis in zip(*samples)]
        spread = max(statistics.median(abs(value - median[i]) for value in axis) for i, axis in enumerate(zip(*samples)))
        ready = (len(samples) >= READY_SAMPLES and spread <= LOCATE_MAX_SPREAD_M
                 and math.hypot(*median) <= LOCATE_MAX_OFFSET_M)
        return {'offset_m': [round(v, 4) for v in median], 'spread_m': round(spread, 4), 'samples': len(samples),
                'ready': ready}

    def summary(self, camera=None):
        drive, offset = self.drive_gain(), self.locate_offset(camera)
        return {
            'drive': {**drive, 'meaning': 'measured base travel / commanded speed*time (1.0 = exact); '
                                          'from object shifts between locate calls around base moves'},
            'locate_offset': {**offset, 'applied_to_locate': offset['ready'],
                              'meaning': 'released position (arm FK) minus later locate of the same object'},
        }


class MemoryTracker:
    """Collects body measurements during one plan-act episode from its skill results.

    Anchors: the last raw locate per label plus the base moves since. Arm actions that may touch
    objects (pick / place / move_to / gripper) clear anchors, so only clean locate-drive-locate
    chains become drive samples. A place followed by a locate of the same label (no arm action in
    between) becomes a locate-offset sample.
    """

    TOUCHES_OBJECTS = frozenset({'pick', 'place', 'move_to', 'gripper'})

    def __init__(self, embodiment, *, run, camera=None):
        self.embodiment, self.run, self.camera = embodiment, run, camera
        self.anchors, self.placed, self.held_label = {}, None, None

    def after_locate(self, label, raw_xyz, sensitivity):
        measurements = {}
        anchor = self.anchors.get(label)
        if anchor and anchor['moves'] and anchor['clean']:
            commanded = [sum(move[i] for move in anchor['moves']) for i in (0, 1)]
            measurements['drive_measurement'] = self.embodiment.observe_drive(
                anchor['xyz'], raw_xyz, commanded, run=self.run, sensitivity=(anchor['sensitivity'], sensitivity),
                label=label)
        if self.placed and self.placed['label'] == label and self.placed['clean']:
            expected = [self.placed['xy'][i] - sum(move[i] for move in self.placed['moves']) for i in (0, 1)]
            measurements['locate_measurement'] = self.embodiment.observe_locate(
                expected, raw_xyz[:2], run=self.run, camera=self.camera, label=label)
            self.placed = None
        self.anchors[label] = {'xyz': list(raw_xyz), 'sensitivity': sensitivity, 'moves': [], 'clean': True}
        return measurements

    def after_drive(self, x, y, theta, duration_ms):
        move = (x * duration_ms / 1000, y * duration_ms / 1000)
        for record in [*self.anchors.values(), *([self.placed] if self.placed else [])]:
            record['moves'].append(move)
            if abs(theta) > 1e-6:
                record['clean'] = False  # rotation changes the object's arm_base position non-linearly

    def after_arm_skill(self, tool, *, ok, target_xyz=None, located=None, release_xyz=None):
        if tool not in self.TOUCHES_OBJECTS:
            return
        self.anchors.clear()
        if tool == 'pick' and ok and target_xyz is not None:
            nearest = min(((math.dist(item['xyz_m'][:2], target_xyz[:2]), label)
                           for label, item in (located or {}).items() if item.get('xyz_m')), default=None)
            self.held_label = nearest[1] if nearest and nearest[0] <= 0.04 else None
            self.placed = None
        elif tool == 'place' and ok and release_xyz is not None and self.held_label:
            self.placed = {'label': self.held_label, 'xy': list(release_xyz[:2]), 'moves': [], 'clean': True}
            self.held_label = None
        else:
            self.placed = None


def import_experience(experience_path, memory_dir, robot_id):
    """Seed memory from an existing experience.jsonl: hypotheses and drive samples, nothing verified.

    A planner lesson is written after seeing the previous skill result, so it is attributed to that
    result (skill and failure signature), not to the action it was attached to.
    """
    lessons, embodiment = LessonMemory(memory_dir, robot_id), EmbodimentMemory(memory_dir, robot_id)
    trackers, previous, candidates = {}, {}, {}
    counts = {'lessons': 0, 'drive_samples': 0}
    for line in Path(experience_path).read_text(encoding='utf-8').splitlines():
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if record.get('robot_id') != robot_id:
            continue
        run = record.get('run') or 'unknown'
        kind = record.get('kind')
        if kind in ('attempt', 'verify') and record.get('lesson'):
            candidates.setdefault(run, []).append({'text': record['lesson'], 'skill': kind,
                                                   'signature': record.get('verdict') or 'unknown', 'final': True})
            continue
        if kind != 'action':
            continue
        tracker = trackers.setdefault(run, MemoryTracker(embodiment, run=run))
        tool, status, result = record.get('tool'), record.get('status'), record.get('result') or {}
        args = record.get('args') or {}
        if record.get('lesson'):
            seen = previous.get(run)
            skill = seen['tool'] if seen else 'general'
            signature = failure_signature(seen['tool'], seen['status'], seen.get('error'), seen.get('detail')) if seen else 'start'
            candidates.setdefault(run, []).append({'text': record['lesson'], 'skill': skill, 'signature': signature})
        previous[run] = record
        if tool == 'locate' and status == 'ok' and result.get('xyz_m'):
            found = tracker.after_locate(args.get('label'), result['xyz_m'], None)
            if (found.get('drive_measurement') or {}).get('accepted'):
                counts['drive_samples'] += 1
        elif tool == 'drive' and status == 'ok' and args.get('drive'):
            drive = args['drive']
            tracker.after_drive(drive['x'], drive['y'], drive['theta'], drive['duration_ms'])
        else:
            tracker.after_arm_skill(tool, ok=status == 'ok')
    for run, items in candidates.items():
        for lesson in select_lessons(items):
            before = lessons.next_id
            lessons.propose(lesson['text'], skill=lesson['skill'], signature=lesson['signature'], run=run,
                            source={'imported_from': str(experience_path)})
            counts['lessons'] += lessons.next_id - before
    lessons.save()
    embodiment.save()
    return counts


def main(argv=None):
    parser = argparse.ArgumentParser(description='Inspect or seed LeKiwi robot memory')
    sub = parser.add_subparsers(dest='command', required=True)
    show = sub.add_parser('show', help='Print lessons and measured body corrections')
    show.add_argument('--memory-dir', type=Path, default=Path('runs/memory'))
    show.add_argument('--robot-id', default='kiwi-01')
    seed = sub.add_parser('import-experience', help='Seed hypotheses and drive samples from experience.jsonl')
    seed.add_argument('experience', type=Path)
    seed.add_argument('--memory-dir', type=Path, default=Path('runs/memory'))
    seed.add_argument('--robot-id', default='kiwi-01')
    args = parser.parse_args(argv)
    if args.command == 'import-experience':
        print(json.dumps(import_experience(args.experience, args.memory_dir, args.robot_id), ensure_ascii=False))
        return 0
    lessons, embodiment = LessonMemory(args.memory_dir, args.robot_id), EmbodimentMemory(args.memory_dir, args.robot_id)
    print(json.dumps({'counts': lessons.counts(), 'embodiment': embodiment.summary(),
                      'recall': lessons.recall()}, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
