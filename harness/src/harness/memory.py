"""Robot memory for a real, non-resettable robot: an ASPIRE-style skill library and body corrections.

Under ``<memory_dir>/``:

* ``skill-library/`` (:class:`SkillLibrary`): topic Markdown files in the entry schema of
  ``harness/.claude/skills/lekiwi-mobile-manip/skills`` (Trigger / Contrast / Snippet / Provenance),
  rendered from ``index.json``. Following ASPIRE, Voyager and ViReSkill, an entry is admitted **only
  from an episode whose success was verified**: the independent VERIFY turn confirmed it on a fresh
  observation *and* host measurements agree (e.g. the gripper still holds after a pick, or opened
  after a place). The Snippet is the skill sequence the host actually executed, not model text.
  - ``workflow`` entries (``episodic-workflows.md``): the verified successful path of a task.
  - ``repair`` entries (topic by failed skill): a failure inside the verified episode and the
    executed steps that got past it (ASPIRE's validated repair).
  Failures alone never become entries. Reuse is recorded: a later verified success that applied an
  entry adds a confirmation; a failed reuse is recorded and shown, so stale entries are visible.
  ``frozen=True`` makes the library read-only (held-out evaluation, as in ASPIRE).
* ``failure-ledger-<robot>.json`` + ``failure-ledger.md`` (:class:`FailureLedger`): ASPIRE's per-task
  analysis of directions already tried. Every failed attempt (a skill failure, or a task-level
  failure where every call "succeeded" but the goal was not met) is recorded as an *approach*: the
  planner's one-line description of what was tried and where it failed (it also names the ledger
  record of the same approach, as ASPIRE's analysis is LLM-maintained), with host-recorded evidence
  (last executed calls, last arm target, failing call) and the reported reason. Status:
  ``tried`` (failed once), ``eliminated`` (the same approach failed twice), ``blocked`` (could not be
  executed: IK / rejection / missing calibration), ``resolved`` (a later verified success on a
  similar task). Infrastructure stops are not recorded. Planning sees the unresolved approaches as
  directions not to repeat unless the condition that caused the failure changed.
* ``embodiment-<robot>.json`` (:class:`EmbodimentMemory`): numeric corrections the host measures
  itself, without any model judgement:
  - drive gain = object shift measured by two ``locate`` calls around base moves / commanded
    speed*time (1.0 = the base travels exactly as commanded);
  - locate offset = where an object was released (arm FK) minus where ``locate`` later saw it.

Nothing here replays actions; it only changes what the planner is told and how locate is corrected.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import statistics

from .episode import atomic_json

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
SIMILAR_TASK = 0.6
RECALL_WORKFLOWS = 3
RECALL_REPAIRS = 5
RECALL_LEDGER = 6
LEDGER_SIMILAR_TASK = 0.5
BLOCKING_SIGNATURES = ('ik', 'rejected', 'camera_uncalibrated', 'roll_limit')


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
    """Short, stable name of how a skill ended; repair entries are keyed by (skill, signature)."""
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


TOPICS = {
    'episodic-workflows.md': 'Episodic Workflows: verified task sequences',
    'mobile-grasp.md': 'Mobile Grasping: verified pick / place / arm repairs',
    'search-and-memory.md': 'Search and Locate: verified locate repairs',
    'navigation.md': 'Navigation: verified base-approach repairs',
    'lekiwi-operations.md': 'LeKiwi Operations: verified recovery from rejections and arm state',
}
REPAIR_TOPIC = {'pick': 'mobile-grasp.md', 'place': 'mobile-grasp.md', 'move_to': 'mobile-grasp.md',
                'gripper': 'mobile-grasp.md', 'locate': 'search-and-memory.md', 'drive': 'navigation.md'}
GOAL_TOOLS = frozenset({'pick', 'place'})


def _call(action):
    """One executed skill call as a snippet line, with the host-measured result."""
    tool, args, result = action.get('tool'), action.get('args') or {}, action.get('result') or {}
    if tool == 'locate':
        text = f'locate({args.get("label")!r})'
        if result.get('xyz_m'):
            text += f' -> xyz {result["xyz_m"]}, reachable={result.get("reachable")}'
    elif tool == 'drive':
        drive = args.get('drive') or {}
        text = (f'drive(x={drive.get("x")} m/s, y={drive.get("y")} m/s, theta={drive.get("theta")} deg/s, '
                f'{drive.get("duration_ms")} ms)')
    elif tool in ('pick', 'place'):
        text = f'{tool}({args.get("xyz_m")})'
        if result.get('gripper_after_close') is not None:
            text += f' -> gripper {result["gripper_after_close"]}% after close'
    elif tool == 'move_to':
        text = f'move_to({args.get("xyz_m")}, approach={args.get("approach")!r})'
    elif tool == 'gripper':
        text = f'gripper({args.get("gripper_percent")}%)'
    else:
        text = f'{tool}()'
    status = action.get('status')
    if status != 'ok':
        text += f'  # {status}: {action.get("error") or ""}'.rstrip()
    return text


def _shape(actions):
    """Tool sequence with repeats collapsed (locate>drive>locate>pick), used to merge workflows."""
    tools = []
    for action in actions:
        if action.get('status') == 'ok' and action.get('tool') not in ('observe',) and (
                not tools or tools[-1] != action['tool']):
            tools.append(action['tool'])
    return '>'.join(tools)


class SkillLibrary:
    def __init__(self, library_dir, robot_id, *, frozen=False, templates_dir=None):
        self.dir = Path(library_dir)
        self.robot_id, self.frozen = robot_id, frozen
        self.templates_dir = Path(templates_dir) if templates_dir else None
        data = _load(self.dir / 'index.json', {})
        self.entries = list(data.get('entries') or [])
        self.next_id = int(data.get('next_id') or len(self.entries) + 1)

    # -- persistence -------------------------------------------------------------------
    def save(self):
        if self.frozen:
            return
        self.dir.mkdir(parents=True, exist_ok=True)
        atomic_json(self.dir / 'index.json', {'version': 1, 'robot_id': self.robot_id, 'next_id': self.next_id,
                                              'entries': self.entries})
        for topic, title in TOPICS.items():
            (self.dir / topic).write_text(self.render(topic, title), encoding='utf-8')

    def render(self, topic, title):
        lines = [f'# {title}', '',
                 'Auto-curated by the plan-act loop from episodes verified on this robot (VERIFY turn on a fresh',
                 'observation + host measurements). Source of truth: `index.json`; do not edit by hand.', '']
        entries = [entry for entry in self.entries if entry['topic'] == topic]
        if not entries:
            lines.append('_No verified entries yet._')
        for entry in entries:
            lines += [f'## {entry["id"]} · {entry["title"]}', '',
                      f'**Trigger:** {entry["trigger"]}', '']
            if entry.get('contrast'):
                lines += [f'**Contrast:** {entry["contrast"]}', '']
            lines += ['**Snippet (executed skill calls, host-recorded):**', '', '```text', *entry['snippet'], '```', '']
            if entry.get('strategy'):
                lines += [f'**Strategy (verifier note):** {entry["strategy"]}', '']
            prov = entry['provenance']
            lines += [f'**Provenance:** task="{entry["task"]}", run={Path(prov["run"]).name}, date={prov["date"][:10]}, '
                      f'verification={json.dumps(prov["verification"], ensure_ascii=False)}; '
                      f'confirmed in {len(entry["confirmed_in"])} later run(s), failed on reuse '
                      f'{len(entry["failed_reuse_in"])} time(s).', '']
        return '\n'.join(lines) + '\n'

    # -- writing -------------------------------------------------------------------------
    def _new(self, **fields):
        entry = {'id': f'S{self.next_id:03d}', 'created_at': _now(), 'confirmed_in': [], 'failed_reuse_in': [],
                 **fields}
        self.next_id += 1
        self.entries.append(entry)
        return entry['id']

    def _confirm(self, entry, run):
        if run not in entry['confirmed_in'] and run != entry['provenance']['run']:
            entry['confirmed_in'].append(run)

    def admit_episode(self, *, task, run, actions, verification, strategy='', labels=(), final_actions=None):
        """Admit verified knowledge from one episode; returns {'added': [...], 'confirmed': [...]} or a skip.

        ``actions``: every executed call of the episode (repairs); ``final_actions``: the verified
        attempt only (workflow; defaults to ``actions``).
        """
        if self.frozen:
            return {'skipped': 'library frozen (evaluation mode)'}
        if not verification.get('passed'):
            return {'skipped': 'episode success not verified', 'verification': verification}
        provenance = {'run': run, 'date': _now(), 'verification': verification}
        added, confirmed = [], []
        final_actions = actions if final_actions is None else final_actions
        ok_path = [action for action in final_actions if action.get('status') == 'ok' and action.get('tool') != 'observe']
        shape = _shape(final_actions)
        twin = next((entry for entry in self.entries if entry['kind'] == 'workflow' and entry['shape'] == shape
                     and similarity(entry['task'], task) >= SIMILAR_TASK), None)
        if twin:
            self._confirm(twin, run)
            confirmed.append(twin['id'])
        elif ok_path:
            added.append(self._new(kind='workflow', topic='episodic-workflows.md', task=task, labels=list(labels),
                                   shape=shape, title=f'{task[:70]} ({shape})',
                                   trigger=f'Task similar to "{task}"' + (f' with {", ".join(labels)}' if labels else ''),
                                   contrast=None, snippet=[_call(action) for action in ok_path],
                                   strategy=strategy, provenance=provenance))
        # Repairs: each failed call, followed by the executed steps up to the next success of the same
        # tool (or of a goal tool, or the end of the verified episode).
        for index, action in enumerate(actions):
            if action.get('status') == 'ok' or action.get('tool') in ('observe', None):
                continue
            tool = action['tool']
            signature = failure_signature(tool, action.get('status'), action.get('error'), action.get('detail'))
            fix = []
            for later in actions[index + 1:]:
                fix.append(later)
                if later.get('status') == 'ok' and (later['tool'] == tool or later['tool'] in GOAL_TOOLS):
                    break
            if not any(item.get('status') == 'ok' for item in fix):
                continue
            fix_shape = _shape(fix)
            twin = next((entry for entry in self.entries if entry['kind'] == 'repair' and entry['failed_tool'] == tool
                         and entry['signature'] == signature and entry['shape'] == fix_shape
                         and similarity(entry['task'], task) >= SIMILAR_TASK), None)
            if twin:
                self._confirm(twin, run)
                confirmed.append(twin['id'])
                continue
            added.append(self._new(kind='repair', topic=REPAIR_TOPIC.get(tool, 'lekiwi-operations.md'), task=task,
                                   labels=list(labels), failed_tool=tool, signature=signature, shape=fix_shape,
                                   title=f'{tool} {signature} -> {fix_shape}',
                                   trigger=f'{tool} ends with {signature} during a task like "{task}"',
                                   contrast=f'{_call(action)} failed; the steps below then succeeded in the same verified episode.',
                                   snippet=[_call(item) for item in fix], strategy=strategy, provenance=provenance))
        return {'added': added, 'confirmed': confirmed}

    def record_use(self, ids, *, run, status, verified):
        """A later run that applied entries: confirmation if it was a verified success, else a failed reuse."""
        if self.frozen or status in INFRASTRUCTURE_STOPS:
            return {}
        changes = {}
        for entry in self.entries:
            if entry['id'] not in ids or entry['provenance']['run'] == run:
                continue
            if verified:
                self._confirm(entry, run)
                changes[entry['id']] = 'confirmed'
            elif run not in entry['failed_reuse_in']:
                entry['failed_reuse_in'].append(run)
                changes[entry['id']] = 'failed_reuse'
        return changes

    # -- reading -------------------------------------------------------------------------
    @staticmethod
    def _stale(entry):
        return len(entry['failed_reuse_in']) >= 2 and len(entry['failed_reuse_in']) > len(entry['confirmed_in'])

    def view(self, entry, compact=False):
        item = {'id': entry['id'], 'kind': entry['kind'], 'topic': entry['topic'], 'trigger': entry['trigger'],
                'snippet': entry['snippet'], 'confirmed': len(entry['confirmed_in']),
                'failed_reuse': len(entry['failed_reuse_in']), 'stale': self._stale(entry)}
        if not compact:
            item.update(contrast=entry.get('contrast'), strategy=entry.get('strategy'), task=entry['task'])
        return item

    def catalog(self):
        """Every entry as one line, for the planner to choose from (no count limit, no similarity cut)."""
        return [{'id': entry['id'], 'line': f'{entry["kind"]} · {_one_line(entry["trigger"], 200)} · task: '
                                            f'{_one_line(entry["task"], 100)} · confirmed {len(entry["confirmed_in"])}'
                                            + (' · stale' if self._stale(entry) else '')}
                for entry in self.entries]

    def pick(self, ids):
        known = {entry['id']: entry for entry in self.entries}
        return [self.view(known[entry_id]) for entry_id in ids if entry_id in known]

    def recall(self, task, labels=(), compact=False):
        """Fallback when no planner chooses: entries most similar to ``task``, workflows then repairs."""
        def score(entry):
            overlap = len(set(labels) & set(entry.get('labels') or [])) * 0.3
            return (not self._stale(entry), similarity(entry['task'], task) + overlap, len(entry['confirmed_in']))
        workflows = sorted((e for e in self.entries if e['kind'] == 'workflow'), key=score, reverse=True)
        repairs = sorted((e for e in self.entries if e['kind'] == 'repair'), key=score, reverse=True)
        return [self.view(entry, compact) for entry in workflows[:RECALL_WORKFLOWS] + repairs[:RECALL_REPAIRS]]

    def counts(self):
        counts = {topic: 0 for topic in TOPICS}
        for entry in self.entries:
            counts[entry['topic']] = counts.get(entry['topic'], 0) + 1
        return counts


def _approach_shape(actions, tail=5):
    """Last ``tail`` tools of an attempt including failed calls (observe skipped, repeats collapsed)."""
    tools = []
    for action in actions:
        tool = action.get('tool')
        if tool and tool != 'observe' and (not tools or tools[-1] != tool):
            tools.append(tool)
    return '>'.join(tools[-tail:])


def _last_target(actions):
    """Last arm target of an attempt, rounded to 2 cm, so 'release at z=0.14' differs from 'z=0.08'."""
    for action in reversed(actions):
        xyz = (action.get('args') or {}).get('xyz_m')
        if action.get('tool') in ('move_to', 'place', 'pick') and xyz:
            return [round(round(value / 0.02) * 0.02, 2) for value in xyz]
    return None


class FailureLedger:
    STATUS_ORDER = {'eliminated': 0, 'blocked': 1, 'tried': 2, 'resolved': 3}

    def __init__(self, memory_dir, robot_id, *, frozen=False):
        self.dir = Path(memory_dir)
        self.path = self.dir / f'failure-ledger-{_safe_name(robot_id)}.json'
        self.robot_id, self.frozen = robot_id, frozen
        data = _load(self.path, {})
        self.records = list(data.get('records') or [])
        self.next_id = int(data.get('next_id') or len(self.records) + 1)

    def save(self):
        if self.frozen:
            return
        self.dir.mkdir(parents=True, exist_ok=True)
        atomic_json(self.path, {'version': 1, 'robot_id': self.robot_id, 'next_id': self.next_id,
                                'records': self.records})
        (self.dir / 'failure-ledger.md').write_text(self.render(), encoding='utf-8')

    def render(self):
        lines = ['# Failure Ledger: approaches already tried', '',
                 'Auto-recorded by the plan-act loop from failed attempts (infrastructure stops excluded).',
                 'Source of truth: the JSON file next to this one; do not edit by hand.', '']
        if not self.records:
            lines.append('_No failed attempts recorded yet._')
        for record in sorted(self.records, key=lambda r: (self.STATUS_ORDER.get(r['status'], 9), r['id'])):
            last = record['failures'][-1]
            lines += [f'## {record["id"]} · {record["status"]} · {record["approach"]}', '',
                      f'**Host evidence:** last tools `{record["tail"]}`, last arm target {record["target"]}', '',
                      f'**Task:** {record["task"]}', '',
                      f'**Failure:** {self._failure_text(record)}; failed {len(record["failures"])} time(s). '
                      f'Last reason: {last["reason"]}', '',
                      '**Executed (host-recorded):**', '', '```text', *record['snippet'], '```', '']
            review = last.get('review')
            if record.get('hypothesis'):
                lines += [f'**Why (hypothesis, {record.get("hypothesis_status", "untested")}):** '
                          f'{record.get("failure_type")} — {record["hypothesis"]}', '',
                          f'**Change to test it:** {record.get("next_try")}'
                          + (f' — expected: {record["prediction"]}' if record.get('prediction') else ''), '']
            if review and any(isinstance(review.get(aspect), dict) for aspect in ('inspection', 'execution', 'planning')):
                lines.append('**Aspects (last review):**')
                for aspect in ('inspection', 'execution', 'planning'):
                    check = review.get(aspect) or {}
                    if check:
                        lines.append(f'- {aspect}: **{check.get("verdict")}** — {check.get("finding")}'
                                     + (f' → fix: {check["fix"]}' if check.get('fix') else ''))
                lines.append('')
            for test in record.get('hypothesis_tests', []):
                lines.append(f'- Tested in {Path(test["run"]).name} attempt {test["attempt"]}: {test["result"]}'
                             + (f' — change: {test["change"]}' if test.get('change') else ''))
            if record.get('hypothesis_tests'):
                lines.append('')
            if record['status'] == 'resolved':
                lines += [f'**Resolved by:** {Path(record["resolved_by"]).name}', '']
        return '\n'.join(lines) + '\n'

    def record_attempt(self, *, task, run, attempt, actions, reason, approach=None, same_as=None, labels=(),
                       review=None):
        """Record one failed attempt; returns (record_id, status) or None when nothing was executed.

        ``approach``/``same_as``: the planner's one-line description of the failed approach and the id
        of the ledger record it repeats (None if new). Without them (e.g. a budget stop) the host
        groups by the last executed tools, last arm target and failure signature.
        ``review``: the REVIEW turn's structured result (first failed step, failure type, hypothesis of
        why, the change to test it); its approach/same_as take precedence over the action turn's.
        """
        if review:
            approach = review.get('approach') or approach
            same_as = review.get('same_as_ledger') or same_as
        if self.frozen:
            return None
        calls = [action for action in actions if action.get('tool') and action.get('tool') != 'observe']
        if not calls:
            return None
        failed = [action for action in calls if action.get('status') != 'ok']
        step = stop = None
        if failed:
            last = failed[-1]
            tool, level = last['tool'], 'skill'
            signature = failure_signature(tool, last.get('status'), last.get('error'), last.get('detail'))
            result = last.get('result') or {}
            step, monitor = result.get('failed_step'), result.get('monitor_stop')
            if isinstance(monitor, dict):  # the runtime stopped the arm in motion where the check broke
                signature = 'object_slipped'
                stop = {key: monitor.get(key) for key in ('condition', 'progress_fraction', 'elapsed_s',
                                                          'gripper_percent', 'gripper_min_percent')}
        else:
            tool, level, signature = None, 'task', 'goal_not_achieved'
        # Blocked only when the attempt ended on a call the robot could not execute.
        blocked = calls[-1].get('status') != 'ok' and signature.startswith(BLOCKING_SIGNATURES)
        tail, target = _approach_shape(calls), _last_target(calls)
        approach = ' '.join(str(approach or '').split())[:300]
        failure = {'run': run, 'attempt': attempt, 'at': _now(), 'reason': ' '.join(str(reason or '').split())[:400],
                   'approach': approach or None}
        if review:
            failure['review'] = {key: review.get(key) for key in (
                'failure_type', 'aspects_in_error', 'inspection', 'execution', 'planning', 'failed_step', 'hypothesis',
                'evidence', 'next_try', 'prediction', 'confidence', 'agrees_with_host', 'host_type_hint')}
        twin = None
        if same_as:
            twin = next((record for record in self.records if record['id'] == same_as and record['status'] != 'resolved'),
                        None)
        if twin is None and not approach:
            twin = next((record for record in self.records if record['status'] != 'resolved'
                         and record['tail'] == tail and record['target'] == target and record['signature'] == signature
                         and similarity(record['task'], task) >= SIMILAR_TASK), None)
        if twin is None:
            twin = {'id': f'F{self.next_id:03d}', 'task': task, 'labels': list(labels),
                    'approach': approach or f'{tail} (last arm target {target})', 'tail': tail, 'target': target,
                    'level': level, 'tool': tool, 'signature': signature, 'failures': [], 'status': 'tried',
                    'created_at': _now()}
            self.next_id += 1
            self.records.append(twin)
        if not any(item['run'] == run and item['attempt'] == attempt for item in twin['failures']):
            twin['failures'].append(failure)
        twin.update(snippet=[_call(action) for action in calls[-8:]], tail=tail, target=target, level=level, tool=tool,
                    signature=signature, failed_step=step, in_motion_stop=stop)
        if review:  # the latest hypothesis of why this approach fails, untested until a later plan tries it
            twin.update(failure_type=review.get('failure_type'), hypothesis=review.get('hypothesis'),
                        next_try=review.get('next_try'), prediction=review.get('prediction'),
                        aspects_in_error=review.get('aspects_in_error') or [], hypothesis_status='untested')
        twin['status'] = 'blocked' if blocked else ('eliminated' if len(twin['failures']) >= 2 else 'tried')
        return twin['id'], twin['status']

    def record_test(self, record_id, *, run, attempt, result, change=''):
        """A later plan tested this record's hypothesis: supported / not_supported / inconclusive."""
        if self.frozen:
            return False
        record = next((item for item in self.records if item['id'] == record_id), None)
        if record is None or not record.get('hypothesis'):
            return False
        tests = record.setdefault('hypothesis_tests', [])
        if not any(test['run'] == run and test['attempt'] == attempt for test in tests):
            tests.append({'run': run, 'attempt': attempt, 'result': result, 'change': str(change or '')[:400],
                          'at': _now()})
        record['hypothesis_status'] = result
        return True

    def resolve(self, task, run):
        """A verified success on a similar task: earlier failed approaches are resolved (kept for context)."""
        if self.frozen:
            return []
        resolved = []
        for record in self.records:
            if record['status'] != 'resolved' and similarity(record['task'], task) >= SIMILAR_TASK:
                record['status'], record['resolved_by'] = 'resolved', run
                resolved.append(record['id'])
        return resolved

    @staticmethod
    def _failure_text(record):
        text = f'{record["level"]}-level {record.get("tool") or "goal"} {record["signature"]}'
        if record.get('failed_step'):
            text += f' at step {record["failed_step"]!r}'
        stop = record.get('in_motion_stop')
        if stop:
            text += (f' (robot stopped the arm in motion at {stop.get("progress_fraction") or 0:.0%}: gripper '
                     f'{stop.get("gripper_percent")}% < {stop.get("gripper_min_percent")}%)')
        return text

    def view(self, record, compact=False):
        item = {'id': record['id'], 'status': record['status'], 'approach': record['approach'],
                'last_tools': record['tail'], 'last_arm_target': record['target'],
                'failure': self._failure_text(record),
                'times_failed': len(record['failures']), 'last_reason': record['failures'][-1]['reason']}
        if record.get('hypothesis'):
            types = {}
            for failure in record['failures']:
                kind = (failure.get('review') or {}).get('failure_type')
                if kind:
                    types[kind] = types.get(kind, 0) + 1
            item.update(failure_type=record.get('failure_type'), failure_types=types,
                        aspects_in_error=record.get('aspects_in_error') or [],
                        hypothesis=record['hypothesis'], next_try=record.get('next_try'),
                        hypothesis_status=record.get('hypothesis_status', 'untested'),
                        hypothesis_tests=[{key: test.get(key) for key in ('result', 'change')}
                                          for test in record.get('hypothesis_tests', [])[-3:]])
        if not compact:
            item.update(task=record['task'], snippet=record['snippet'])
        return item

    @staticmethod
    def _recall_group(record):
        """0 open hypothesis (untested/inconclusive) · 1 hypothesis tested and not supported ·
        2 eliminated/blocked without one · 3 other unresolved · 4 resolved."""
        if record['status'] == 'resolved':
            return 4
        state = record.get('hypothesis_status') if record.get('hypothesis') else None
        if state in ('untested', 'inconclusive'):
            return 0
        if state == 'not_supported':
            return 1
        return 2 if record['status'] in ('eliminated', 'blocked') else 3

    def catalog(self):
        """Every record as one line, for the planner to choose from (no count limit, no similarity cut)."""
        lines = []
        for record in self.records:
            why = record.get('hypothesis') or record['approach']
            lines.append({'id': record['id'], 'line': (
                f'{record["status"]} · task: {_one_line(record["task"], 100)} · '
                f'{record.get("failure_type") or record["signature"]} · {_one_line(why, 220)}'
                + (f' · hypothesis {record.get("hypothesis_status")}' if record.get('hypothesis') else '')
                + f' · failed {len(record["failures"])}x')})
        return lines

    def pick(self, ids):
        known = {record['id']: record for record in self.records}
        return [self.view(known[record_id]) for record_id in ids if record_id in known]

    def recall(self, task, compact=False):
        """Fallback when no planner chooses: similar-task records, open hypotheses first."""
        def last_failure(record):
            try:
                return datetime.fromisoformat(record['failures'][-1]['at']).timestamp()
            except (KeyError, IndexError, TypeError, ValueError):
                return 0.
        scored = [(record, similarity(record['task'], task)) for record in self.records]
        scored = [(record, score) for record, score in scored if score >= LEDGER_SIMILAR_TASK]
        scored.sort(key=lambda item: (self._recall_group(item[0]), -round(item[1], 2), -last_failure(item[0])))
        return [self.view(record, compact) for record, _ in scored[:RECALL_LEDGER]]

    def counts(self):
        counts = {status: 0 for status in self.STATUS_ORDER}
        for record in self.records:
            counts[record['status']] = counts.get(record['status'], 0) + 1
        return counts


def camera_key(camera_config):
    """Locate offsets belong to one camera extrinsic calibration."""
    transform = (camera_config or {}).get('T_base_camera')
    return hashlib.sha256(json.dumps(transform).encode()).hexdigest()[:12] if transform else None


RECALL_REFLECTIONS = 4
_REFLECTION_META = re.compile(r'<!-- reflection (\{.*?\}) -->\n?')


def _one_line(text, limit):
    return ' '.join(str(text or '').split())[:limit]


class ReflectionMemory:
    """Episode reflections (Reflexion-style): after a failed episode the planner writes what happened,
    why it failed, what it regrets and what to do next time. The Markdown file IS the memory: there is
    no JSON behind it. Each entry is a '## ' heading followed by a '<!-- reflection {...} -->' line; the
    text may be edited by people and is read back verbatim at PLANNING of similar tasks.
    """

    def __init__(self, memory_dir, robot_id, *, frozen=False):
        self.path = Path(memory_dir) / f'reflections-{_safe_name(robot_id)}.md'
        self.robot_id, self.frozen = robot_id, frozen

    def entries(self):
        if not self.path.exists():
            return []
        entries = []
        for block in re.split(r'(?m)^(?=## )', self.path.read_text(encoding='utf-8')):
            if not block.startswith('## '):
                continue
            meta, match = {}, _REFLECTION_META.search(block)
            if match:
                try:
                    meta = json.loads(match.group(1))
                except ValueError:
                    meta = {}
            title = block.splitlines()[0][3:].strip()
            entries.append({'id': f'R{len(entries) + 1}', 'run': meta.get('run'), 'task': meta.get('task') or title,
                            'outcome': meta.get('outcome'),
                            'at': meta.get('at', ''), 'title': title,
                            'text': _REFLECTION_META.sub('', block).strip()})
        return entries

    def catalog(self):
        """Every reflection as one line (R1 = first in the file), for the planner to choose from."""
        return [{'id': entry['id'], 'line': f'{entry["outcome"] or "note"} · {entry["at"][:10]} · task: '
                                            f'{_one_line(entry["task"], 100)} · {_one_line(entry["title"], 200)}'}
                for entry in self.entries()]

    def pick(self, ids):
        known = {entry['id']: entry for entry in self.entries()}
        return [known[entry_id] for entry_id in ids if entry_id in known]

    def recall(self, task, limit=RECALL_REFLECTIONS):
        """Fallback when no planner chooses: reflections on similar tasks, newest first."""
        similar = [entry for entry in self.entries() if similarity(entry['task'], task) >= LEDGER_SIMILAR_TASK]
        similar.sort(key=lambda entry: entry['at'], reverse=True)
        return similar[:limit]

    def append(self, *, run, task, outcome, reflection, hypotheses=()):
        """Append one episode's reflection; returns the entry title, or None when frozen."""
        if self.frozen:
            return None
        title = _one_line(reflection.get('title'), 160) or 'Episode reflection'
        meta = json.dumps({'run': str(run), 'task': _one_line(task, 400).replace('-->', '->'), 'outcome': outcome,
                           'at': _now(), 'confidence': reflection.get('confidence')}, ensure_ascii=False)
        lines = [f'## {title}', '', f'<!-- reflection {meta} -->',
                 f'*{_now()[:16].replace("T", " ")} UTC · run `{Path(str(run)).name}` · outcome **{outcome}** · '
                 f'confidence {reflection.get("confidence")}*', '',
                 f'**Task.** {_one_line(task, 400)}', '',
                 f'**What happened.** {reflection.get("what_happened", "")}', '',
                 f'**Why it failed.** {reflection.get("why_failed", "")}', '',
                 f'**Regret.** {reflection.get("regret", "")}', '']
        lessons = [lesson for lesson in reflection.get('lessons') or [] if str(lesson).strip()]
        if lessons:
            lines += ['**Next time:**', *[f'- {_one_line(lesson, 400)}' for lesson in lessons], '']
        if reflection.get('next_hypothesis'):
            lines += [f'**Next hypothesis to test.** {reflection["next_hypothesis"]}', '']
        if reflection.get('evidence'):
            lines += [f'**Evidence.** {reflection["evidence"]}', '']
        if hypotheses:
            lines += ['**Hypotheses this episode:** ' + '; '.join(
                f'{item.get("id")} {item.get("failure_type")} — {item.get("status")}: {_one_line(item.get("hypothesis"), 160)}'
                for item in hypotheses), '']
        self.path.parent.mkdir(parents=True, exist_ok=True)
        header = '' if self.path.exists() else (
            f'# Episode reflections ({self.robot_id})\n\nWritten by the robot after each failed episode and read back '
            'at PLANNING of similar tasks. You may edit the text; keep each `<!-- reflection ... -->` line.\n\n')
        with self.path.open('a', encoding='utf-8') as file:
            file.write(header + '\n'.join(lines) + '\n')
        return title


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
    """Seed drive-gain samples from an existing experience.jsonl (locate -> drive -> locate chains)."""
    embodiment = EmbodimentMemory(memory_dir, robot_id)
    trackers, counts = {}, {'drive_samples': 0}
    for line in Path(experience_path).read_text(encoding='utf-8').splitlines():
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if record.get('robot_id') != robot_id or record.get('kind') != 'action':
            continue
        run = record.get('run') or 'unknown'
        tracker = trackers.setdefault(run, MemoryTracker(embodiment, run=run))
        tool, status, result = record.get('tool'), record.get('status'), record.get('result') or {}
        args = record.get('args') or {}
        if tool == 'locate' and status == 'ok' and result.get('xyz_m'):
            found = tracker.after_locate(args.get('label'), result['xyz_m'], None)
            if (found.get('drive_measurement') or {}).get('accepted'):
                counts['drive_samples'] += 1
        elif tool == 'drive' and status == 'ok' and args.get('drive'):
            drive = args['drive']
            tracker.after_drive(drive['x'], drive['y'], drive['theta'], drive['duration_ms'])
        else:
            tracker.after_arm_skill(tool, ok=status == 'ok')
    embodiment.save()
    return counts


def import_failures(experience_path, memory_dir, robot_id):
    """Seed the failure ledger from experience.jsonl, attempt by attempt, in run order.

    An attempt failed when the action loop declared ``failed``, when VERIFY did not confirm it, or
    when the episode stopped inside it for a non-infrastructure reason. A run that ended in a
    completion claim resolves earlier records of similar tasks.
    """
    ledger = FailureLedger(memory_dir, robot_id)
    runs = {}
    for line in Path(experience_path).read_text(encoding='utf-8').splitlines():
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if record.get('robot_id') != robot_id:
            continue
        state = runs.setdefault(record.get('run') or 'unknown', {'task': record.get('task') or '', 'attempts': {},
                                                                 'pending_verify': None, 'episode': None})
        kind = record.get('kind')
        if kind in ('action', 'attempt'):
            attempt = state['attempts'].setdefault(record.get('attempt') or 1, {'actions': [], 'verdict': None,
                                                                              'reason': None, 'verify': None,
                                                                              'approach': None, 'same_as': None})
            if kind == 'action':
                attempt['actions'].append(record)
            else:
                attempt['verdict'], attempt['reason'] = record.get('verdict'), record.get('reason')
                attempt['approach'], attempt['same_as'] = record.get('failed_approach'), record.get('same_as_ledger')
                if attempt['verdict'] == 'done':
                    state['pending_verify'] = record.get('attempt')
        elif kind == 'verify' and state['pending_verify'] in state['attempts']:
            state['attempts'][state['pending_verify']]['verify'] = (record.get('verdict'), record.get('reason'))
        elif kind == 'episode':
            state['episode'] = (record.get('status'), record.get('reason'))
    counts = {'recorded': 0, 'resolved': 0}
    for run, state in runs.items():
        status, detail = state['episode'] or (None, None)
        for number, attempt in sorted(state['attempts'].items()):
            verify = attempt['verify']
            if attempt['verdict'] == 'failed':
                reason = attempt['reason'] or 'failed'
            elif attempt['verdict'] == 'done' and verify and verify[0] != 'confirm':
                reason = f'VERIFY {verify[0]}: {verify[1]}'
            elif (attempt['verdict'] is None and status and status not in INFRASTRUCTURE_STOPS
                  and status != 'completion_claim'):
                reason = detail or status
            else:
                continue
            if ledger.record_attempt(task=state['task'], run=run, attempt=number, actions=attempt['actions'],
                                     reason=reason, approach=attempt['approach'], same_as=attempt['same_as']):
                counts['recorded'] += 1
        if status == 'completion_claim':
            counts['resolved'] += len(ledger.resolve(state['task'], run))
    ledger.save()
    return counts


def main(argv=None):
    parser = argparse.ArgumentParser(description='Inspect or seed LeKiwi robot memory')
    sub = parser.add_subparsers(dest='command', required=True)
    show = sub.add_parser('show', help='Print the skill library and measured body corrections')
    show.add_argument('--memory-dir', type=Path, default=Path('runs/memory'))
    show.add_argument('--robot-id', default='kiwi-01')
    seed = sub.add_parser('import-experience', help='Seed drive samples from experience.jsonl')
    seed.add_argument('experience', type=Path)
    seed.add_argument('--memory-dir', type=Path, default=Path('runs/memory'))
    seed.add_argument('--robot-id', default='kiwi-01')
    failures = sub.add_parser('import-failures', help='Seed the failure ledger from experience.jsonl')
    failures.add_argument('experience', type=Path)
    failures.add_argument('--memory-dir', type=Path, default=Path('runs/memory'))
    failures.add_argument('--robot-id', default='kiwi-01')
    args = parser.parse_args(argv)
    if args.command == 'import-experience':
        print(json.dumps(import_experience(args.experience, args.memory_dir, args.robot_id), ensure_ascii=False))
        return 0
    if args.command == 'import-failures':
        print(json.dumps(import_failures(args.experience, args.memory_dir, args.robot_id), ensure_ascii=False))
        return 0
    library = SkillLibrary(args.memory_dir / 'skill-library', args.robot_id)
    embodiment = EmbodimentMemory(args.memory_dir, args.robot_id)
    ledger = FailureLedger(args.memory_dir, args.robot_id)
    print(json.dumps({'skill_library': library.counts(), 'entries': [library.view(e) for e in library.entries],
                      'failure_ledger': ledger.counts(), 'ledger': [ledger.view(r) for r in ledger.records],
                      'embodiment': embodiment.summary()}, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
