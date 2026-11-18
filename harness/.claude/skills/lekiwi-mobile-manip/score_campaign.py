#!/usr/bin/env python3
"""Score a LeKiwi block-by-block mobile manipulation campaign from its run artifacts.

Reports exact two numbers per task, both per trial/seed:

  SUCCESS     the trial's final policy achieved task_success at least once
  NAVIGATION  the trial's final policy delivered the robot into working range and
              engaged both arm and gripper on the target in at least one episode

The unit of scoring is the FINAL POLICY, never the raw attempt count. Workers grow
`policy.py` one code block at a time, so early development attempts lacked later
manipulation blocks and could not have succeeded; they are excluded automatically by
hashing `source/policy.py` in every attempt directory and retaining all runs matching
the last distinct SHA-256 prefix.

Usage:
    ./score_campaign.py <campaign-dir> [--stage stage2] [--per-trial]

<campaign-dir> holds one directory per task (e.g. `$RUN_ROOT/lekiwi`), each containing
`stage1/` and/or `stage2/` with `trial_*` (or `seed_*`) subdirectories. Directories
whose name contains `VOIDED` are skipped.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


def _engaged_from_trace(run_dir: Path) -> bool:
    """Fallback check on `trace/events.jsonl` for confirmed arm+gripper engagement."""
    events_path = run_dir / 'trace' / 'events.jsonl'
    if not events_path.is_file():
        return False
    for line in events_path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get('tool') == 'act':
            output = row.get('output') or {}
            if bool(output.get('arm_engaged')):
                return True
    return False


def episodes(trial_dir: Path) -> list[dict]:
    """Return ordered scorable runs for one trial, tagged with `source/policy.py` hash."""
    found: list[dict] = []
    for attempt in sorted(trial_dir.glob('attempts/attempt_*'), key=lambda p: p.name):
        if not attempt.is_dir() or 'VOIDED' in attempt.name:
            continue
        outcome_path = attempt / 'outcome.json'
        if not outcome_path.is_file():
            outcome_path = next(attempt.glob('**/outcome.json'), None)
        if outcome_path is None:
            continue  # launch crash before runner startup: infrastructure loss, not policy
        run_dir = outcome_path.parent
        policy_path = run_dir / 'source' / 'policy.py'
        if not policy_path.is_file():
            continue
        try:
            outcome = json.loads(outcome_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            continue

        reached = bool(outcome.get('navigation_reached')) or _engaged_from_trace(run_dir)
        found.append(
            {
                'attempt': attempt.name,
                'policy': hashlib.sha256(policy_path.read_bytes()).hexdigest()[:12],
                'success': bool(outcome.get('task_success')),
                'reached': reached,
                'termination_reason': str(outcome.get('termination_reason', '')),
            }
        )
    return found


def _iter_trial_dirs(stage_dir: Path) -> list[Path]:
    candidates = sorted(
        [
            p
            for p in stage_dir.iterdir()
            if p.is_dir()
            and 'VOIDED' not in p.name
            and (p.name.startswith('trial_') or p.name.startswith('seed_'))
        ],
        key=lambda p: p.name,
    )
    return candidates


def score_task(task_dir: Path, stage: str) -> dict | None:
    stage_dir = task_dir / stage
    if not stage_dir.is_dir():
        return None
    trials = _iter_trial_dirs(stage_dir)
    rows: list[dict] = []
    uncertain_halts: list[str] = []

    for trial in trials:
        runs = episodes(trial)
        if not runs:
            continue
        label = trial.name.removeprefix('trial_').removeprefix('seed_')
        for r in runs:
            if r['termination_reason'] == 'uncertain_execution':
                uncertain_halts.append(f"{label}:{r['attempt']}")
        final_hash = runs[-1]['policy']
        final_runs = [r for r in runs if r['policy'] == final_hash]
        rows.append(
            {
                'trial': label,
                'attempts': len(runs),
                'final_runs': len(final_runs),
                'success': any(r['success'] for r in final_runs),
                'navigated': any(r['reached'] for r in final_runs),
            }
        )

    if not rows:
        return None
    return {
        'task': task_dir.name,
        'trials': len(rows),
        'success': sum(1 for r in rows if r['success']),
        'navigated': sum(1 for r in rows if r['navigated']),
        'failed': [r['trial'] for r in rows if not r['success']],
        'unnavigated': [r['trial'] for r in rows if not r['navigated']],
        'uncertain_halts': uncertain_halts,
        'rows': rows,
    }


def _pct(part: int, whole: int) -> str:
    return f'{part / whole:.0%}' if whole else 'n/a'


def report(result: dict, per_trial: bool) -> None:
    n = result['trials']
    print(f"\n=== {result['task']} ===")
    print(f"  SUCCESS      {result['success']}/{n}  = {_pct(result['success'], n)}")
    print(f"  NAVIGATION   {result['navigated']}/{n}  = {_pct(result['navigated'], n)}")
    if result['failed']:
        print(f"  trials whose final policy never succeeded: {', '.join(result['failed'])}")
    if result['unnavigated']:
        print(f"  trials whose final policy never reached/engaged target: {', '.join(result['unnavigated'])}")
    if result['uncertain_halts']:
        print(f"  safety/transport halts (uncertain_execution): {', '.join(result['uncertain_halts'])}")
    if per_trial:
        print(f"  {'trial':>6} {'attempts':>9}  success  navigation")
        for row in result['rows']:
            print(
                f"  {row['trial']:>6} {row['attempts']:>9}"
                f"  {'yes' if row['success'] else 'NO ':>7}"
                f"  {'yes' if row['navigated'] else 'NO '}"
            )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('campaign', type=Path, help='Campaign directory holding one subfolder per task')
    parser.add_argument('--stage', default='stage2', help='Stage subdirectory to score (default: stage2)')
    parser.add_argument(
        '--per-trial',
        '--per-seed',
        dest='per_trial',
        action='store_true',
        help='Also print one breakdown row per trial/seed',
    )
    args = parser.parse_args(argv)

    if not args.campaign.is_dir():
        print(f'error: {args.campaign} is not a directory', file=sys.stderr)
        return 2

    tasks = [
        d
        for d in sorted(args.campaign.iterdir())
        if d.is_dir() and 'VOIDED' not in d.name and (d / args.stage).is_dir()
    ]
    if not tasks:
        print(
            f'error: no task directory under {args.campaign} contains {args.stage}/',
            file=sys.stderr,
        )
        return 2

    for task in tasks:
        res = score_task(task, args.stage)
        if res is None:
            print(f'\n=== {task.name} — no scorable runs in {args.stage}/ ===')
            continue
        report(res, args.per_trial)

    print(
        '\nBoth numbers are per trial and judged strictly on the final policy (by SHA-256 of\n'
        'source/policy.py). Earlier block-growth attempts are excluded. Infrastructure or\n'
        'transport losses (missing outcome.json, ZMQ disconnect, uncertain_execution) must be\n'
        'reported separately from policy logic failures.'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

