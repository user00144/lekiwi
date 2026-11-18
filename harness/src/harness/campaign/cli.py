"""CLI entrypoint (`harness-campaign`) for LeKiwi mobile manipulation campaigns."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Sequence

from .runner import (
    DEFAULT_MAX_LOOP_TICKS,
    CampaignPolicyRunner,
    LiveInnerActionExecutor,
    LiveLocalizationProvider,
    LiveNavigationExecutor,
    LiveSkillExecutor,
    SimulatedCampaignWorld,
)
from .semantic_memory import SemanticMemoryStore
from .topology import TopologicalMap


def default_skill_templates_dir() -> Path:
    """Locate `harness/.claude/skills/lekiwi-mobile-manip/skills/` from package tree."""
    # cli.py lives at harness/src/harness/campaign/cli.py -> parents[3] is harness/
    harness_root = Path(__file__).resolve().parents[3]
    return harness_root / '.claude' / 'skills' / 'lekiwi-mobile-manip' / 'skills'


def extract_headings_only(markdown_text: str) -> str:
    """Extract only `#` and `##` Markdown headings outside fenced code blocks."""
    in_fence = False
    headings: list[str] = []
    for raw_line in markdown_text.splitlines():
        stripped = raw_line.strip()
        if stripped.startswith('```'):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        if stripped.startswith('# ') or stripped.startswith('## '):
            headings.append(stripped)
    if not headings:
        return ''
    return '\n\n'.join(headings) + '\n'


def init_skill_library(
    dest_dir: str | Path,
    *,
    templates_dir: str | Path | None = None,
    force: bool = False,
) -> list[Path]:
    """Seed a campaign's `skill-library/` with empty-body topic Markdown files."""
    src = Path(templates_dir) if templates_dir is not None else default_skill_templates_dir()
    if not src.is_dir():
        raise FileNotFoundError(f'Skill templates directory not found: {src}')

    dest = Path(dest_dir)
    dest.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for template_path in sorted(src.glob('*.md')):
        if template_path.name.lower() == 'readme.md':
            continue
        target_path = dest / template_path.name
        if target_path.exists() and not force:
            continue
        skeleton = extract_headings_only(template_path.read_text(encoding='utf-8'))
        target_path.write_text(skeleton, encoding='utf-8')
        written.append(target_path)
    return written


def build_default_sim_topology() -> TopologicalMap:
    topo = TopologicalMap()
    topo.add_node('1f_lobby', floor='1F', x_m=0.5, y_m=0.5, yaw_deg=0.0, aliases=('lobby', 'start'))
    topo.add_node('1f_ramp_bottom', floor='1F', x_m=3.0, y_m=0.5, yaw_deg=0.0, kind='ramp', aliases=('ramp_bottom',))
    topo.add_node('2f_ramp_top', floor='2F', x_m=0.5, y_m=0.5, yaw_deg=0.0, kind='ramp', aliases=('ramp_top',))
    topo.add_node('2f_pantry', floor='2F', x_m=4.0, y_m=1.5, yaw_deg=0.0, aliases=('pantry', 'kitchen'))
    topo.add_edge('1f_lobby', '1f_ramp_bottom', via='drive')
    topo.add_edge('1f_ramp_bottom', '2f_ramp_top', via='ramp', cost_m=6.0)
    topo.add_edge('2f_ramp_top', '2f_pantry', via='drive')
    return topo


def resolve_act_loop(act_loop: str, skills_config) -> str:
    """`auto` uses the plan-act loop (calibrated locate/pick skills) whenever skills are configured."""
    if act_loop == 'auto':
        return 'plan-act' if skills_config else 'step'
    return act_loop


def plan_act_episode_fn(skills_config, camera_config, experience_path=None):
    """`act()` episodes through Planning -> big-skill Action Loop -> Verify (`plan_loop`).

    Without `experience_path` each stage keeps its lessons next to the stage directories of the
    same attempt, so no experience leaks across trials (Stage 2 isolation).
    """
    from functools import partial

    from ..plan_loop import run_plan_episode

    return partial(run_plan_episode, skills_config=skills_config, camera_config=camera_config,
                   experience_path=experience_path)


def _cmd_run(args: argparse.Namespace) -> int:
    store = SemanticMemoryStore(persist_dir=args.semantic_db)
    if args.topology:
        topo = TopologicalMap.load(args.topology)
    elif args.mode == 'sim':
        topo = build_default_sim_topology()
    else:
        topo = TopologicalMap()

    if args.mode == 'sim':
        world = SimulatedCampaignWorld.default_two_floor_world()
        if store.count() == 0 and args.seed_sim_memory:
            store.upsert(
                'red ceramic mug on pantry counter',
                floor='2F',
                x_m=4.0,
                y_m=1.5,
                yaw_deg=0.0,
                room='pantry',
                tags=('mug', 'ceramic', 'red'),
                robot_id=args.robot_id,
            )
        runner = CampaignPolicyRunner(
            task_language=args.task,
            robot_id=args.robot_id,
            mode='sim',
            semantic_memory=store,
            topology=topo,
            localization=world,
            navigation_executor=world,
            action_executor=world,
            skill_executor=world,
            max_loop_ticks=args.max_loop_ticks,
        )
    else:
        act_loop = resolve_act_loop(args.act_loop, args.skills_config)
        if act_loop == 'plan-act' and not (args.kinematics_config and args.skills_config):
            raise SystemExit('--act-loop plan-act needs --kinematics-config and --skills-config')
        localization = LiveLocalizationProvider(pose_file=args.pose_file)
        nav_exec = LiveNavigationExecutor(
            server=args.server,
            model=args.model,
            localization=localization,
            kinematics_config=args.kinematics_config,
        )
        act_exec = LiveInnerActionExecutor(
            server=args.server,
            model=args.model,
            kinematics_config=args.kinematics_config,
            act_episode_fn=(plan_act_episode_fn(args.skills_config, args.camera_config)
                            if act_loop == 'plan-act' else None),
        )
        skill_exec = None
        if args.kinematics_config and args.skills_config:
            skill_exec = LiveSkillExecutor(
                server=args.server,
                model=args.model,
                kinematics_config=args.kinematics_config,
                skills_config=args.skills_config,
                camera_config=args.camera_config,
            )
        runner = CampaignPolicyRunner(
            task_language=args.task,
            robot_id=args.robot_id,
            mode='live',
            semantic_memory=store,
            topology=topo,
            localization=localization,
            navigation_executor=nav_exec,
            action_executor=act_exec,
            skill_executor=skill_exec,
            max_loop_ticks=args.max_loop_ticks,
        )

    outcome = runner.run_policy_file(args.policy, args.output_dir)
    print(json.dumps(outcome.to_dict(), ensure_ascii=False, indent=2))
    return 0 if outcome.program_ok else 1


def _cmd_init_library(args: argparse.Namespace) -> int:
    written = init_skill_library(args.dest, templates_dir=args.templates_dir, force=args.force)
    print(json.dumps({'dest': str(Path(args.dest).resolve()), 'files_written': [p.name for p in written]}, indent=2))
    return 0


def _cmd_memory_upsert(args: argparse.Namespace) -> int:
    store = SemanticMemoryStore(persist_dir=args.semantic_db)
    tags = [t.strip() for t in args.tags.split(',') if t.strip()] if args.tags else []
    hit = store.upsert(
        args.text,
        floor=args.floor,
        x_m=args.x_m,
        y_m=args.y_m,
        yaw_deg=args.yaw_deg,
        room=args.room,
        tags=tags,
        robot_id=args.robot_id,
    )
    print(json.dumps(hit.to_dict(), ensure_ascii=False, indent=2))
    return 0


def _cmd_memory_search(args: argparse.Namespace) -> int:
    store = SemanticMemoryStore(persist_dir=args.semantic_db)
    hits = store.search(
        args.query,
        floor=args.floor,
        top_k=args.top_k,
        include_absent=args.include_absent,
    )
    print(json.dumps([h.to_dict() for h in hits], ensure_ascii=False, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='harness-campaign',
        description='ASPIRE-style block-by-block mobile manipulation campaign runner for LeKiwi.',
    )
    sub = parser.add_subparsers(dest='command', required=True)

    run_p = sub.add_parser('run', help='Execute a policy.py attempt and write structured artifacts.')
    run_p.add_argument('--policy', type=Path, required=True, help='Path to policy.py')
    run_p.add_argument('--output-dir', type=Path, required=True, help='Attempt artifact output directory')
    run_p.add_argument('--task', default='Complete mobile manipulation goal', help='Task instruction language')
    run_p.add_argument('--robot-id', default='kiwi-01', help='Robot identifier')
    run_p.add_argument('--mode', choices=('live', 'sim'), default='live', help='Execution backend mode')
    run_p.add_argument('--server', default='tcp://127.0.0.1:5556', help='ZeroMQ router address for live mode')
    run_p.add_argument('--model', default=None, help='Planner model for inner stages (default: Codex default model)')
    run_p.add_argument('--kinematics-config', type=Path, default=None, help='Optional SO-101 kinematics JSON config')
    run_p.add_argument('--skills-config', type=Path, default=None,
                       help='Host skill settings (home/grasp/gripper); enables pick/place/home_arm/drive')
    run_p.add_argument('--camera-config', type=Path, default=None,
                       help='Calibrated front camera; enables locate() (bbox -> arm_base XYZ)')
    run_p.add_argument('--act-loop', choices=('auto', 'step', 'plan-act'), default='auto',
                       help='Inner loop for act(): auto = plan-act when --skills-config is given, else step')
    run_p.add_argument('--pose-file', type=Path, default=None, help='Optional live SLAM pose JSON file')
    run_p.add_argument('--semantic-db', type=Path, default=None, help='ChromaDB persistence directory')
    run_p.add_argument('--topology', type=Path, default=None, help='TopologicalMap JSON path')
    run_p.add_argument('--seed-sim-memory', action='store_true', help='Seed demo ChromaDB entry in sim mode')
    run_p.add_argument('--max-loop-ticks', type=int, default=DEFAULT_MAX_LOOP_TICKS, help='Max loop iterations')
    run_p.set_defaults(func=_cmd_run)

    init_p = sub.add_parser('init-library', help='Seed an empty headings-only skill-library/ directory.')
    init_p.add_argument('--dest', type=Path, required=True, help='Destination skill-library directory')
    init_p.add_argument('--templates-dir', type=Path, default=None, help='Override reference skills/ directory')
    init_p.add_argument('--force', action='store_true', help='Overwrite existing topic files')
    init_p.set_defaults(func=_cmd_init_library)

    up_p = sub.add_parser('memory-upsert', help='Insert or update a spatial-semantic object record in ChromaDB.')
    up_p.add_argument('--semantic-db', type=Path, required=True, help='ChromaDB persistence directory')
    up_p.add_argument('--text', required=True, help='Scene or object description')
    up_p.add_argument('--floor', required=True, help='Floor identifier, e.g. 1F or 2F')
    up_p.add_argument('--x-m', type=float, required=True, help='X coordinate in metres')
    up_p.add_argument('--y-m', type=float, required=True, help='Y coordinate in metres')
    up_p.add_argument('--yaw-deg', type=float, default=0.0, help='Heading in degrees')
    up_p.add_argument('--room', default='', help='Optional room label')
    up_p.add_argument('--tags', default='', help='Comma-separated tags')
    up_p.add_argument('--robot-id', default='kiwi-01', help='Robot identifier')
    up_p.set_defaults(func=_cmd_memory_upsert)

    search_p = sub.add_parser('memory-search', help='Query ChromaDB spatial-semantic memory.')
    search_p.add_argument('--semantic-db', type=Path, required=True, help='ChromaDB persistence directory')
    search_p.add_argument('--query', required=True, help='Natural-language search query')
    search_p.add_argument('--floor', default=None, help='Optional floor filter')
    search_p.add_argument('--top-k', type=int, default=3, help='Max hits to return')
    search_p.add_argument('--include-absent', action='store_true', help='Include records marked absent')
    search_p.set_defaults(func=_cmd_memory_search)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))

