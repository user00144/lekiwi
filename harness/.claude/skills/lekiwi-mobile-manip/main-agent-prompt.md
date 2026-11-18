# Block-by-Block Fix Loop on LeKiwi — Coordinator Guide

Read [SKILL.md](SKILL.md) first. Unlike whole-program tabletop pipelines that freeze one static
script per task, a LeKiwi mobile manipulation campaign runs an agentic block-growth loop in both
stages and **freezes the Markdown skill library** between Stage 1 and Stage 2. Every trial's
`policy.py` is disposable; the deliverable of Stage 1 is `$RUN_ROOT/lekiwi/skill-library/`.

## Campaign Shape

- **Working directory**: `REF/lekiwi/harness/` (the directory containing `pyproject.toml` for the
  `harness` package).
- **Partitions**:
  - **Stage 1 (Learning)**: `trial_01` through `trial_10`, run sequentially so each worker benefits
    from skills discovered on earlier learning trials.
  - **Stage 2 (Evaluation)**: `trial_01` through `trial_25`, each run by a **fresh** isolated worker
    against `skill-library-frozen/` with append-only block authoring.
  - **Diagnostics**: `diag_01+`, used only for hardware/comms checks and never scored.
- **Attempt budgets**: One attempt is one invocation of `harness-campaign run` on `$TRIAL_DIR/policy.py`.
  A mobile fetch-and-place task typically needs 4–6 blocks (pose/memory check, navigation or visual
  search, scene verification, local grasp + lift, optional transport/place, finish). Budget **12–15
  attempts** per trial in Stage 1 (give `trial_01` 50% extra as the bootstrap trial) and **10–12
  attempts** in Stage 2.
- **Serial robot execution**: Only one process may drive a given `robot_id` (`kiwi-01`) at a time.
  On physical hardware (`--mode live`), verify the operator reset the scene between attempts when
  an earlier attempt displaced objects or ended in `uncertain_execution`.

## Campaign Initialization

```bash
# Run from REF/lekiwi/harness/
LIB="$RUN_ROOT/lekiwi/skill-library"
CAMP="$RUN_ROOT/lekiwi/$TASK"
mkdir -p "$CAMP/stage1" "$CAMP/stage2"
: > "$CAMP/campaign-state.md"

# First task of a campaign ONLY: seed headings-only Markdown files (empty bodies)
if [ -z "$(ls -A "$LIB" 2>/dev/null)" ]; then
  uv run harness-campaign init-library --dest "$LIB"
fi
```

Never copy prose or code from `.claude/skills/lekiwi-mobile-manip/skills/` into `$LIB`. Starting
with headings-only files guarantees that every workflow and recovery rule in `$LIB` comes from
measured campaign evidence.

## Stage 1: Continual Skill & Episodic Acquisition (`trial_01` .. `trial_10`)

For each learning trial in order:

1. Dispatch one worker using [subagent-prompt.md](subagent-prompt.md) with `STAGE: 1` for that
   single trial.
2. When the worker finishes, inspect `$TRIAL_DIR/trial-summary.md`, its `Non-privileged API audit`
   status, and any candidate entries added to `$LIB/*.md`.
3. Curate and deduplicate `$LIB/*.md`:
   - Route coarse task stage decompositions (*"When task requires X, sequence blocks A -> B -> C"*)
     to `episodic-workflows.md`.
   - Route contrastive operational repairs (*what failed with measured error/observation vs. what
     worked*) to `search-and-memory.md`, `navigation.md`, `mobile-grasp.md`, `time-budget.md`, or
     `lekiwi-operations.md`.
   - Enforce the entry contract: **Trigger**, **Contrast (what failed & why)**, **Working Python
     snippet**, and **Provenance** (`task`, `trial`, `attempt`, `date`). Delete prose notes that
     lack a runnable snippet or provenance.
   - Tag cross-task transferability explicitly: harness and embodiment facts (e.g., `EpisodeRunner`
     two-observation completion rule, 5-DoF `arm_base` IK constraints, `relocalize()` confidence
     gate) transfer across tasks; object-specific grasp offsets or text queries are **hypotheses**
     until confirmed on the new task.
4. Optional verification gate: if a worker proposes a subtle or high-impact skill, dispatch a blind
   verifier with [verifier-prompt.md](verifier-prompt.md) in `$RUN_ROOT/lekiwi/verification/` before
   locking it into `$LIB`.
5. Append one ledger row to `$CAMP/campaign-state.md`: trial ID, attempts used, `task_success`,
   `navigation_reached`, infrastructure losses (if any), and skill headings added/updated.

## Freeze Gate (Before Any Stage 2 Trial)

```bash
cp -r "$RUN_ROOT/lekiwi/skill-library" "$CAMP/skill-library-frozen"
chmod -R a-w "$CAMP/skill-library-frozen"
( cd "$CAMP" && find skill-library-frozen -type f -print0 | sort -z \
    | xargs -0 shasum -a 256 > frozen-manifest.sha256 )
```

*(On GNU coreutils systems `sha256sum` and `shasum -a 256` produce identical formats; verify with
`shasum -a 256 -c frozen-manifest.sha256` on macOS or `sha256sum -c` on Linux.)*

Once frozen, `skill-library-frozen/`, the prompt templates, the model, and `kinematics_config`
must not change for the remainder of `$TASK`'s Stage 2. Verify the manifest before every Stage 2
worker dispatch:

```bash
( cd "$CAMP" && shasum -a 256 -c frozen-manifest.sha256 ) || echo "FROZEN LIBRARY CHANGED - STOP"
```

## Stage 2: Held-Out Evaluation (`trial_01` .. `trial_25`)

- Dispatch one **fresh** subagent per evaluation trial with `STAGE: 2`.
- Each worker receives an empty `$TRIAL_DIR/policy.py` and read-only access to
  `$CAMP/skill-library-frozen/`.
- **Strict isolation**: Never reuse a worker conversation across Stage 2 trials, never pass another
  trial's `policy.py` or `trial-summary.md`, and do not inspect Stage 2 failures mid-run to patch
  the library.

## Scoring: Exactly Two Numbers on the Final Policy

Run the standalone campaign scorer from `.claude/skills/lekiwi-mobile-manip/`:

```bash
./.claude/skills/lekiwi-mobile-manip/score_campaign.py "$RUN_ROOT/lekiwi" --stage stage2 --per-trial
```

- **Unit of scoring**: The **FINAL POLICY** of each trial, identified exactly by hashing each
  attempt's `source/policy.py` (`sha256[:12]`). Early block-growth attempts lacked later
  manipulation blocks by construction and are excluded from scoring.
- **Two reported metrics (both per trial)**:
  1. **SUCCESS**: Fraction of trials whose final policy achieved `outcome.json`
     `task_success: true` at least once.
  2. **NAVIGATION**: Fraction of trials whose final policy delivered the robot into working range
     and engaged the arm and gripper on the target (`navigation_reached: true`) at least once.
- Do **not** report per-attempt ratios, per-leg `navigate_to` rates, or repeat reliability numbers
  alongside these two headline metrics.
- **Separate infrastructure and safety halts from policy failures**: Attempts without `outcome.json`
  (daemon crash before start) or halted by transport loss / hardware fault (`uncertain_execution`
  from ZMQ disconnect or uncalibrated joint encoder drift) must be named explicitly in the report
  with trial ID and cause rather than silently dropped or miscounted as policy logic errors.

## Stop and Escalate to the Operator If

- Any live attempt halts with `termination_reason: "uncertain_execution"` — physical robot state is
  ambiguous and **must not** be automatically retried until hardware clearance is confirmed.
- `shasum -a 256 -c frozen-manifest.sha256` fails during Stage 2.
- A live joint reports `arm_current_out_of_calibration` on every joint needed to stow or grasp.
- Two processes attempt to control the same `robot_id` simultaneously.

