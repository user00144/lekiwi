# Clean Task Slate: LeKiwi Mobile Manipulation

Follow this checklist before rerunning a trial, a stage, or an entire campaign.

1. **Choose the exact archive scope (never `rm` evidence)**:
   - Single trial: `$RUN_ROOT/lekiwi/<task>/stage<N>/trial_<NN>/`. Move it aside into
     `$RUN_ROOT/lekiwi/<task>/archive/VOIDED_trial_<NN>_<timestamp>/` (directories with `VOIDED` in
     their name are skipped automatically by `score_campaign.py` if left in place, or keep archives
     under `archive/`).
   - Never delete individual `attempts/attempt_NNN/` directories inside an active trial; failed
     attempts are part of the empirical record.
2. **Stage 1 skill rollback**:
   If rerunning a Stage 1 trial, inspect `skill-library/*.md` for entries whose provenance cites
   **only** that voided trial and remove or revert them before restarting the trial.
3. **Stage 2 rerun prohibition**:
   **Never rerun a Stage 2 evaluation trial to improve its score.** A Stage 2 trial may be voided
   and rerun only when invalidated by external infrastructure failure (e.g., power loss on Jetson
   Orin, dead USB camera stream at startup, or a human stepping into the workspace), and the exact
   reason must be logged in `campaign-state.md`.
4. **Check lingering processes**:
   Verify no `harness-campaign run` or `harness-run` process is still connected to the target
   `robot_id`:
   ```bash
   pgrep -af 'harness-campaign|harness-run' | grep -v pgrep
   ```
5. **Hardware & transport health (`--mode live` only)**:
   - Confirm `server` router (`:5555` agent / `:5556` client) and `lekiwi_runtime` on Jetson Orin
     are connected with `guarded_execution: 1`.
   - If the previous run ended with `termination_reason: "uncertain_execution"` or
     `arm_current_out_of_calibration`, inspect the physical SO-101 arm posture and workspace
     clearance before issuing any new command.
   - Reset displaced target objects to the trial's initial physical layout.
6. **Re-verify the Stage 2 freeze manifest** (if Stage 2 has begun):
   ```bash
   ( cd "$RUN_ROOT/lekiwi/<task>" && shasum -a 256 -c frozen-manifest.sha256 )
   ```
7. **Workspace isolation**:
   Workers write only inside their assigned `$TRIAL_DIR` (plus `$RUN_ROOT/lekiwi/skill-library/`
   when `STAGE=1`). Never edit `.claude/skills/lekiwi-mobile-manip/` during an active campaign.

