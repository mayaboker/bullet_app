# Manual VS Code app task

Status: Implemented; task JSON and command help verified. Implementation commit: Pending (no commit requested).

Goal: Make the Bullet App VS Code run task start in Manual mode.

Implementation: Keep one play-icon app task, run `uv run --locked bullet-app --mode manual`, and update its label, detail, and README instructions.

Interface: `Bullet: Run app (manual)` appears in VS Code Tasks. Auto remains available through the CLI.

Verification: Parsed task JSON, confirmed it contains one task with `--mode manual`, and ran the locked command with `--help`. No commit requested.
