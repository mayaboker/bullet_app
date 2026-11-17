# VS Code mission choice

Status: Pending
Implementation commit: Pending

Goal: Let the Bullet App run task select the automatic mission at launch.

Implementation: Replace the fixed target-hit run task with a VS Code pickString input that supplies `--mission hover-land` or `--mission target-hit`. Keep the separate manual task.

Interface: The automatic task prompts for a mission and defaults to hover-land. Its command passes the selected value to the existing CLI.

Verification: Parse tasks.json and check both choices against the CLI; inspect task and documentation diffs.

Verification outcome: Task JSON parsed successfully, both values were accepted by CliParser, and `git diff --check` passed.

Scope/defaults: No application CLI or flight logic change.
