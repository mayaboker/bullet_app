# VS Code auto mission task

Status: Implemented; task JSON and command help verified. Implementation commit: Pending (no commit requested).

Goal: Run the standalone `bullet-app` mission directly from the multi-root VS Code workspace after the launcher has been started manually.

Implementation: Add a play-icon VS Code task owned by `bullet_app`, with that project as its working directory and the locked uv command. Document its use in the README.

Interface: `Bullet: Run auto mission` appears in VS Code Tasks and runs `uv run --locked bullet-app`. No application API changes.

Verification: Parsed the task JSON, confirmed its locked uv command and working directory, and ran `bullet-app --help`. The separate launcher flow was verified by the previous two-process SITL flight. No commit requested.
