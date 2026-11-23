# Rename mission runtime to app

Goal: Name the Bullet App runtime module `app.py` while keeping mission classes in the `missions` package.

Implementation steps: Move `src/bullet_app/mission.py` to `src/bullet_app/app.py`. Update the main entry point, test imports, App documentation links and diagrams, and the Bullet View implementation reference.

Interface changes: Internal Python imports use `bullet_app.app` instead of `bullet_app.mission`. The `bullet-app` console script, `run_mission` function, and runtime behavior remain unchanged. Do not retain a `mission.py` compatibility module.

Verification: The locked App suite passed (54 tests); `bullet-app --help` succeeded. No active old imports or broken links remain in the changed App docs. `git diff --check` passed in both repositories.

Scope/defaults: Change only module references and documentation, with no mission-control logic changes. Root README has no module-path reference and remains unchanged after maintain-readme review.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
