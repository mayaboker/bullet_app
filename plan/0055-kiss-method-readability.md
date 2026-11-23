# KISS method readability guidance

Goal: Guide future refactors of long orchestration methods into readable, logical steps without unnecessary abstraction.

Implementation steps: Add a repository-wide method-readability rule to `AGENTS.md`, using control loops such as `run_mission` as an example. Name coherent responsibilities for extraction, keep safety and cleanup order visible, use existing context objects, and discourage trivial wrappers and arbitrary line limits.

Interface changes: None; agent guidance only.

Verification: Reviewed the new rule beside the SOLID guidance for consistency; `git diff --check` passed. No application behavior changed, so no runtime tests were needed.

Scope/defaults: Do not refactor application code as part of this guidance change. Do not mandate a fixed method length or number of helpers.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
