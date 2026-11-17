# Replace GitHub main with this checkout

Status: Published and verified. Implementation commit: `c0dcd83ccdeeb4fb48dceecd07f6fa291f6a6263`.

Goal: Publish the current standalone Bullet App code as `mayaboker/bullet_app` main while preserving the existing unrelated GitHub history on a backup branch.

Implementation: Refresh the lockfile after the sibling `beta_client` source change, run the locked tests, and commit the verified local state. Add the GitHub repository as `origin`, fetch its main branch, back up its exact tip, rename local master to main, and push with an exact `--force-with-lease`.

Interface: The active App–View MAVLink link remains one UDP conversation through View port 14550. No flight behavior or CLI changes are planned.

Scope/defaults: Preserve all existing local commits and files. Keep the old GitHub main history on a `backup/main-<old-short-sha>` branch. Stop if the remote tip changes before the guarded push.

Verification: `uv run --locked pytest -q`, lockfile portability check, clean local Git status, and remote main and backup SHA checks after pushing.

Verification results: All 6 locked tests passed. The refreshed lockfile uses the relative `../bullet_drone` source and contains no absolute local path. GitHub main was replaced with `c0dcd83ccdeeb4fb48dceecd07f6fa291f6a6263` using an exact force-with-lease; the former `343f3d194ffa651b17754553497e4511e6918d95` tip is preserved at `backup/main-343f3d1`.
