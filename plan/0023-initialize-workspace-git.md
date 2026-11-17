# 0023: Initialize workspace Git repositories

## Goal
Give each folder in `bullet_ng.code-workspace` its own local Git repository.

## Implementation steps
- Initialize `bullet_ng`, `bullet_drone`, `bullet_render`, and `bullet_app` separately with `git init`.
- Use the configured default branch; leave files unstaged and uncommitted.

## Interfaces
No application interfaces change. Git metadata is local to each folder.

## Verification
Verified that `git rev-parse --show-toplevel` returns the correct root for all four folders. Each has its own `.git` directory and an unborn `master` branch. All have zero commits, no staged files, and no remotes.

## Scope and defaults
No initial commits, remotes, or global Git configuration changes.

## Status
Implemented and verified. Implementation commit: Pending.
