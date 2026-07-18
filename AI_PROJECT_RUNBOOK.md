# AI Project Runbook

This file defines the preferred workflow for this repository.

## Core workflow preferences
- Keep context files updated continuously, not just at the end.
- Prefer practical implementation over long planning.
- Validate changes after edits (syntax/lint/tests when available).
- Keep setup artifacts organized in dedicated folders.
- Create setup zips when handoff or deployment needs portability.

## Required context files
- `PROJECT_CONTEXT.md`: current architecture, active decisions, known constraints.
- `AI_HANDOFF_CHANGELOG.md`: chronological change log and what changed.
- `AI_HANDOFF_CHECKLIST.md`: actionable verification checklist.

## Update policy for AI sessions
- On session start: read all required context files.
- During work: append meaningful deltas to changelog.
- After major changes: update checklist and context summary.
- Before completion: state what is validated vs not validated.

## Implementation style
- Make smallest safe change that solves the problem.
- Preserve existing behavior unless explicitly changing behavior.
- Add concise comments only where logic is non-obvious.
- Keep scripts production-friendly (clear errors, exit codes, logs).

## Packaging and setup
- Keep setup instructions in one obvious location.
- Keep companion scripts with their owning subsystem.
- Prefer one-command setup/build scripts for repeatability.

## Communication style
- Be direct and practical.
- Report exactly what changed, where, and what was validated.
- Distinguish repo-complete work from hardware/runtime-unverified work.
