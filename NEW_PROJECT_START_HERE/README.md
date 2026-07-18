# New Project Start Here

Use this folder as a reusable starter kit for any unrelated new project.

## What this gives you
- A startup prompt template for Copilot
- A project context template
- A handoff changelog template
- A session checklist template
- A runbook that tells the AI how you like projects handled
- A one-click initializer script for automatic setup
- A dedicated Templates folder that stays separate from project root docs

## How to use in a new project
1. Copy this whole folder into your new repository.
2. Double-click START_NEW_PROJECT.bat.
3. The initializer automatically creates:
	- PROJECT_CONTEXT.md
	- AI_HANDOFF_CHANGELOG.md
	- AI_HANDOFF_CHECKLIST.md
	- AI_PROJECT_RUNBOOK.md
	- COPILOT_START_PROMPT.txt
	- START_HERE.md
	- RESUME_PROJECT.bat
4. Template source files stay in NEW_PROJECT_START_HERE/Templates.
5. Keep NEW_PROJECT_START_HERE as a toolkit folder.
6. Use RESUME_PROJECT.bat in the project root for future refresh runs.
7. Start a new chat and paste COPILOT_START_PROMPT.txt.

## Optional advanced use
- Run init_new_project.ps1 with -Force to overwrite existing generated files.
- Run init_new_project.ps1 with -TargetDir to initialize a different path.

## Why this helps
This makes context persistence explicit so you do not need to repeat your process in each chat.
