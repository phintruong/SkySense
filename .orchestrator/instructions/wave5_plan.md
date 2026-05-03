# Wave 5: Demos + Tests + Polish — Orchestration Plan

## Overview
Wave 4 frontend is COMPLETE. This wave finishes the project by building:
1. Failure recovery demo scripts with publication-quality matplotlib plots
2. Expanded Python unit test suite (drone model, motor model, more EKF coverage)
3. Documentation and showcase CLAUDE.md update

## Agents

| Agent | Role | Files |
|-------|------|-------|
| Agent A | Plotter library + demo runner (5 failure scenarios + plots) | `src/telemetry/plotter.py`, `demos/run_demos.py` |
| Agent B | Expanded unit test suite | `tests/test_drone_model.py`, `tests/test_motor_model.py`, expand `tests/test_ekf.py` |
| Agent C | Documentation update + frontend CLAUDE.md | `showcase/CLAUDE.md`, `AGENTS.md` |

### All three agents run in parallel — no dependencies between them.

## Current state
- 52 tests passing across: quaternion (13), rotations (12), PID (8), motor_mixer (6), EKF (8), closed_loop (3), control_config (2)
- Missing tests: `test_drone_model.py`, `test_motor_model.py`
- Missing: `demos/` directory, `src/telemetry/plotter.py`
- Frontend fully rebuilt as GNC dashboard (Wave 4 complete)
