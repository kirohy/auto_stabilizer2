---
name: wbms-plan-work-unit
description: Read-only planning for one WBMS external whole-body teleoperation Work Unit. Use when a Work Unit needs branch archaeology, current-code inspection, interface design, safety-invariant analysis, acceptance criteria, and a frozen Work Unit Contract before implementation. Do not use to implement source changes, review a completed diff, or commit.
---

# WBMS Work Unit planning

## Purpose

One invocation plans exactly one Work Unit for the external whole-body teleoperation project.

The result is a reviewable `Work Unit Contract`. Source code must not be modified in this Skill.

## Required input

Require or resolve the following before producing a Contract.

- Work Unit ID and title.
- Target repository or repositories.
- Target branch and current HEAD for every repository.
- Authoritative implementation plan.
- Latest Progress entry.
- Intended outcome and known constraints.

If the Work Unit identity is ambiguous, stop after reporting the ambiguity. Do not invent a Work Unit or silently widen scope.

## Mandatory reading order

For work under `auto_stabilizer/`, read in this order.

1. Repository-root `AGENTS.md`.
2. `auto_stabilizer/AGENTS.md`.
3. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md`.
4. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md`.
5. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`.
6. The current Work Unit Contract, if one exists.
7. `auto_stabilizer/docs/WBMSWalkingPreparationDesignRevisionPlan.md` when walking preparation, COM height, READY, or walking API ownership is relevant.
8. Older documents only for history and rejected alternatives.

For another repository, read its nearest `AGENTS.md` and the same authoritative plan and Progress documents before inspecting code.

## Repository-state check

For every repository, record the exact output or equivalent facts for:

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

Also record dependency branch and SHA for `auto_stabilizer2`, `whole_body_teleop`, `rtmros_msg_bridge`, `ik_solvers2`, and `prioritized_qp` when relevant.

Do not reset, stash, checkout, clean, delete, or overwrite user changes.

## Read-only investigation

Inspect enough current code and history to establish:

1. The actual call order and ownership boundaries.
2. Relevant classes, functions, ports, messages, IDL, and parameters.
3. Differences between the authoritative plan and current code.
4. Safety invariants that must remain true.
5. Failure paths, stale paths, invalid-data paths, and mode transitions.
6. Cross-repository schema and compatibility impact.
7. Exact files that may be changed and files that must not be changed.
8. Build, static checks, simulation, log, and real-robot checks required for acceptance.
9. Items that cannot be verified in the current environment.

For 500 Hz work, explicitly inspect whether the proposed change could introduce blocking I/O, ROS callbacks, network waits, mutex waits, condition-variable waits, per-cycle thread creation, unbounded queues, unnecessary cloning, full searches, or unbounded allocation.

For control changes, trace joint limits, self-collision, feet/contact, COM/ZMP/refdz/omega/l, walking preparation ownership, hidden-goal behavior, and final validation.

## Contract rules

- Keep one primary responsibility per Work Unit.
- Split protocol, producer, bridge, consumer, safety behavior, diagnostics, and cleanup when they can be reviewed independently.
- Do not combine unrelated repositories in one commit. Related repository changes may share one Work Unit ID with sub-unit suffixes.
- Do not choose an implementation merely because it is described in an older document.
- Do not mark simulation or real-robot behavior as verified from source inspection alone.
- If the plan must change, identify the conflict and return to design rather than silently changing scope.

## Required output

Return exactly one Contract in this structure.

```markdown
# Work Unit Contract: <ID> <title>

## Goal

## Scope
- repository
- branch
- base SHA
- files allowed

## Out of scope

## Current behavior

## Required behavior

## Safety invariants

## Interface/schema impact

## Implementation steps

## Acceptance criteria

## Verification commands

## Review focus

## Known unverified items

## Dependency compatibility

## Open decisions blocking implementation
```

The Contract must contain concrete, testable acceptance criteria and exact commands where known.

## Completion condition

Planning is complete only when:

- branch and base SHA are explicit;
- scope and out-of-scope are explicit;
- safety invariants and failure behavior are explicit;
- interface changes are mapped producer-to-consumer;
- verification and unverified items are separated; and
- no implementation-blocking decision remains.

Do not implement, stage, commit, push, merge, or open a pull request while using this Skill.
