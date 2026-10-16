# Firmware project context and update recording

## Read at the start of relevant work

- `CLAUDE.md` for the existing development workflow and hardware constraints.
- `docs/Firmware-Development-Handoff.md` for current implementation, decisions, measured values, and unresolved checks.
- The relevant stage and latest Bring-up log entries in `docs/BLDC Controller Firmware Bring-Up Test Plan.md`.
- Current source before claiming that planned functionality is implemented.

## Standing user preference: preserve bring-up updates

Developer for this chat's requests, decisions, and setup reports: **Nathaniel Su** (explicit attribution request, 2026-10-15). Name Nathaniel Su when recording his updates. Apply this attribution to this agent's session contributions, not retroactively to earlier contributors or historical bench reports of unspecified authorship. Distinguish the developer/requester from agent-written documentation and distinguish reported evidence from independently verified results.

When the user reports firmware changes, bench results, measurements, failures, stage decisions, or next steps, record substantive updates during that turn without requiring another request to save them:

1. Append a dated entry to the existing **Bring-up log** in `docs/BLDC Controller Firmware Bring-Up Test Plan.md`. This is the canonical chronological record; do not create a competing results log.
2. Include stage, board/motor configuration if known, test/firmware settings, reported measurements with units, evidence source, outcome, decisions, and remaining checks. Label missing information as not provided; never fill it with assumptions.
3. Distinguish user-reported observations, inspected logs/captures, code inspection, hypotheses, and proposed tests. A report that a motor spins is not proof that every stage criterion passed. Preserve explicit waivers as waivers.
4. Update the current-state sections of `docs/Firmware-Development-Handoff.md` when the update changes implementation status, measurements, limits, decisions, open work, or next steps. Date the revision and preserve historical provenance.
5. Correct affected plan/checklist sections when warranted. Keep older log entries intact; append corrections or mark superseded conclusions rather than erasing history.
6. Link any actual saved evidence by path. Do not invent saved attachments or imply that a chat-only screenshot is in the repository. Preserve supplied raw logs/captures when accessible and relevant.
7. Briefly tell the user where the update was recorded. Logging a report does not authorize implementing the next stage, changing operating limits, building, flashing, or testing hardware.

The personal `docs/Firmware-Walkthrough.md` is gitignored and is not the shared status record. The plan, handoff, and these instructions should remain eligible for version control. Do not commit or push merely to record an update unless authorized.

Chats do not substitute for these files: read the current documents instead of assuming another chat's full conversation is available. Separate worktrees/checkouts may have different versions; never assume uncommitted documentation has propagated to them.
