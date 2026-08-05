# Beginner User Manual Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a Chinese beginner manual that a nontechnical operator can follow, with a separate administrator appendix and a README entry.

**Architecture:** One Markdown file provides the complete user journey. The main section contains only browser operations and observable success criteria; an appendix isolates SSH commands, backup checks, manual-session solving status, outputs, and quality thresholds.

**Tech Stack:** Markdown, Git, GitHub.

## Global Constraints

- Use short Chinese sentences and numbered actions.
- Explain every technical term the first time it appears.
- Do not claim that the Web “开始求解” button processes manual samples; the current successful solve is an administrator-run offline trial.
- Keep commands out of the ordinary-operator section.
- Add a relative README link that works on GitHub.

---

### Task 1: Write and Publish the Beginner Manual

**Files:**
- Create: `docs/零基础使用手册.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: current RK3588 URL/port, 32-pose guidance, backup layout, output layout, and actual Web labels.
- Produces: a GitHub-readable manual linked from the repository homepage.

- [ ] **Step 1: Write the ordinary-operator section**

Include preparation, power/connection order, browser opening, page field explanations, the 32-shot flow, acceptance criteria, completion handoff, warnings, and common failures. Every numbered step must state what the operator should see before continuing.

- [ ] **Step 2: Write the administrator appendix**

Include exact SSH/start/tunnel/status commands, `sessions/` and `backups/` checks, the current `5×8` pattern, measured-square warning, manual-solve status, result path pattern, required matrices, and PASS thresholds.

- [ ] **Step 3: Add the README entry**

Add `[零基础使用手册](docs/零基础使用手册.md)` near the top of `README.md`.

- [ ] **Step 4: Verify document consistency**

Run `git diff --check`; verify the manual contains all 32 pose groups, the “开始求解” limitation, troubleshooting, administrator commands, result files, and no placeholder terms. Verify the README link target exists.

- [ ] **Step 5: Commit and push**

Commit both files as `docs: add beginner operation manual`, push `main` to `origin`, and confirm the GitHub remote branch points at the new commit.
