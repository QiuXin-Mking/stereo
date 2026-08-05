# README Interface Screenshot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a real RK3588 Web calibration interface screenshot in the initial `0/32` state to the GitHub README.

**Architecture:** Restart the deployed service to obtain a clean session, capture the visible in-app browser page, store the PNG under `docs/images/`, and reference it from README using a repository-relative path.

**Tech Stack:** RK3588 Web UI, in-app browser, PNG, Markdown, Git/GitHub.

## Global Constraints

- Use a real page screenshot, not a mockup.
- Screenshot must show `capturing`, `0/32`, first central-facing guidance, live preview, and controls.
- Do not include stopped, connection errors, or the removed manual-material card.
- Preserve the prior 32-shot session through the existing backup behavior.

---

### Task 1: Capture and Publish the UI Screenshot

**Files:**
- Create: `docs/images/rk3588-web-ui.png`
- Modify: `README.md`

**Interfaces:**
- Consumes: live `http://127.0.0.1:18765/` page after service restart.
- Produces: a GitHub-renderable README image.

- [ ] **Step 1: Restart and verify clean state**

Stop only the running calibration service, restart it with the existing RK3588 command, and confirm `/api/status` reports `manual_pairs: 0` and first central-facing guidance.

- [ ] **Step 2: Capture the real browser page**

Reload the local tunnel URL, confirm the required visible fields in the DOM, capture the page as PNG, and save it to `docs/images/rk3588-web-ui.png`.

- [ ] **Step 3: Embed in README**

Add `![RK3588 Web 标定界面](docs/images/rk3588-web-ui.png)` to the RK3588 browser section with a one-sentence caption.

- [ ] **Step 4: Verify and publish**

Verify the PNG type and dimensions, visually inspect it, run `git diff --check` and the full tests, commit as `docs: add RK3588 web interface screenshot`, push `main`, and confirm the GitHub remote commit.
