# Costmap Curb Clearance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Keep the MPPI-controlled Prius farther from curbs, especially while turning.

**Architecture:** Preserve the existing `nav_base` footprint and MPPI controller. Expand the inflation field in both costmaps, with a slower local decay so MPPI evaluates curb-adjacent trajectories as expensive.

**Tech Stack:** ROS 2 Humble, Nav2 costmap_2d, pytest.

## Global Constraints

- Do not alter MPPI, planner, vehicle footprint, goal relay, or frame transforms.
- Set both inflation radii to `2.2` metres.
- Set local inflation cost scaling to `0.8`; retain global cost scaling at `0.7`.
- Do not commit or push.

---

### Task 1: Costmap inflation regression

**Files:**
- Modify: `test/test_nav2_mppi_integration.py`
- Modify: `config/nav2_params.yaml`

- [ ] Add assertions for the agreed local/global inflation parameters.
- [ ] Run the focused test and verify it fails on the old `1.5/2.0` settings.
- [ ] Change only the agreed inflation parameters.
- [ ] Run the focused test, full package test suite, YAML parse, and package build.
