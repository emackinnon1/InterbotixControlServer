---
name: commit-pr-description
description: Guide for writing effective commit messages and PR descriptions. Use when the user wants to write or improve a commit message, PR description, or similar change summary for code review.
---

# Commit Message / PR Description

This skill guides how to write commit messages and PR descriptions that help reviewers understand changes quickly.

## Purpose

Commit messages and PR descriptions are for humans doing code review. They should explain what changed and why, so reviewers know where to focus and what to question. The diff shows the code; the message should add context and intent.

## Core Principles

### Explain Why, Not What

Describe the rationale and intent behind changes. Avoid restating what the diff already shows (e.g., "added X", "removed Y"). Instead, explain why the change was made and what problem it solves.

### Avoid File Inventories

Don't list added, removed, or modified files. Reviewers can see that in the diff. For moves or renames, use short phrases like "refactored middleware into a dedicated folder" or "moved survey from subgraph to microflow."

### Write for Humans

Use a natural, conversational tone. Avoid robotic or overly formal language. Assume a colleague is reading.

### Keep It Scannable

Use bullet points and short sections so reviewers can skim quickly. Long paragraphs are harder to scan.

### Length

Aim for a concise reference, not a long essay. If it's too long, people won't read it. Prioritize the most important context.

### Focus on Expectations

Help reviewers understand what changed and why, so they know where to look and what to question. Highlight non-obvious choices or trade-offs.

## Structure

A good commit message or PR description typically includes:

1. **Title or summary** – One line that captures the main change.
2. **Context** – Brief background on why the change was made (if not obvious).
3. **Sections by area** – Group changes by theme (e.g., Survey, Streaming, API) with bullets under each.
4. **Optional** – Callouts for breaking changes, migration notes, or follow-up work.

## Anti-Patterns

- Listing files that were added, removed, or modified.
- Repeating what the diff already shows.
- Long, dense paragraphs without structure.
- Overly formal or robotic language.
- Omitting the "why" and only describing the "what."
