---
name: code-review
description: Review code for quality, correctness, and maintainability following team criteria. Use when reviewing pull requests, examining code changes, or when the user asks for a code review.
---

# Code Review

## Quick Start

When reviewing code:

1. Check correctness and dependency/import integrity
2. Verify commit and PR scope (no mixed concerns)
3. Assess code quality (types, docstrings, tests)
4. Check that documentation describes current state only (no legacy/changelog narrative)
5. Note whether linting passes (suggest running `make lint`; flag if changes might introduce lint issues)
6. Flag security and error-handling issues

---

## What to Look For

### Code Smells

- Duplication, overly complex logic, unclear naming
- Unnecessary indirection or abstraction
- Long functions, classes, or files that should be split

### Pydantic Types

- Function signatures should have correct Pydantic (or standard) types
- Check for missing or incorrect type hints in parameters and return values

### Docstrings

Docstrings must follow this format. Include only the sections that apply:

```
  """
  <function description>

  Args: (only if there are args)
    <arg name>: <arg description>

  Returns: (only if there are returns)
    <return type>: <return value description>

  Yields: (only if there are yields)
    <yield type>: <yield value description>

  Raises: (only if there are raises)
    <raise type>: <raise description>
  """
```

- **Avoid strict references**: Don't reference variable names or line numbers in docstrings. Use generic language (e.g., "the state" not "the `state` variable") so docstrings remain valid when variables are renamed or code moves.

### Documentation

- **Current state only**: Documentation (in-code docstrings, comments, and project docs like `docs/`) should describe how things work *now*. Flag any text that:
  - Describes legacy or old behavior (e.g. "legacy behavior was …", "previously we …")
  - Narrates what was changed (e.g. "this was changed to …", "we refactored this so that …")
  - Reads like a changelog or change history
- Suggest rewriting or removing that content so the doc describes current behavior only—unless the user explicitly asked for change history or legacy documentation.

### Comments

- **Describe the code, not the ask**: Comments should describe what the code does or the context around it. Flag comments that restate the user's request or instructions (e.g. "same order as in X", "as requested", "module-level types in same order as …"). The reader has the code; comments should add useful context about behavior or structure, not echo what was asked for.

### References After Removal

- If functions, files, or modules were **removed**, check for any remaining references (imports, calls, usages)
- Ensure nothing still imports from or calls deleted code

### Dead Code

- If functions or files were **added**, verify they are actually used

### Test Coverage

- Read the tests for any files added or changed
- Verify the tests cover the entire module properly
- **No line-coverage comments**: Flag test comments or docstrings that say things like "covers lines X–Y"; tests should describe what they do, not which lines they cover

### Linting

- When reviewing changes, consider whether they might introduce lint issues (formatting, style, unused imports, etc.).
- Recommend running `make lint` to confirm; flag if the branch or commit should fix lint before merge.

---

## Commit and PR Structure

### Scope

- **Single concern per commit**: Functional changes (e.g., `rag_messages.append`) belong with the feature they enable, not in a chore commit.
- **Chore commits**: Docstrings, types, formatting only—no behavioral changes.
- **Feature commits**: One logical feature or refactor. Don't mix unrelated changes.

### Dependencies

- **Import correctness**: Deleting a module breaks anything that imports it. Before removing `foo.py`, ensure no other file imports from it (e.g., `streaming.py` and `interrupt_helpers`).
- **Multiple commits**: When reviewing multiple commits, flag dependency or merge-order issues (e.g., B depends on A but A comes later) so the user can merge in the right order.

### .talismanrc (per commit)

- For each commit, check `.talismanrc`
- When a file is **modified**, its checksum may need updating; update if the pre-commit hook fails
- When a file is **deleted or moved**, check `.talismanrc` for a reference to it and remove or update that entry

---

## Error Handling

- Exceptions should be meaningful and not swallowed silently.
- Consider edge cases and failure modes.

---

## Feedback Format

Use this structure when giving feedback:

- **Critical** – Must fix before merge (bugs, broken imports, security issues)
- **Should fix** – Important improvements (scope drift, missing tests)
- **Consider** – Optional polish (typos, clearer names, minor style)

Be specific: point to files/lines and suggest concrete changes when possible.

---

## Checklist

- [ ] No code smells
- [ ] Pydantic types correct in function signatures
- [ ] Docstrings present and follow format; only include Args/Returns/Yields/Raises when applicable
- [ ] Docstrings use generic language, not variable names or line numbers
- [ ] Documentation describes current state only (no legacy/changelog narrative unless requested)
- [ ] No test comments or docstrings that say "covers lines …" or similar
- [ ] No comments that restate the user's request (e.g. "same order as …", "as requested"); comments describe the code/context only
- [ ] No references to removed functions/files/modules
- [ ] New functions/files are actually used
- [ ] Tests cover the changes and the module properly
- [ ] No circular or broken imports
- [ ] Linting passes (`make lint`); no new lint issues from the changes
- [ ] Commit/PR scope is appropriate (no functional changes in chores)
- [ ] No dependency or merge-order issues (when reviewing multiple commits)
- [ ] .talismanrc updated when files are modified, deleted, or moved
- [ ] No secrets or sensitive data in code
- [ ] Error handling is reasonable
