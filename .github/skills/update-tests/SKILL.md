---
name: update-tests
description: Update tests to match current code changes. Use when the user asks to update tests, fix failing tests, or get tests passing after code changes.
---

# Update Tests

When the user asks for tests to be updated, follow this workflow so tests stay in sync with the codebase and coverage stays at 100%.

## Scope: test code only, changed files only

- **Do not change non-test (production) code** unless the user explicitly asks you to. Restrict edits to test files and test-related config (e.g. fixtures, test helpers).
- **Only touch files that are currently changed.** Base your edits on uncommitted or recent changes (e.g. `git diff`, `git status`). Update only test files that correspond to changed production code, or test files that are themselves in the change set. Do not modify unrelated test files or production files.
- **If you think production code is wrong** (e.g. a failing test suggests a bug or a design mistake): do not change the production code. Describe what you think is incorrect and why, and ask the user to decide whether to fix the test or the code. Do not make the change yourself.

## 1. Inspect current changes

- Review uncommitted or recent changes (e.g. `git diff`, `git status`) to see what was added, removed, or modified.
- Identify which production modules and which test files are affected.

## 2. Remove obsolete tests

- **Tests for removed code**: If a file, class, or function was removed, remove the tests that only targeted it. Do not leave tests that import or reference deleted code or files.
- **Tests for removed behavior**: If behavior was removed or replaced, delete or rewrite tests that asserted the old behavior so they no longer depend on it.
- Keep test files and test modules consistent with the current layout (e.g. if a module was moved or renamed, move or rename its test module accordingly).

## 3. Fix failing tests

- Run tests with `make test` and fix every failing test **by changing test code only** (see Scope above).
- Update assertions, mocks, and fixtures to match current function signatures, return values, and behavior.
- Adjust imports and paths in test code if production code or tests were moved or renamed.
- If a failure suggests that production code might be wrong (e.g. the test expectation seems correct and the implementation seems wrong), report that to the user and do not edit production code—let them decide whether to fix the test or the code.
- Prefer updating existing tests over deleting them unless the scenario is no longer relevant (see step 2).

## 4. Add tests for new functionality

- **New modules, classes, or functions**: Add unit tests that cover all classes and functions.
- **New behavior in existing code**: Add or extend tests so the new behavior is covered; avoid leaving new logic untested.
- Follow existing patterns in the project (same test layout, naming, and style as nearby tests).
- Place tests in the corresponding test tree (e.g. `src/tests/...` mirroring `src/...`).
- **No line-coverage comments**: Do not add comments or docstrings that say things like "covers lines X–Y" or "this test covers …". Describe what the test does (behavior or scenario), not which lines it covers.
- **No "ask" comments**: Do not add comments that restate the user's request or instructions (e.g. "same order as in X", "as requested", "module-level types in same order as …"). Comments should describe the code or context around them, not what was asked for.

## 5. Run tests and verify coverage

- Run the full test suite: `make test`.
- Ensure all tests pass and that coverage is 100%. If the project’s default coverage threshold is lower, run pytest with a 100% threshold to verify:
  - `uv run pytest --cov=src --cov-report=term-missing --cov-fail-under=100`
- If coverage is below 100%, add or fix tests until every line in the changed (and relevant) code is covered, then run the command again.

## 6. Run lint

- After making changes, run `make lint` to ensure your edits introduce no lint issues.
- Fix any lint failures before finishing.

## Summary

1. Inspect changes.
2. Remove tests for removed code or behavior.
3. Fix all failing tests.
4. Add unit tests for all new classes and functions; cover new behavior.
5. Run `make test` and verify 100% coverage (using `--cov-fail-under=100` if needed).
6. Run `make lint` and fix any issues.
