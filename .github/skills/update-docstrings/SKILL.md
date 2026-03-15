---
name: update-docstrings
description: Update docstrings and documentation to match current code and team standards. Use when the user asks to update docstrings, fix docs, add docstrings, or keep documentation in sync after code changes.
---

# Update Docstrings and Documentation

This skill covers **both** in-code docstrings and **project documentation** (e.g. `docs/`, README, ADRs). When the user asks for docstrings or documentation to be updated, follow this workflow so docstrings and project docs stay in sync with the code.

**Documentation describes current state only.** Do not add or keep text that explains what changed or legacy behavior (e.g. "legacy behavior was …", "this was changed to …"). Update docs to describe how things work now; remove or rewrite any legacy or changelog-style content. Only document change history if the user explicitly asks for it.

**Comments describe the code, not the ask.** Do not add or keep comments that restate the user's request or instructions (e.g. "same order as in X", "as requested", "module-level types in same order as …"). Comments should describe what the code does or the context around it; the reader has the code and does not need the original ask echoed in a comment.

**Only touch currently changed files.** Base your edits on uncommitted or recent changes (e.g. `git diff`, `git status`). Update docstrings only in files that are in the change set. Update project docs (e.g. `docs/`) only when they describe code or behavior that changed. Do not modify unrelated files.

## Docstring format

### Modules and classes

Use a description only:

```
"""
<description>
"""
```

### Functions and methods

Use a description plus only the sections that apply (Args, Returns, Yields, Raises):

```
"""
<function description>

Args: (only if there are parameters)
  <arg name>: <arg description>

Returns: (only if the callable returns a value)
  <return type>: <return value description>

Yields: (only if the callable yields)
  <yield type>: <yield value description>

Raises: (only if the callable raises documented exceptions)
  <exception type>: <when it is raised>
"""
```

- **Generic language only**: Do not reference variable names, line numbers, or file paths. Use generic wording (e.g. "the state", "the request body") so docstrings remain valid when code is renamed or moved.
- **Only include sections that apply**: Omit Args/Returns/Yields/Raises when they do not apply.

## 1. Inspect current changes

- Review uncommitted or recent changes (e.g. `git diff`, `git status`) to see what was added, removed, or modified.
- Identify affected modules, classes, functions, **and** any project docs that describe that code or behavior (e.g. `docs/`, README, architecture or ADR files). Plan to update both docstrings and docs as needed.

## 2. Update or remove obsolete docstrings

- **Removed code**: No docstring to update; ensure no leftover references in other docstrings or comments.
- **Changed signatures or behavior**: Update existing function/method docstrings to match current parameters, return value, yields, and raised exceptions. Use the module/class format for modules and classes (description only); use the function format for callables.
- **Stale references**: Rewrite any docstrings that mention old names, paths, or behavior in generic language.

## 3. Add docstrings for new or undocumented code

- **New modules**: Add a module-level docstring (description only).
- **New classes**: Add a class docstring (description only).
- **New functions or methods**: Add a docstring with description and only the Args/Returns/Yields/Raises sections that apply.
- Keep descriptions concise and generic; match the style of the rest of the codebase.

## 4. Check and update project documentation

- Look under `docs/` (and any other doc locations, e.g. README, ADRs) for content that describes the changed code or behavior.
- **Current state only**: Docs should describe how things work now. Update or remove any text that describes legacy behavior, what was changed, or changelog-style narrative—unless the user explicitly asked for that.
- **Outdated docs**: If code or APIs were renamed, moved, or removed, update or remove the corresponding docs (paths, examples, step-by-step instructions) so they reflect the current codebase.
- **New or changed behavior**: If you find **new** features, endpoints, or flows that are not yet documented in project docs, **do not add or update docs for them automatically**. Instead, list what you found and **ask the user**: "I noticed [brief list]. Would you like me to document this in the project docs?" Only add or update docs for new behavior after the user confirms. For **existing** documented behavior that changed, update the docs to match the current behavior without asking.
- **Cross-references**: Fix links, file paths, and module names in docs so they point to the current codebase.

## 5. Apply the format everywhere you touch

- Modules and classes: description only.
- Functions and methods: description plus Args/Returns/Yields/Raises when applicable.
- Use generic language; avoid variable names and line numbers.

## 6. Run lint

- After making changes, run `make lint` to ensure your edits introduce no lint issues.
- Fix any lint failures before finishing.

## Summary

1. Inspect changes to find affected code **and** related project docs; plan updates for both docstrings and docs.
2. Remove or fix obsolete docstrings; align existing docstrings with current signatures and behavior.
3. Add docstrings for new modules, classes, and functions (description-only for modules/classes; full format for callables).
4. Check `docs/` and other documentation: fix outdated docs and references; for **new** behavior not yet in docs, ask the user if they want it documented before adding it.
5. Ensure every touched docstring follows the correct format (module/class vs function) and uses generic language only.
6. Run `make lint` and fix any issues.
