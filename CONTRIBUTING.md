# Contributing to bitrl

## Commit messages

bitrl uses [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/).
PRs are squash merged, so the **PR title** becomes the commit on `master`. A check
fails any PR whose title does not follow the format:

```
<type>(<optional scope>): <what changes>
```

| Type | Use it for | In the changelog | Version bump |
| ---- | ---------- | ---------------- | ------------ |
| `feat` | A new feature: a class, an environment, an example | Features | minor |
| `fix` | A bug fix | Bug Fixes | patch |
| `perf` | Faster code that behaves the same | Performance | patch |
| `refactor` | Restructured code that behaves the same | Refactoring | none |
| `docs` | Documentation and example pages only | Documentation | none |
| `test` | Tests only | hidden | none |
| `build` | CMake, dependencies | hidden | none |
| `ci` | GitHub workflows, `ci/` scripts | hidden | none |
| `chore`, `style` | Anything else, formatting | hidden | none |
| `revert` | Reverting an earlier commit | Reverts | none |

The scope is optional and names the part of the library, for example `actuators`,
`sensors`, `rigid_bodies`, `worlds` or `examples`.

A change that breaks existing code adds `!` after the type, `feat(actuators)!: ...`, or
a `BREAKING CHANGE:` paragraph in the PR description. It bumps the major version.

Examples:

```
feat(actuators): add a DC motor with simulation and MQTT backends
fix(rigid_bodies): use the z coordinate for the sensor frame of the diff drive robot
docs(examples): add the page for example 30
ci: cache the Chrono build
```

## Formatting

The code is formatted with clang-format 14 and the settings in `.clang-format`; CI fails
on code that is not. Format it before you push:

```
pip install clang-format==14.0.6   # once, if your clang-format is not version 14
./format.sh
```

`./format.sh --check` only reports the files that are not formatted.

Commits that only reformat code are listed in `.git-blame-ignore-revs`. Run
`git config blame.ignoreRevsFile .git-blame-ignore-revs` once so that `git blame` skips them.

## Releases

Releases are made by [release-please](https://github.com/googleapis/release-please).
After every merge to `master` it updates a release PR that bumps `version.txt` and
adds the new entries to `CHANGELOG.md`. Merging that PR tags the release, `vX.Y.Z`,
and creates the GitHub release.

`version.txt` is the only place the version is kept; CMake reads it. Do not change
it by hand.
