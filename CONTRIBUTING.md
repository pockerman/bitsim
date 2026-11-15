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
`sensors`, `envs`, `rigid_bodies`, `worlds` or `examples`.

A change that breaks existing code adds `!` after the type, `feat(actuators)!: ...`, or
a `BREAKING CHANGE:` paragraph in the PR description. It bumps the major version.

Examples:

```
feat(actuators): add a DC motor with simulation and MQTT backends
fix(rigid_bodies): use the z coordinate for the sensor frame of the diff drive robot
docs(examples): add the page for example 30
ci: cache the Chrono build
```

## Releases

Releases are made by [release-please](https://github.com/googleapis/release-please).
After every merge to `master` it updates a release PR that bumps `version.txt` and
adds the new entries to `CHANGELOG.md`. Merging that PR tags the release, `vX.Y.Z`,
and creates the GitHub release.

`version.txt` is the only place the version is kept; CMake reads it. Do not change
it by hand.
