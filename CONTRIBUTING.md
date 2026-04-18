# Contributing

Thanks for the interest. This package is primarily a personal research project, but issues and PRs are welcome.

## Setup

See [README.md — Quickstart](README.md#quickstart) for native and Docker setup.

## Running tests

```bash
python3 -m pytest test/ -v
```

Kinematic tests must pass within `1e-5` against the fixtures in [`docs/kinematic_fixtures.md`](docs/kinematic_fixtures.md). If a fixture changes, update both the fixtures doc and the TypeScript mirror in the [nishalangovender.com portfolio](https://github.com/nishalangovender/nishalangovender.com) — the two implementations share the same numbers.

## Branch conventions

- `main` — polished, default branch. All new work lands here.
- `thesis` — preserved snapshot of the as-submitted final-year thesis.
- Feature branches: `feat/<slug>`, `fix/<slug>`, `docs/<slug>`.

## Commit style

One-line subject only. No body unless absolutely necessary. Past-tense or imperative, lowercase prefix:

```
feat: add crab-steering primitive
fix: wheel-speed sign under reverse ackermann
docs: note pivot singularity in kinematics.md
test: cover counter-steer round-trip
ci: cache pip between runs
```

## Before opening a PR

- `python3 -m pytest test/ -v` passes.
- `flake8 park_bot test` passes.
- README / kinematic docs updated if behaviour changed.
