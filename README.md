# interactor-physics

A riscv64 Linux build of [MuJoCo](https://github.com/google-deepmind/mujoco), with host bindings for
driving it as a `libriscv` sandboxed guest. Seeded from `interactor-gyre`'s vendored MuJoCo work and
moved here after that project dropped MuJoCo for Godot's Jolt physics on entity and prop contact.

## What is here

- `thirdparty/mujoco` — MuJoCo source, a clean checkout of commit
  `b85fdca54f0e0038b804af146a0b4e94199e00d0`, tag `3.11.0`. Taken from upstream rather than from
  `interactor-gyre`'s git history, which briefly held a corrupted subtree merge. See Provenance.
- `cmake/mujoco.cmake` — the CMake recipe that adds MuJoCo as a subdirectory target, carried over.
- `src/physics/mj_physics.c` and `.h` — the MuJoCo C API wiring, `mj_physics_init`,
  `mj_physics_step`, and `mj_physics_close`, built and linked in `interactor-gyre`, ported as-is.
- `test/unit/test_mj_physics_freefall.c` — the one MuJoCo test that existed, a free-fall recovery
  check against a minimal MJCF scene.

## Scope

This repository does not yet cross-compile MuJoCo for riscv64, and does not wire it into a
`libriscv` guest. That work is unstarted, and the name states the type this repository is meant to
take rather than the one it has today.

What exists is x86_64-buildable: this exact `mj_physics.c` and `cmake/mujoco.cmake` pairing built
and linked clean in `interactor-gyre`'s containerized CI, several times, before being moved here.

## Provenance

`interactor-gyre` vendored MuJoCo as a git submodule pinned at
`b85fdca54f0e0038b804af146a0b4e94199e00d0`. An attempt to convert it to a `git subtree` was
interrupted mid-fetch and produced a corrupted squash commit, whose "MuJoCo" tree was a copy of that
repository rather than MuJoCo. It was never pushed anywhere as real MuJoCo content.

`thirdparty/mujoco` here is a fresh checkout of the same pinned commit taken directly from MuJoCo
upstream, confirmed by its Apache license header and its README banner. `mujoco.h` reports
`mjVERSION_HEADER 3011000`, which is the 3.11.0 the pin claims.
