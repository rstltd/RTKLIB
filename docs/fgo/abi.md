# librtklib ABI

## ABI 2 — current (RTKLIB-EX 2.5.2)

Two exported signatures changed while the FGO callbacks were being built:
`ddres_core()` now takes `diag(P)` instead of the full covariance, and
`fgo_dd_freeze_pairs()` takes the state to select at. Neither is a struct
layout change, and both are covered by the rule below — which is the point of
stating that rule in terms of "would an already-linked executable be unsafe"
rather than in terms of structs.

**ABI 1 was never released.** It existed only on the FGO development branch and
was superseded before merge. Nothing should be looking for a
`librtklib.so.1`; if something is, it predates versioning entirely.

## ABI 1 — first versioned ABI (superseded, never released)

**This release breaks binary compatibility with every earlier build.**
Everything that links `librtklib` must be rebuilt.

`prcopt_t` gained the `fgo_*` option fields and `rtk_t` gained `rtk_t::fgo`
(plan.md §6.6 M17/M18). Both are appended at the tail, so the offset of every
existing field is unchanged — but `sizeof()` is not, and both structs are
allocated by callers and passed across the library boundary. An application
compiled against the old headers allocates a smaller `rtk_t` than `rtkinit()`
believes it received, and `rtkinit()` writes past the end of it. This is risk
**RC-2** in plan.md §6.10.

## How the ABI is identified

`VER_RTKLIB_ABI` in `src/rtklib.h` is the ABI generation. The build turns it
into the ELF SONAME (`librtklib.so.1`) and the library file version
(`librtklib.so.1.0.0`).

It is deliberately separate from `PATCH_LEVEL`:

| Macro | Moves when | Reaches the SONAME |
|---|---|---|
| `PATCH_LEVEL` | any release | no |
| `VER_RTKLIB_ABI` | the library stops being binary-compatible | yes |

**Bump `VER_RTKLIB_ABI` for any change that makes an already-linked
executable unsafe against the new library.** Bumping `PATCH_LEVEL` instead
documents the break without preventing it.

Struct layout is only one way to break it. All of these require a bump:

- **A public struct changes layout** — `prcopt_t`, `rtk_t`, `sol_t`, `nav_t`,
  `obsd_t`, `ssat_t` and friends. Appending a field counts: callers allocate
  these, so `sizeof()` is part of the contract. *(This is what ABI 1 is.)*
- **An exported function's signature changes** — parameters added, removed,
  reordered or retyped, or a return type changed. The caller pushes the old
  argument list and the callee reads the new one. `ddres_core()` gaining its
  `ws` parameter during development is exactly this shape of change.
- **An exported symbol is removed or renamed**, including a function that
  becomes `static`. Old binaries fail to resolve it, or silently bind to
  something else.
- **The meaning of an argument or return value changes** without its type
  changing — units, sign convention, ownership of a pointer, error-code
  values. The linker cannot see this one at all, which makes it the easiest
  to miss and the worst to debug.
- **A public macro that callers compile into their own code changes value** —
  array bounds such as `MAXSAT` or `NFREQ`, or enum-like `#define` constants.
  The caller baked the old value into its own allocations.

Adding a *new* exported function, or a new `#define`, does not require a bump:
existing binaries neither reference nor depend on it.

When in doubt, bump. The cost of an unnecessary bump is a rebuild; the cost of
a missed one is memory corruption in the field.

## What the SONAME does and does not protect

**Does**: any executable linked from this release onward records
`DT_NEEDED: librtklib.so.1`. It will refuse to start against a future
`librtklib.so.2` rather than corrupting memory.

**Does not**: an executable linked *before* this release recorded
`DT_NEEDED: librtklib.so`, because the library had no SONAME for the linker to
copy. The dynamic loader resolves `DT_NEEDED` by filename and never compares
the target's SONAME, so such an executable will still load whatever
`librtklib.so` points at. That cannot be fixed retroactively — the name is
baked into binaries that already exist, and the only way to make it fail
would be to stop shipping a file called `librtklib.so` at all, which would
break linking new code.

What is in our control, and is done:

- The unversioned `librtklib.so` symlink is installed under the
  **Development** component, not **Runtime** (`NAMELINK_COMPONENT` in
  `src/CMakeLists.txt`). A runtime deployment therefore contains only
  `librtklib.so.1` and `librtklib.so.1.0.0`, and a legacy binary dropped onto
  it will fail to find its library rather than silently loading the wrong one.
- This note, so the rebuild requirement is stated rather than assumed.

The remaining exposure is a host with the development package installed and a
stale executable. plan.md §6.10 RC-2 already prescribes the answer:
**deploy the whole project in sync.**

## Checking a build

```sh
readelf -d lib/librtklib.so.1.0.0 | grep SONAME     # -> librtklib.so.1
cmake --install <build> --component Runtime          # -> no bare librtklib.so
```

The in-tree applications under `app/` link the `rtklib` CMake target and are
therefore always rebuilt with it; only external consumers are exposed.
