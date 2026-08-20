# librtklib ABI

## ABI 1 — first versioned ABI (RTKLIB-EX 2.5.2)

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
| `VER_RTKLIB_ABI` | a struct crossing the library boundary changes layout | yes |

**Bump `VER_RTKLIB_ABI` on any change to the layout of a public struct** —
`prcopt_t`, `rtk_t`, `sol_t`, `nav_t`, `obsd_t`, `ssat_t` and friends —
including merely appending a field. Bumping `PATCH_LEVEL` instead documents
the break without preventing it.

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
