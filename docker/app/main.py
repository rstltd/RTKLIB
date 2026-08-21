"""RTKLIB GNSS post-processing service.

Wraps rnx2rtkp behind HTTP so an internal network can submit RINEX files and
get a position solution back.

What this actually solves, stated plainly because the repository is in the
middle of adding a second engine: the solver used here is RTKLIB's extended
Kalman filter.  The Factor Graph Optimization work (plan.md) is compiled into
the library and reports its state through /capabilities, but it does not yet
produce solutions -- fgo_process_epoch() returns FGO_ERR_NOTCONV and rtkpos.c
has no solver branch.  Asking for an FGO solver here is refused rather than
silently served by the EKF.
"""

from __future__ import annotations

import asyncio
import json
import os
import re
import shutil
import subprocess
import tempfile
import uuid
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.responses import JSONResponse, PlainTextResponse

RNX2RTKP = os.environ.get("RTKLIB_BIN", "/opt/rtklib/bin/rnx2rtkp")
CONF_DIR = Path(os.environ.get("RTKLIB_CONF_DIR", "/opt/rtklib/conf"))
MAX_UPLOAD_MB = int(os.environ.get("RTKLIB_MAX_UPLOAD_MB", "256"))
SOLVE_TIMEOUT_S = int(os.environ.get("RTKLIB_SOLVE_TIMEOUT_S", "900"))

app = FastAPI(
    title="RTKLIB solve service",
    description=__doc__,
    version=os.environ.get("RTKLIB_SERVICE_VERSION", "0"),
)


class BodySizeLimitMiddleware:
    """Reject oversized request bodies before anything is written to disk.

    Enforcing this inside the endpoint is too late: Starlette parses the whole
    body and spools every part to a temporary file before the handler runs, so
    a limit there bounds only the second copy. An oversized upload would fill
    the container's tmpfs and be killed rather than receiving a 413.

    Written as raw ASGI rather than as an http middleware because FastAPI
    catches exceptions raised while it parses the body and turns them into
    "There was an error parsing the body" with status 400 -- so a limit that
    signals by raising never reaches an outer handler. Measured: a 3 MB
    chunked body against a 1 MB limit produced 400, not 413. Instead the
    overflow is recorded and the downstream response is REPLACED on its way
    out, which works whichever way the framework chooses to fail.

    Content-Length is checked first, which covers ordinary clients without
    reading a byte. Chunked requests declare no length, so their bytes are
    counted as they arrive.
    """

    def __init__(self, app, max_bytes: int) -> None:
        self.app = app
        self.max_bytes = max_bytes

    async def __call__(self, scope, receive, send) -> None:
        if scope.get("type") != "http" or scope.get("method") not in (
            "POST",
            "PUT",
            "PATCH",
        ):
            await self.app(scope, receive, send)
            return

        headers = {k.lower(): v for k, v in scope.get("headers", [])}
        declared = headers.get(b"content-length")
        if declared is not None:
            try:
                if int(declared) > self.max_bytes:
                    await self._reject(send)
                    return
            except ValueError:
                pass  # malformed; the counter below still bounds it

        overflow = False
        received = 0

        async def counting_receive():
            nonlocal overflow, received
            message = await receive()
            if message.get("type") == "http.request":
                received += len(message.get("body", b""))
                if received > self.max_bytes:
                    overflow = True
                    # Report a disconnect so the parser stops pulling rather
                    # than waiting for a body that will never be accepted.
                    return {"type": "http.disconnect"}
            return message

        async def replacing_send(message):
            if overflow:
                if message.get("type") == "http.response.start":
                    await self._start(send)
                    return
                if message.get("type") == "http.response.body":
                    await self._body(send)
                    return
            await send(message)

        await self.app(scope, counting_receive, replacing_send)

    def _payload(self) -> bytes:
        mb = self.max_bytes // (1024 * 1024)
        return json.dumps({"detail": f"body exceeds {mb} MB"}).encode()

    async def _start(self, send) -> None:
        body = self._payload()
        await send(
            {
                "type": "http.response.start",
                "status": 413,
                "headers": [
                    (b"content-type", b"application/json"),
                    (b"content-length", str(len(body)).encode()),
                ],
            }
        )

    async def _body(self, send) -> None:
        await send(
            {"type": "http.response.body", "body": self._payload(), "more_body": False}
        )

    async def _reject(self, send) -> None:
        await self._start(send)
        await self._body(send)


app.add_middleware(BodySizeLimitMiddleware, max_bytes=MAX_UPLOAD_MB * 1024 * 1024)



def _run(args: list[str], timeout: int) -> subprocess.CompletedProcess:
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout)


def _build_info() -> dict:
    """Read the build metadata baked in at image build time."""
    info: dict = {}
    meta = Path("/opt/rtklib/build-info.txt")
    if meta.is_file():
        for line in meta.read_text().splitlines():
            if ":" in line and not line.startswith("#"):
                k, v = line.split(":", 1)
                info[k.strip()] = v.strip()
    return info


@app.get("/health", response_class=JSONResponse)
async def health() -> JSONResponse:
    """Liveness plus a real check that the solver binary runs.

    Returns 503 when it does not. Both healthchecks look only at the status
    code, so reporting a failure in the body while answering 200 would leave a
    container with an unusable solver marked healthy.
    """
    ok = Path(RNX2RTKP).is_file() and os.access(RNX2RTKP, os.X_OK)
    version = None
    if ok:
        try:
            # Running --version proves the dynamic linker resolves librtklib
            # and its dependencies, which merely stat-ing the file does not.
            proc = _run([RNX2RTKP, "--version"], timeout=10)
            if proc.returncode != 0:
                ok = False
                version = f"exit {proc.returncode}: {(proc.stderr or '').strip()[:200]}"
            else:
                version = (proc.stdout or proc.stderr).strip().splitlines()[0]
        except Exception as exc:  # noqa: BLE001
            ok = False
            version = f"{type(exc).__name__}: {exc}"
    else:
        version = f"solver not executable: {RNX2RTKP}"
    return JSONResponse(
        {"status": "ok" if ok else "unhealthy", "solver": version},
        status_code=200 if ok else 503,
    )


@app.get("/capabilities", response_class=JSONResponse)
async def capabilities() -> dict:
    """What this build can and cannot do.

    The fgo block is deliberately explicit.  The library exports the FGO C ABI
    and, when built with ENABLE_FGO=ON, links GTSAM -- but exporting an ABI is
    not the same as being able to solve with it, and a caller choosing a
    solver needs to know the difference.
    """
    info = _build_info()
    fgo_built = info.get("enable-fgo", "OFF").upper() == "ON"
    return {
        "rtklib": {
            "version": info.get("rtklib-version"),
            "abi": info.get("rtklib-abi"),
            "macros": info.get("rtklib-macros"),
        },
        "solvers": {
            "available": ["ekf"],
            "requested_but_unavailable": {
                "fgo-batch": "not implemented yet (plan.md 11.3 PR-6/PR-7)",
                "fgo-sliding": "not implemented yet (plan.md 11.3 PR-6/PR-7)",
                "fgo-isam2": "not implemented yet (plan.md 11.3 PR-6/PR-7)",
            },
        },
        "fgo": {
            "compiled_in": fgo_built,
            "gtsam": info.get("gtsam-version") if fgo_built else None,
            "solves": False,
            # The wording has to follow the build. A fixed string saying GTSAM
            # is linked would be a lie in the ENABLE_FGO=OFF image, and this
            # field exists precisely so a caller does not have to guess.
            "note": (
                (
                    "The FGO C ABI is present and GTSAM is linked, but "
                    "fgo_process_epoch() returns FGO_ERR_NOTCONV and rtkpos.c "
                    "has no solver branch yet."
                )
                if fgo_built
                else (
                    "This image was built with ENABLE_FGO=OFF. The FGO C ABI "
                    "resolves to the stub in fgo_stub.c, which returns "
                    "FGO_ERR_DISABLED, and GTSAM is not linked."
                )
            )
            + " All solutions from /solve come from the EKF, and the two "
              "builds produce identical results.",
        },
        "presets": sorted(p.stem for p in CONF_DIR.glob("*.conf")),
        "limits": {
            "max_upload_mb": MAX_UPLOAD_MB,
            "solve_timeout_s": SOLVE_TIMEOUT_S,
        },
    }


@app.get("/presets/{name}", response_class=PlainTextResponse)
async def preset(name: str) -> str:
    """Return a shipped option file, so a caller can see exactly what it sets."""
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", name):
        raise HTTPException(400, "bad preset name")
    path = CONF_DIR / f"{name}.conf"
    if not path.is_file():
        raise HTTPException(404, f"no such preset: {name}")
    return path.read_text()


async def _save(upload: UploadFile, dest: Path, budget: list[int]) -> None:
    """Stream an upload to disk, enforcing a shared size budget."""
    with dest.open("wb") as fh:
        while chunk := await upload.read(1 << 20):
            budget[0] -= len(chunk)
            if budget[0] < 0:
                raise HTTPException(
                    413, f"upload exceeds {MAX_UPLOAD_MB} MB in total"
                )
            fh.write(chunk)


@app.post("/solve")
async def solve(
    rover: UploadFile = File(..., description="rover observation RINEX"),
    base: Optional[UploadFile] = File(None, description="base observation RINEX"),
    nav: UploadFile = File(..., description="navigation RINEX"),
    preset: str = Form("static", description="option preset name"),
    solver: str = Form("ekf", description="ekf only; see /capabilities"),
    stat: bool = Form(False, description="also return the solution-status file"),
):
    """Solve one dataset and return the .pos, optionally with the .stat file."""
    if solver != "ekf":
        # Refused rather than quietly served by the EKF: a caller that asked
        # for FGO and got a number back would reasonably assume it was FGO.
        raise HTTPException(
            400,
            f"solver '{solver}' is not available in this build; see "
            "/capabilities. Only 'ekf' produces solutions today.",
        )
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", preset):
        raise HTTPException(400, "bad preset name")
    conf = CONF_DIR / f"{preset}.conf"
    if not conf.is_file():
        raise HTTPException(404, f"no such preset: {preset}")

    # A relative preset asks rnx2rtkp for receiver 2, and without base
    # observations it fails deep inside with "no position in rinex header" --
    # accurate, but useless to whoever sent the request. Say what is wrong and
    # what to send instead.
    if base is None and not _is_single_point(conf):
        raise HTTPException(
            400,
            {
                "message": (
                    f"preset '{preset}' is a relative (differential) mode and "
                    "needs base observations"
                ),
                "hint": "supply 'base', or use a single-point preset",
                "single_point_presets": _single_point_presets(),
            },
        )

    job = uuid.uuid4().hex[:12]
    work = Path(tempfile.mkdtemp(prefix=f"solve-{job}-"))
    try:
        budget = [MAX_UPLOAD_MB * 1024 * 1024]
        rov_p = work / "rover.obs"
        nav_p = work / "nav.rnx"
        await _save(rover, rov_p, budget)
        await _save(nav, nav_p, budget)
        inputs = [str(rov_p)]
        if base is not None:
            bas_p = work / "base.obs"
            await _save(base, bas_p, budget)
            inputs.append(str(bas_p))
        inputs.append(str(nav_p))

        out = work / "solution.pos"
        # -y is passed either way. The presets set out-outstat=residual, so
        # omitting it does not mean "off" -- it means the solver writes a
        # status file on every request that nobody reads, which on a long
        # dataset is both large and slow.
        args = [RNX2RTKP, "-k", str(conf), "-o", str(out), "-y", "2" if stat else "0"]
        args += inputs

        try:
            proc = await asyncio.to_thread(_run, args, SOLVE_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            raise HTTPException(504, f"solve exceeded {SOLVE_TIMEOUT_S}s")

        if proc.returncode != 0 or not out.is_file():
            raise HTTPException(
                422,
                {
                    "message": "solve failed",
                    "returncode": proc.returncode,
                    "stderr": (proc.stderr or "")[-4000:],
                },
            )

        text = out.read_text()
        epochs = sum(1 for ln in text.splitlines() if ln and not ln.startswith("%"))
        if epochs == 0:
            # A .pos with only a header is a failure the exit code does not
            # report -- usually no common satellites, or a base/rover mismatch.
            raise HTTPException(
                422,
                {
                    "message": "solver produced no epochs",
                    "stderr": (proc.stderr or "")[-4000:],
                },
            )

        body: dict = {
            "job": job,
            "solver": "ekf",
            "preset": preset,
            "epochs": epochs,
            "quality": _quality_counts(text),
            "pos": text,
        }
        if stat:
            stat_p = Path(str(out) + ".stat")
            body["stat"] = stat_p.read_text() if stat_p.is_file() else None
        return JSONResponse(body)
    finally:
        shutil.rmtree(work, ignore_errors=True)


def _is_single_point(conf: Path) -> bool:
    """Whether a preset solves without base observations."""
    for line in conf.read_text().splitlines():
        if line.strip().startswith("pos1-posmode"):
            value = line.split("=", 1)[1].split("#", 1)[0].strip().lower()
            # MODOPT in src/options.c: 0 single, 1 dgps, 2 kinematic,
            # 3 static, 4 static-start, 5 movingbase, 6 fixed, 7 ppp-kine,
            # 8 ppp-static, 9 ppp-fixed. Either spelling is valid in a
            # configuration file, so both are accepted.
            return value in {
                "single", "0",
                "ppp-kine", "7",
                "ppp-static", "8",
                "ppp-fixed", "9",
            }
    return False


def _single_point_presets() -> list[str]:
    return sorted(p.stem for p in CONF_DIR.glob("*.conf") if _is_single_point(p))


def _quality_counts(pos_text: str) -> dict:
    """Count epochs by RTKLIB quality flag: 1 fix, 2 float, 5 single, ..."""
    names = {1: "fix", 2: "float", 3: "sbas", 4: "dgps", 5: "single", 6: "ppp"}
    counts: dict = {}
    for line in pos_text.splitlines():
        if not line or line.startswith("%"):
            continue
        parts = line.split()
        if len(parts) > 5:
            try:
                q = int(parts[5])
            except ValueError:
                continue
            counts[names.get(q, str(q))] = counts.get(names.get(q, str(q)), 0) + 1
    return counts
