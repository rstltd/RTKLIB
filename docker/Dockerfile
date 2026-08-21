# RTKLIB solve service.
#
#   docker build -f docker/Dockerfile -t rtklib-solve:local .
#
# Build context is the REPOSITORY ROOT, not docker/, because the build needs
# src/, app/, tools/fgo/ and the CMake files.
#
# Two stages.  The builder needs a full conda environment for GTSAM -- about
# 2 GB -- and none of that belongs in a service image, so the runtime stage
# takes only the executable, librtklib, and the shared objects ldd says are
# actually required.

# =============================================================================
# builder
# =============================================================================
FROM debian:12-slim AS builder

ARG ENABLE_FGO=ON
ARG CONDA_INSTALLER=https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential ca-certificates curl file git \
    && rm -rf /var/lib/apt/lists/*

# With FGO on, GTSAM and cmake come from the same pinned lock the developer
# environment uses, so the image cannot drift from what the gates were run
# against.  With it off there is nothing to pin: the build needs only a
# compiler and cmake, which apt supplies in a fraction of the time and without
# a 2 GB conda tree.
COPY tools/fgo/conda-linux-64.lock /tmp/conda-linux-64.lock
RUN set -eu; \
    if [ "$ENABLE_FGO" = "ON" ]; then \
        curl -fsSL "$CONDA_INSTALLER" -o /tmp/miniforge.sh; \
        bash /tmp/miniforge.sh -b -p /opt/conda; \
        /opt/conda/bin/conda create -y -p /opt/fgoenv --file /tmp/conda-linux-64.lock; \
        /opt/conda/bin/conda clean -afy; \
        ln -s /opt/fgoenv/bin/cmake /usr/local/bin/cmake; \
        ln -s /opt/fgoenv/bin/ninja /usr/local/bin/ninja; \
    else \
        apt-get update; \
        apt-get install -y --no-install-recommends cmake ninja-build; \
        rm -rf /var/lib/apt/lists/*; \
    fi; \
    rm -f /tmp/miniforge.sh /tmp/conda-linux-64.lock

WORKDIR /src
COPY CMakeLists.txt ./
COPY src ./src
COPY app ./app
COPY lib ./lib
COPY test ./test
COPY tools ./tools

# The system compiler, not conda's: tools/fgo/fgo-toolchain.cmake pins it so
# that floating-point results stay reproducible (plan.md 6.9).
# fgo-toolchain.cmake exists to join the system compiler to a conda GTSAM, and
# it refuses to configure when GTSAM is absent -- correctly.  With FGO off
# there is no GTSAM to join, so the compilers are pinned directly instead; the
# pinning itself matters either way (plan.md 6.9).
RUN set -eu; \
    if [ "$ENABLE_FGO" = "ON" ]; then \
        FGO_ENV_PREFIX=/opt/fgoenv cmake -S /src -B /build -G Ninja \
            --toolchain /src/tools/fgo/fgo-toolchain.cmake \
            -DCMAKE_BUILD_TYPE=Release -DENABLE_FGO=ON -DBUILD_TESTING=OFF; \
    else \
        cmake -S /src -B /build -G Ninja \
            -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
            -DCMAKE_BUILD_TYPE=Release -DENABLE_FGO=OFF -DBUILD_TESTING=OFF; \
    fi; \
    cmake --build /build --target rnx2rtkp

# Collect the executable, the library and every shared object they resolve to,
# excluding the ones a base image already provides.
RUN mkdir -p /out/bin /out/lib /out/conf \
    && cp /src/bin/rnx2rtkp /out/bin/ \
    && cp -P /src/lib/librtklib.so* /out/lib/ \
    && for f in /out/bin/rnx2rtkp /src/lib/librtklib.so.*.*.*; do \
         LD_LIBRARY_PATH=/src/lib:/opt/fgoenv/lib ldd "$f" 2>/dev/null \
         | awk '/=> \//{print $3}'; \
       done | sort -u \
       | grep -vE '^/(lib|usr/lib)/x86_64-linux-gnu/(libc|libm|libdl|libpthread|librt|ld-linux)' \
       | while read -r so; do cp -Lv "$so" /out/lib/ >/dev/null; done \
    && strip --strip-unneeded /out/bin/rnx2rtkp /out/lib/*.so* 2>/dev/null || true

# Metadata the service reports through /capabilities.  Generated from what was
# actually built, not restated by hand.
RUN set -eu; \
    ver=$(sed -n 's/^#define PATCH_LEVEL[ ]*"\([^"]*\)".*/\1/p' /src/src/rtklib.h); \
    abi=$(sed -n 's/^#define VER_RTKLIB_ABI[ ]*"\([^"]*\)".*/\1/p' /src/src/rtklib.h); \
    macros=$(sed -n 's/^add_definitions(\(-D.*\))$/\1/p' /src/CMakeLists.txt | head -1); \
    { echo "# generated at image build time"; \
      echo "rtklib-version: $ver"; \
      echo "rtklib-abi: $abi"; \
      echo "rtklib-macros: $macros"; \
      echo "enable-fgo: ${ENABLE_FGO}"; \
      if [ "${ENABLE_FGO}" = "ON" ]; then \
        echo "gtsam-version: $(/opt/conda/bin/conda list -p /opt/fgoenv gtsam 2>/dev/null | awk '$1=="gtsam"{print $2}')"; \
      fi; \
      echo "built-at: $(date -u +%Y-%m-%dT%H:%M:%SZ)"; \
    } > /out/build-info.txt

COPY docker/conf/ /out/conf/

# =============================================================================
# runtime
# =============================================================================
FROM python:3.11-slim AS runtime

LABEL org.opencontainers.image.title="RTKLIB solve service" \
      org.opencontainers.image.description="RINEX post-processing over HTTP (RTKLIB EKF)" \
      org.opencontainers.image.source="https://github.com/rstltd/RTKLIB"

RUN groupadd --system rtklib && useradd --system --gid rtklib --home /app rtklib

COPY --from=builder /out /opt/rtklib
ENV LD_LIBRARY_PATH=/opt/rtklib/lib \
    RTKLIB_BIN=/opt/rtklib/bin/rnx2rtkp \
    RTKLIB_CONF_DIR=/opt/rtklib/conf \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

COPY docker/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm -f /tmp/requirements.txt

# Fail the build rather than ship an image whose solver cannot start.
RUN /opt/rtklib/bin/rnx2rtkp --version

WORKDIR /app
COPY docker/app /app
USER rtklib

EXPOSE 8000
HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 \
    CMD python -c "import urllib.request,sys; sys.exit(0 if urllib.request.urlopen('http://127.0.0.1:8000/health',timeout=3).status==200 else 1)"

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
