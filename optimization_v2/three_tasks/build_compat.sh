#!/usr/bin/env bash
set -Eeuo pipefail
r01_app="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# A candidate only: never silently replace the qualified .so or its hash lock.
mkdir -p "$r01_app/build/candidate"
g++ -std=c++17 -O2 -fPIC -shared "$r01_app/../ws/src/task_runtime/src/shutdown_compat.cc" \
 -o "$r01_app/build/candidate/libr01_shutdown_compat.so" \
 -Wl,-z,nodelete -Wl,--no-as-needed -lboost_thread -ldl -pthread
sha256sum "$r01_app/build/candidate/libr01_shutdown_compat.so"
