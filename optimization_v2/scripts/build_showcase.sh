#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$project_root"
source optimization_v2/env.sh
colcon --log-base optimization_v2/ws/log build \
  --base-paths optimization_v2/ws/src/showcase_demo \
  --build-base optimization_v2/ws/build --install-base optimization_v2/ws/install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
python3 - <<'PY'
from pathlib import Path
import hashlib,json
p=Path('optimization_v2');source=p/'ws/src/showcase_demo/src/ReactiveActors.cc';binary=p/'ws/install/showcase_demo/lib/libshowcase_actors.so'
out=p/'showcase/actor_build.json';out.parent.mkdir(exist_ok=True)
out.write_text(json.dumps({'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest()},indent=2)+'\n')
PY
