#!/usr/bin/env bash
set -Eeuo pipefail
LESSON="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
BASE="$(cd "$LESSON/../.." && pwd -P)"
mkdir -p "$BASE/ws/build/s01_lessons"
g++ -std=c++17 -Wall -Wextra -Wpedantic "$LESSON/lesson.cpp" -o "$BASE/ws/build/s01_lessons/lesson"
python3 "$LESSON/lesson.py" "$LESSON/valid.json"
"$BASE/ws/build/s01_lessons/lesson" "$LESSON/valid.json"
for case_name in missing wrong_domain; do
  set +e
  python3 "$LESSON/lesson.py" "$LESSON/$case_name.json"
  py_code=$?
  "$BASE/ws/build/s01_lessons/lesson" "$LESSON/$case_name.json"
  cpp_code=$?
  set -e
  [[ "$py_code" == 2 && "$cpp_code" == 2 ]] || exit 1
done
echo 'LESSONS_PASS: both languages accept the valid input and reject missing resources and wrong domain.'
