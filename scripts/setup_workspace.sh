#!/usr/bin/env bash
# Install the experiment launch files and code patch into the pinned checkout.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace=${1:-$repo}
source_dir="$workspace/src/multi_proxy"
expected=e9f9e3a3ab3fdc4f38bfd724566904b9948264d4
patch_file="$repo/patches/multi_proxy_instrumentation.patch"
[[ -d "$source_dir/.git" ]] || { echo "Not a git checkout: $source_dir" >&2; exit 1; }
actual=$(git -C "$source_dir" rev-parse HEAD)
[[ $actual == "$expected" ]] || { echo "Expected Multi-Proxy commit $expected, got $actual" >&2; exit 1; }
# Verify all destination launch files before changing the source checkout.
for f in "$repo"/launch/*.launch; do
  dst="$source_dir/launch/$(basename "$f")"
  if [[ -e $dst ]] && ! cmp -s "$f" "$dst"; then
    echo "Refusing to overwrite changed launch file: $dst" >&2; exit 1
  fi
done
if git -C "$source_dir" apply --reverse --check "$patch_file" 2>/dev/null; then
  echo 'Instrumentation patch already applied.'
elif git -C "$source_dir" apply --check "$patch_file"; then
  git -C "$source_dir" apply "$patch_file"
  echo 'Instrumentation patch applied.'
else
  echo 'Patch does not apply cleanly; inspect existing source changes.' >&2; exit 1
fi
for f in "$repo"/launch/*.launch; do
  dst="$source_dir/launch/$(basename "$f")"
  [[ -e $dst ]] || cp "$f" "$dst"
done
echo "Experiment files installed in $source_dir"
