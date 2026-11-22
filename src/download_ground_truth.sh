#!/usr/bin/env bash
# S3E Campus Road 1 RTK text files; verify SHA-256 before installation.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
dest="$root/datasets/S3E/ground_truth_campus_road_1"
mkdir -p "$dest"
names=(alpha bob carol)
hashes=(b8581aa32e6d11aab466ab0f0c9406301aab8d6b4ed4d9a767f4ba592bc88cc8 42cc45f016de24e6ee6c50f33b03c16becfd14b7c253f1b1c099b2d7d2a5cd2e 93d12b1bbc414d87e1c11199413668425ab7cf61504a2b916aff71e6593a37bd)
base=https://huggingface.co/datasets/PengYu-Team/S3E/resolve/main/S3Ev1/S3E_Campus_Road_1
tmp=''
cleanup() { [[ -z $tmp ]] || rm -f -- "$tmp"; }
trap cleanup EXIT
for i in 0 1 2; do
  target="$dest/${names[$i]}_gt.txt"
  if [[ -e $target ]]; then
    printf '%s  %s\n' "${hashes[$i]}" "$target" | sha256sum -c -
    continue
  fi
  tmp=$(mktemp "$dest/.${names[$i]}.XXXXXX")
  curl --fail --location --retry 2 --output "$tmp" "$base/${names[$i]}_gt.txt"
  printf '%s  %s\n' "${hashes[$i]}" "$tmp" | sha256sum -c -
  mv -- "$tmp" "$target"
  tmp=''
done
