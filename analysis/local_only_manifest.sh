#!/usr/bin/env bash
# List the local-only (git-ignored) run artefacts to back up externally: rosout logs, per-KF registration
# event folders (reg_events/) and recorded run bags (run.bag), with absolute paths and sizes.
# GT copies (analysis/gt.tum) are not listed: eval_vio.py regenerates them from the dataset.
# Usage: analysis/local_only_manifest.sh   ->  runs/local_only_backup_list.txt
REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/runs/local_only_backup_list.txt"
cd "$REPO"
ROS=$(git ls-files --others --ignored --exclude-standard runs/ | grep -E '/rosout\.log$' | sort)
EVT=$(ls -d runs/*/reg_events runs/_failed/*/reg_events 2>/dev/null | sort)
BAG=$(git ls-files --others --ignored --exclude-standard runs/ | grep -E '/run\.bag$' | sort)
size() { [ -z "$1" ] && echo 0 && return; echo "$1" | tr '\n' '\0' | du -cb --files0-from=- | tail -1 | cut -f1; }
hr() { numfmt --to=iec --suffix=B "$1"; }
S1=$(size "$ROS"); S2=$(size "$EVT"); S3=$(size "$BAG")
{
  echo "# Local-only files to back up externally (not in git), generated $(date -Iseconds)"
  echo "# rosout logs: $(echo "$ROS" | grep -c .) files, $(hr "$S1")"
  echo "# registration event folders: $(echo "$EVT" | grep -c .) folders, $(hr "$S2")"
  echo "# run bags: $(echo "$BAG" | grep -c .) files, $(hr "$S3")"
  echo "# total: $(hr $((S1 + S2 + S3)))"
  for f in $ROS $EVT $BAG; do echo "$REPO/$f"; done
} > "$OUT"
head -5 "$OUT"
