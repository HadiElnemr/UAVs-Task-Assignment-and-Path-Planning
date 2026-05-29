#!/usr/bin/env bash
# Run all algorithms via the unified Python runners (benchmark 1 by default).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
PY="${ROOT}/.venv/bin/python"
export MPLBACKEND="${MPLBACKEND:-Agg}"
BENCH="${1:-1}"
exec "$PY" runs/run_all.py --benchmark "$BENCH" --skip-sa "${SKIP_SA:-}"
