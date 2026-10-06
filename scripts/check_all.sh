#!/bin/bash
# ============================================================================
# CHECK ALL — cổng kiểm tra tổng hợp của dự án (chạy trước mỗi lần merge lớn)
# ============================================================================
# 1. Build toàn bộ (CMake + Ninja)
# 2. ctest (183 test, bao gồm regression dữ liệu thật nếu có testdata/)
# 3. Benchmark dữ liệu thật (mặc định subset 40 recordings MotionSense + 2 dataset
#    tự thu nếu có; dùng --full để chạy hết 360)
# 4. Oracle chéo gói 160-bit (tools/validate_packet.py)
# ============================================================================
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"
FULL="${1:-}"

echo "=== [1/4] BUILD ==="
cmake -S "$ROOT" -B "$BUILD" >/dev/null
cmake --build "$BUILD" --target unit_tests replay_evaluation 2>&1 | grep -E "error|FAILED" && exit 1 || echo "OK"

echo "=== [2/4] CTEST ==="
ctest --test-dir "$BUILD" -C Debug --output-on-failure 2>&1 | tail -3

echo "=== [3/4] BENCHMARK DỮ LIỆU THẬT ==="
SUBSET_FLAG=""
if [[ "$FULL" != "--full" ]]; then SUBSET_FLAG="--subset 40"; fi
python "$ROOT/scripts/run_benchmarks.py" --label check $SUBSET_FLAG

echo "=== [4/4] ORACLE GÓI 160-BIT ==="
python "$ROOT/tools/validate_packet.py" && echo "OK"

echo "=== HOÀN TẤT — so sánh docs/check_summary.txt với docs/phase3d_summary.txt (baseline tốt nhất) ==="
