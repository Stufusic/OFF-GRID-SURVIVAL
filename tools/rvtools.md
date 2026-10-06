# CODE REVIEW — THƯ MỤC `tools/` (CÔNG CỤ XỬ LÝ / KIỂM CHỨNG)

**Ngày review:** 06/09/2026 | **Phạm vi:** `tools/validate_packet.py`, `tools/replay_evaluation.cpp`, `tools/debug_stairs.cpp`

---

## 1. TỔNG QUAN

3 công cụ dev: `validate_packet.py` (kiểm chứng gói 160-bit độc lập với C++, dùng làm oracle chéo), `replay_evaluation.cpp` (replay dataset thật + xuất kết quả đánh giá), `debug_stairs.cpp` (debug phát hiện cầu thang trên 1 dataset).

## 2. ĐÁNH GIÁ THEO FILE

### validate_packet.py — ✅ TỐT
- Roundtrip encode/decode đối xứng; khôi phục dấu int24 lat/lon đúng (:79-86); altitude 12-bit + ratio 4-bit nhất quán với C++ (:40-41 vs :90); các clamp đúng.
- **[LOW]** Chưa test NegativeLAT/NEG-LON biên (±90/±180) và chưa có self-test `if __name__ == "__main__"` chạy assert — khuyến nghị thêm vài assert biên để dùng như oracle CI.

### replay_evaluation.cpp — [MEDIUM] chỉ dùng được trên máy tác giả
- **[MEDIUM] Hardcoded path cá nhân** (:194-198): `C:/Users/KieuKhang/Downloads/...` — không chạy được máy khác. Fix: nhận argv[1] = folder dataset, argv[2] = output (10 dòng).
- **[LOW] Output stream không kiểm tra** (:190): `file_out` không check `is_open()` — ghi vào file fail thì mất kết quả im lặng.
- Giá trị: logic timeline/rộp metric của nó chính là nguồn `evaluation_results.txt` — nên nâng cấp thành công cụ chính thức (argv + kiểm tra) thay vì script one-off.

### debug_stairs.cpp — [LOW] one-off
- Hardcoded path (:12) — cùng vấn đề trên; mục đích debug đơn lẻ, chấp nhận được nhưng nên tham số hóa.

## 3. KHUYẾN NGHỊ

1. Tham số hóa 2 tool C++ bằng argv + trả exit-code khác 0 khi lỗi (để CI gọi được).
2. Gộp validate_packet.py vào CI (so khớp với roundtrip test C++ đã có — hiện 2 oracle độc lập nhưng chưa chạy tự động chéo nhau).
3. Đổi tên/bỏ file one-off sau khi xây `tools/evaluate.py` thống nhất.

**Điểm đánh giá chung: 6/10** — validate_packet.py đạt chuẩn oracle; 2 tool C++ là bản nháp dev cần tham số hóa.
