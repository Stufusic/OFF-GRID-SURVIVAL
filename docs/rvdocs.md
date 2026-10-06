# REVIEW — THƯ MỤC `docs/` (TÀI LIỆU KỸ THUẬT)

**Ngày review:** 06/09/2026 | **Phạm vi:** toàn bộ tài liệu markdown + artifact trong `docs/`

---

## 1. DANH MỤC HIỆN TRẠNG

| Tài liệu | Vai trò | Trạng thái |
|---|---|---|
| `Plan.md` | Kim chỉ nam kiến trúc + roadmap 5 phase | ✅ Còn nguyên giá trị; Phase 1–4 đã hoàn thành tương ứng |
| `Rule.md` | Quy chuẩn lập trình (C++20, zero-alloc, invariant) | ⚠️ Mục I.2 yêu cầu `-Werror`, I.3 yêu cầu `-fno-exceptions` — **CMake chưa enforce** (deviation đã ghi trong rvcore.md) |
| `frame_conventions.md` | Quy ước frame/quaternion | ✅ Đã cập nhật mục 5 (ngữ nghĩa euler vật lý + cảnh báo eulerToQuaternion) sau fix H1 |
| `safety_invariants.md` | 12 bất biến an toàn | ✅ Đối chiếu: I1,2,4,5,7,8,9,10,11,12 đã có fix + test; I3 grep-manual; I6 chưa test |
| `audit_report_v2.2_deep_dive_and_v2.3_roadmap.md` | Kiểm toán 12 lỗi + roadmap v2.3 | ✅ Baseline lịch sử — 12 lỗi đã xử lý xong (8 gốc + 4 hoàn thiện S1) |
| `comprehensive_system_audit_and_test_report.md` | Báo cáo kiểm thử tổng | ✅ Tham chiếu lịch sử |
| `EXPERIMENT_REPORTS/baseline_v232.md` | **Baseline đo lường chính thức v2.3.2** (mới) | ✅ 20 MC × 120 bước, NEES 12.56, hành trình metric |
| `Pipeline.md`, `SURVIVAL_NAV_AGENT_SKILLS_v2.2.md` | Mô tả pipeline/kỹ năng agent | Cần rà cập nhật sau S1–S3 (một mô tả hành vi đã thay đổi: bias QNH, declination, NO_FIX bit) |
| `rv.md`, `rv2.md`, `rv3.md`, `rv4.md` | Các bản review trước | Lịch sử — nên ghi chú "superseded" đầu file |
| `dataset1_steps_detailed.csv`, `flight_analysis_results.csv` | Artifact phân tích | ⚠️ Nên chuyển sang `export/` |

## 2. ĐIỂM MẠNH

- Văn hóa tài liệu **rất tốt cho dự án nghiên cứu**: mỗi quyết định lớn đều có bản ghi (Plan → Rule → audit → experiment report), đánh số phiên bản rõ.
- `frame_conventions.md` là ví dụ tốt: quy ước + công thức + cảnh báo bẫy (eulerToQuaternion aerospace vs euler vật lý).
- Baseline v2.3.2 có hành trình NEES 0.24 → 12.56 — chứng minh chuỗi đo lường được sửa như thế nào.

## 3. VẤN ĐỀ & KHUYẾN NGHỊ

1. **[MEDIUM] Tài liệu mô tả hành vi đã lỗi thời sau S1–S3**: `Pipeline.md` và `SURVIVAL_NAV_AGENT_SKILLS_v2.2.md` cần đợt rà một lượt — các mô tả về baro rate, stair FSM, governor, SMS payload đã đổi. Khuyến nghị: thêm mục "v2.3.2 changes" vào mỗi file.
2. **[LOW] `MATHEMATICAL_MODELS.md` được Plan.md tham chiếu nhưng không tồn tại** — hoặc viết (chứng minh ESKF/PF/Weinberg) hoặc xóa tham chiếu.
3. **[LOW] Phân kỳ Rule.md ↔ CMake**: `-Werror`/`-fno-exceptions` — hoặc enforce hoặc ghi chú "planned" trong Rule.md để tránh hiểu nhầm.
4. **[LOW] Truy vết rv1–4**: đặt tên không tự mô tả; khuyến nghị đổi tên thành `archive_review_2026-09-0X.md` hoặc thêm header ngày/phạm vi.
5. **[INFO] Thiếu tài liệu hợp đồng C API** (`survival_c_api.h` chỉ có comment) — với lớp consumed bởi Android sau này, nên có `docs/C_API_CONTRACT.md` liệt kê entry point, luồng thread, unit của từng trường.

## 4. KẾT LUẬN

**Điểm đánh giá chung: 7.5/10** — văn hóa ghi chép mạnh, tài liệu quy ước chuẩn; việc cần làm là đồng bộ hóa nội dung mô tả hành vi sau 3 sprint fix và bổ sung hợp đồng C API trước khi tích hợp mobile.
