# 12 BẤT BIẾN AN TOÀN TUYỆT ĐỐI (SAFETY INVARIANTS)

**HỆ THỐNG ĐỊNH VỊ SINH TỒN NGOẠI TUYẾN (OFF-GRID SURVIVAL ENGINE)**

---

Hệ thống định vị sinh tồn tuân thủ nghiêm ngặt 12 bất biến an toàn (Safety Invariants) dưới mọi điều kiện vận hành, kể cả khi gặp hỏng hóc cảm biến hoặc rơi vào tình huống nguy cấp:

### Invariant 1 — No Teleportation (Không dịch chuyển tức thời)
Một sự kiện ngắt GPS (GPS Cutoff) hoặc tái khởi động bộ ước lượng không bao giờ được phép làm thay đổi vị trí ước lượng tức thời $(\Delta E, \Delta N)$ vượt quá $3\sigma_{pos}$ so với vị trí ước lượng trước đó.

### Invariant 2 — No Null-Island Egress (Không phát tọa độ 0,0)
Gói tin cứu hộ 160-bit (BLE Beacon / Satellite) và tin nhắn SMS tuyệt đối không phát tọa độ Vĩ độ/Kinh độ $(0.0, 0.0)$ khi hệ thống đã có ít nhất một bản ghi `LastKnownGoodFix` hợp lệ.

### Invariant 3 — Core Platform Isolation (Độc lập nền tảng lõi)
Mã nguồn trong thư mục `core/` không được phép chứa bất kỳ Header hoặc lời gọi hàm trực tiếp nào của hệ điều hành cụ thể (`<windows.h>`, `<sys/mman.h>`, `<unistd.h>`). Mọi tương tác đĩa/bộ nhớ phải đi qua interface hoặc C++ Standard Library.

### Invariant 4 — Gated Sensor Ingestion (Cổng kiểm duyệt đo lường)
Không một mẫu cảm biến đơn lẻ nào được phép cưỡng bức thay đổi Navigation State nếu giá trị Innovation vượt quá ngưỡng kiểm định Chi-Square ($NIS = r^T \mathbf{S}^{-1} r > 9.0$).

### Invariant 5 — Monotonic Uncertainty on Dropout (Bảo toàn độ bất định khi mất tín hiệu)
Độ bất định vị trí (Hiệp phương sai $\mathbf{P}$, Bán trục lớn elip $CE95$, diện tích bất định $A_{95}$) là hàm không giảm theo thời gian khi xảy ra hiện tượng mất cảm biến (Sensor Dropout).

### Invariant 6 — Degraded Mode Fail-Safe (Chuyển chế độ an toàn khi suy giảm)
Khi điểm tin cậy tổng thể (GNS) hoặc chất lượng cảm biến rơi xuống dưới ngưỡng tới hạn, hệ thống bắt buộc chuyển sang chế độ `kDegraded` hoặc `kLost` thay vì tiếp tục giả định trạng thái chính xác.

### Invariant 7 — No False Confidence (Không báo điểm tin cậy ảo)
Điểm tin cậy $GNS$ không bao giờ được phép cao hơn chất lượng bằng chứng đo lường thực tế:
$$GNS \le \min\left(H_{\text{IMU}}, H_{\text{Mag}}, H_{\text{Baro}}, C_{\text{TRN}}, 1.0 - \frac{CE95}{100.0}\right)$$

### Invariant 8 — Covariance Validity (Bảo toàn tính hợp lệ hiệp phương sai)
Toàn bộ ma trận hiệp phương sai sai số $\mathbf{P}$ trong mọi chu trình tính toán phải luôn:
1. Đối xứng: $\|\mathbf{P} - \mathbf{P}^T\| < 10^{-6}$.
2. Xác định không âm: $\lambda_{\min}(\mathbf{P}) \ge 10^{-8}$.
3. Hữu hạn: Không chứa phần tử $NaN$ hoặc $\pm\infty$.

### Invariant 9 — Timestamp Monotonicity (Tính đơn điệu của nhãn thời gian)
Mọi mẫu đo đầu vào phải thỏa mãn $t_{i+1} > t_i$. Nhãn thời gian của cảm biến phần cứng là nguồn chân lý duy nhất, cấm gọi `std::chrono::now()` ở giữa luồng tính toán thuật toán.

### Invariant 10 — No NaN/Inf State (Tuyệt đối không có biến rác số học)
Toàn bộ các biến trạng thái, ma trận nội tại và dữ liệu đầu ra không bao giờ chứa giá trị $NaN$ hoặc $\pm\infty$. Mọi phép chia hoặc căn bậc hai đều phải có bộ bảo vệ epsilon ($\epsilon \ge 10^{-12}$).

### Invariant 11 — Estimator Freshness Watchdog (Giám sát chu kỳ tính toán)
Nếu khoảng thời gian kể từ lần cập nhật trạng thái gần nhất vượt quá $1.5\text{s}$, hệ thống lập tức kích hoạt cờ cảnh báo `ESTIMATOR_STALE` và hạ cấp chế độ định vị.

### Invariant 12 — Rescue Provenance & Staleness (Toàn vẹn nguồn gốc cứu hộ)
Mọi gói tin cứu hộ và tin nhắn SMS phát ra đều phải chứa rõ ràng: Nguồn gốc vị trí (`PositionSource`), Tuổi của mẫu đo (`Fix Age`), Độ bất định $CE95$ và Chế độ định vị (`NavigationMode`).
