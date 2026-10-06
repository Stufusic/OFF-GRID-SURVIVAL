# QUY ƯỚC HỆ QUY CHIẾU & TOÁN HỌC KHÔNG GIAN (FRAME CONVENTIONS)

**OFF-GRID SURVIVAL ENGINE - SAFETY-CRITICAL SPECIFICATION**

---

## I. HỆ QUY CHIẾU ĐẠO HÀNG (NAVIGATION FRAME - ENU)

Hệ quy chiếu đạo hàng cục bộ là hệ tọa độ phẳng tiếp xúc bề mặt Trái Đất (Local Tangent Plane) tuân theo chuẩn **ENU (East-North-Up)**:
- **Trục $+X$ ($\mathbf{e}_E$):** Hướng Đông (East - Kinh tuyến tăng dần theo hướng Đông).
- **Trục $+Y$ ($\mathbf{e}_N$):** Hướng Bắc (North - Vĩ tuyến tăng dần theo hướng Bắc cực địa lý).
- **Trục $+Z$ ($\mathbf{e}_U$):** Hướng Thiên đỉnh (Up - Vuông góc mặt elipsoid WGS-84 hướng ra xa tâm Trái Đất).

---

## II. HỆ QUY CHIẾU THÂN THIẾT BỊ (BODY FRAME)

Được gắn cố định với vỏ phần cứng điện thoại / thiết bị đeo:
- **Trục $+X$ ($\mathbf{b}_X$):** Hướng sang Phải thiết bị (Right side of screen).
- **Trục $+Y$ ($\mathbf{b}_Y$):** Hướng Tiến / Đỉnh thiết bị (Top edge / Forward).
- **Trục $+Z$ ($\mathbf{b}_Z$):** Hướng Ra ngoài mặt màn hình (Out of screen / Upward).

Khi thiết bị đặt phẳng nằm ngang trên mặt bàn, màn hình ngửa lên trời và đỉnh máy quay về hướng Bắc thực:
$$\mathbf{b}_X \parallel \mathbf{e}_E, \quad \mathbf{b}_Y \parallel \mathbf{e}_N, \quad \mathbf{b}_Z \parallel \mathbf{e}_U$$

---

## III. QUATERNION ĐỊNH HƯỚNG & MA TRẬN QUAY DCM

1. **Quaternion Thái độ $\mathbf{q}_{nb}$:**
   - Biểu diễn phép quay biến đổi một vectơ từ **Body Frame sang Navigation Frame (Body $\to$ Nav)**:
     $$\mathbf{v}_{nav} = \mathbf{q}_{nb} \otimes \mathbf{v}_{body} \otimes \mathbf{q}_{nb}^*$$
   - Dạng vector: $\mathbf{q} = [w, x, y, z]^T$, với $w = \cos(\theta/2)$ là phần thực, $[x, y, z]^T = \mathbf{u} \sin(\theta/2)$ là phần ảo.

2. **Ma trận Xoay Cosine Chỉ hướng (Direction Cosine Matrix $\mathbf{R}_{nb}$):**
   $$\mathbf{v}_{nav} = \mathbf{R}_{nb} \cdot \mathbf{v}_{body}$$
   $$\mathbf{R}_{nb} = \begin{bmatrix}
   1 - 2(y^2+z^2) & 2(xy - wz) & 2(xz + wy) \\
   2(xy + wz) & 1 - 2(x^2+z^2) & 2(yz - wx) \\
   2(xz - wy) & 2(yz + wx) & 1 - 2(x^2+y^2)
   \end{bmatrix}$$

3. **Góc Phương vị La bàn (Compass Heading $\psi$):**
   - Góc quay quanh trục $+Z$ (Up), tính từ hướng Bắc ($+Y$) theo **chiều kim đồng hồ** về hướng Đông ($+X$):
     $$\psi = \text{atan2}(R_{nb}(0,1), R_{nb}(1,1)) = \text{atan2}(2(xy - wz), 1 - 2(x^2 + z^2))$$
   - Chuẩn hóa: $\psi \in [0, 2\pi)$ hoặc $[-\pi, \pi]$.

4. **Multiplicative Error-State Quaternion Injection:**
   - Trạng thái danh định $\mathbf{q}^-$ và sai số $\delta\boldsymbol{\theta} \in \mathbb{R}^3$:
     $$\delta\mathbf{q} \approx \begin{bmatrix} 1 \\ \frac{1}{2}\delta\theta_x \\ \frac{1}{2}\delta\theta_y \\ \frac{1}{2}\delta\theta_z \end{bmatrix}$$
   - Cập nhật sau bước lọc Kalman:
     $$\mathbf{q}^+ = \mathbf{q}^- \otimes \delta\mathbf{q}$$

5. **Góc Euler vật lý (AttitudeEstimate.euler) — Quy ước bắt buộc (v2.3.1):**
   - Với frame BODY ở mục II (+X = Phải, +Y = Trước, +Z = Lên), ngữ nghĩa euler là ngữ nghĩa **vật lý của người dùng**, KHÔNG phải tham số aerospace ZYX:
     - `roll_rad` — nghiêng **ngang** (bank quanh trục trước $+Y$): $\phi = \text{atan2}(-R_{nb}(2,0),\ R_{nb}(2,2)) = \text{atan2}(2(wy - xz),\ 1 - 2(x^2+y^2))$. **Dương = nghiêng phải.**
     - `pitch_rad` — đổ **về trước** (quanh trục phải $+X$): $\theta = -\arcsin(R_{nb}(2,1)) = -\arcsin(2(yz + wx))$. **Dương = trục trước chúc xuống (thân đổ về trước).**
   - Lý do dấu: bank phải góc $\alpha$ (quay quanh $+Y$) làm $R(2,0) = -\sin\alpha$; đổ trước góc $\theta$ (quay quanh $+X$ chiều âm) làm $R(2,1) = -\sin\theta$.
   - **Cảnh báo:** `eulerToQuaternion(roll, pitch, yaw)` trong `frame_conventions.hpp` dùng tham số aerospace ZYX (roll về trục X, pitch về trục Y) — không dùng nó để tái tạo $\mathbf{q}_{nb}$ từ `AttitudeEstimate.euler`.
   - Audit H1 (09/2026): công thức cũ `asin(2(wy − zx))` là độ cao của trục PHẢI (+X) = nghiêng ngang, khiến mọi gate pitch trong `MotionActivityClassifier` khóa nhầm tín hiệu.
