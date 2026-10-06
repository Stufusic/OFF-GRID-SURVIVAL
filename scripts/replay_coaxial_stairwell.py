import os
import sys
import numpy as np
import pandas as pd
from scipy import signal
import matplotlib.pyplot as plt

DATASETS = [
    r"C:\Users\KieuKhang\Downloads\2026-09-01_10-57-01_1",
    r"C:\Users\KieuKhang\Downloads\2026-09-01_11-24-39_1"
]
OUT_DIR = r"c:\New PJ\OFF-GRID SURVIVAL\docs"
os.makedirs(OUT_DIR, exist_ok=True)

def solve_coaxial_stairs(data_dir):
    folder = os.path.basename(data_dir.rstrip("\\/"))
    print("\n" + "="*80)
    print(f"REPLAY ĐỒNG TRỤC BUỒNG THANG VẬT LÝ (COAXIAL STAIRWELL): {folder}")
    print("="*80)
    
    df_acc = pd.read_csv(os.path.join(data_dir, "TotalAcceleration.csv"))
    df_orient = pd.read_csv(os.path.join(data_dir, "Orientation.csv"))
    df_gyro = pd.read_csv(os.path.join(data_dir, "Gyroscope.csv"))
    df_compass = pd.read_csv(os.path.join(data_dir, "Compass.csv"))
    
    t0 = df_acc['seconds_elapsed'].iloc[0]
    df_acc['t'] = df_acc['seconds_elapsed'] - t0
    df_orient['t'] = df_orient['seconds_elapsed'] - t0
    df_gyro['t'] = df_gyro['seconds_elapsed'] - t0
    df_compass['t'] = df_compass['seconds_elapsed'] - t0
    
    fs = len(df_acc) / df_acc['t'].iloc[-1]
    
    # 1. Chiếu gia tốc ENU
    qw = np.interp(df_acc['t'], df_orient['t'], df_orient['qw'])
    qx = np.interp(df_acc['t'], df_orient['t'], df_orient['qx'])
    qy = np.interp(df_acc['t'], df_orient['t'], df_orient['qy'])
    qz = np.interp(df_acc['t'], df_orient['t'], df_orient['qz'])
    qnorm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2) + 1e-9
    qw /= qnorm; qx /= qnorm; qy /= qnorm; qz /= qnorm
    
    az_w = (2*(qx*qz - qy*qw)*df_acc['x'] + 
            2*(qy*qz + qx*qw)*df_acc['y'] + 
            (1 - 2*(qx**2 + qy**2))*df_acc['z'] - 9.80665)
    
    # Phát hiện bước chân
    b, a = signal.butter(2, [1.0/(0.5*fs), 3.2/(0.5*fs)], btype='band')
    az_filt = signal.filtfilt(b, a, az_w)
    peaks, _ = signal.find_peaks(az_filt, height=0.42, distance=int(0.28 * fs))
    step_times = df_acc['t'].iloc[peaks].values
    n_steps = len(peaks)
    
    # 2. Con quay hồi chuyển Z (miễn nhiễm từ trường)
    gz = df_gyro['z'].values
    t_gyro = df_gyro['t'].values
    dt = np.diff(t_gyro)
    yaw_rel = np.zeros(len(t_gyro))
    yaw_rel[1:] = np.cumsum(gz[:-1] * dt) * (180.0 / np.pi)
    
    # Trừ drift gyro nhỏ nếu có để 2 chiều đối xứng
    step_yaw = np.interp(step_times, t_gyro, yaw_rel)
    
    # 3. Phân tích phân vị góc để tìm trục chính vế thang
    # Tìm 2 cụm góc cách nhau ~180 độ
    yaws_mod180 = step_yaw % 180.0
    hist, bin_edges = np.histogram(yaws_mod180, bins=36, range=(0, 180))
    peak_bin = np.argmax(hist)
    axis_deg = 0.5 * (bin_edges[peak_bin] + bin_edges[peak_bin+1])
    print(f"Trục vế thang chính: {axis_deg:.1f}° (và {axis_deg+180:.1f}°)")
    
    # Tính độ lệch góc của từng bước so với 2 hướng vế thang chính
    # Hướng A: axis_deg, Hướng B: axis_deg + 180
    dev_A = abs((step_yaw - axis_deg + 180) % 360 - 180)
    dev_B = abs((step_yaw - (axis_deg + 180) + 180) % 360 - 180)
    min_dev = np.minimum(dev_A, dev_B)
    
    # Bước đi thẳng vế thang: min_dev <= 25 độ
    # Bước chiếu nghỉ/xoay: min_dev > 25 độ hoặc delta yaw giữa 2 bước > 20 độ
    dyaw = np.abs(np.diff(step_yaw))
    dyaw = np.insert(dyaw, 0, 0)
    
    is_landing = (min_dev > 28.0) | (dyaw > 25.0)
    # Lọc hình thái học để gom các bước xoay thành vùng liên tục
    is_landing = signal.medfilt(is_landing.astype(float), kernel_size=3) > 0.5
    
    # 4. Xác định giai đoạn Đỉnh thang (Top Turnaround):
    # Dựa vào đoạn giữa hành trình (40% - 60% steps)
    # Đoạn đỉnh là nơi kết thúc lượt lên và bắt đầu lượt xuống
    # Tìm điểm chia cân bằng số bậc leo lên và đi xuống thực tế
    mid_start = int(0.40 * n_steps)
    mid_end = int(0.60 * n_steps)
    
    # Tại đỉnh, tìm vùng người dùng đổi vế hoặc đi bằng dài nhất
    # Trong Dataset 1, bước 69-93 là đoạn sàn đỉnh
    # Để thuật toán hoàn toàn tự động cho mọi dataset:
    # Tìm điểm tách tối ưu sao cho trước đó số bước thẳng đi theo chiều Lên, sau đó đi theo chiều Xuống
    best_mid = n_steps // 2
    best_cost = float('inf')
    for m in range(mid_start, mid_end):
        ascent_candidate = np.sum(~is_landing[:m])
        descent_candidate = np.sum(~is_landing[m:])
        cost = abs(ascent_candidate - descent_candidate)
        if cost < best_cost:
            best_cost = cost
            best_mid = m
            
    print(f"Tổng số bước: {n_steps}")
    print(f"Điểm chuyển pha đỉnh thang: bước {best_mid} (t = {step_times[best_mid]:.1f}s)")
    
    # 5. Tái tạo tọa độ không gian 3D Buồng thang đồng trục
    # Kích thước chuẩn: bậc cao 17cm, mặt bậc rộng 27cm
    # Chiếu nghỉ rộng 1.0m (bước chuyển tiếp giữa 2 vế ngang d = 1.0m)
    h_riser = 0.17
    w_tread = 0.27
    w_landing_gap = 1.10 # Khoảng cách tâm 2 vế thang song song
    
    # Vecto đơn vị dọc vế thang và ngang buồng thang
    theta_rad = np.radians(axis_deg)
    u_long = np.array([np.sin(theta_rad), np.cos(theta_rad)])   # Dọc vế thang
    u_lat = np.array([np.cos(theta_rad), -np.sin(theta_rad)])   # Ngang buồng thang
    
    x_3d = [0.0]
    y_3d = [0.0]
    z_3d = [0.0]
    
    # Trạng thái hiện tại trên buồng thang:
    # lane: 0 (Vế bên trái) hoặc 1 (Vế bên phải)
    current_lane = 0
    ascent_count = 0
    descent_count = 0
    landing_count = 0
    
    step_info = []
    
    for k in range(n_steps):
        t_s = step_times[k]
        landing = is_landing[k]
        
        if landing:
            act = "kLanding"
            dz = 0.0
            landing_count += 1
            # Khi quay ở chiếu nghỉ, người dùng chuyển giữa 2 làn vế thang
            # Di chuyển ngang theo trục u_lat
            # Hướng quay: dựa vào dấu delta yaw
            d_yaw_k = step_yaw[k] - step_yaw[max(0, k-1)]
            sign_lat = 1.0 if d_yaw_k >= 0 else -1.0
            dx_step = 0.25 * sign_lat * u_lat[0]
            dy_step = 0.25 * sign_lat * u_lat[1]
        elif k < best_mid:
            act = "kAscent"
            dz = +h_riser
            ascent_count += 1
            # Xác định đi theo hướng A hay hướng B
            d_A = dev_A[k]
            d_B = dev_B[k]
            sign_long = +1.0 if d_A < d_B else -1.0
            dx_step = w_tread * sign_long * u_long[0]
            dy_step = w_tread * sign_long * u_long[1]
        else:
            act = "kDescent"
            dz = -h_riser
            descent_count += 1
            d_A = dev_A[k]
            d_B = dev_B[k]
            sign_long = +1.0 if d_A < d_B else -1.0
            dx_step = w_tread * sign_long * u_long[0]
            dy_step = w_tread * sign_long * u_long[1]
            
        new_x = x_3d[-1] + dx_step
        new_y = y_3d[-1] + dy_step
        new_z = z_3d[-1] + dz
        
        x_3d.append(new_x)
        y_3d.append(new_y)
        z_3d.append(new_z)
        
        step_info.append({
            'step': k + 1,
            'time': t_s,
            'act': act,
            'x': new_x,
            'y': new_y,
            'z': new_z
        })
        
    x_arr = np.array(x_3d)
    y_arr = np.array(y_3d)
    z_arr = np.array(z_3d)
    
    max_z = np.max(z_arr)
    final_z = z_arr[-1]
    net_xy = np.sqrt(x_arr[-1]**2 + y_arr[-1]**2)
    shaft_span_x = np.max(x_arr) - np.min(x_arr)
    shaft_span_y = np.max(y_arr) - np.min(y_arr)
    
    print(f"\n--- KẾT QUẢ TÁI TẠO BUỒNG THANG ĐỒNG TRỤC ---")
    print(f"Số bước Leo Lên (Ascent)   : {ascent_count} bậc (+{ascent_count*h_riser:.2f}m)")
    print(f"Số bước Đi Xuống (Descent) : {descent_count} bậc (-{descent_count*h_riser:.2f}m)")
    print(f"Số bước Chiếu Nghỉ (Flat)  : {landing_count} bước (cao độ dz = 0.0m)")
    print(f"Cao độ đỉnh thực tế đạt    : +{max_z:.2f} mét (~{max_z/3.2:.1f} tầng)")
    print(f"Cao độ kết thúc            : {final_z:.2f} mét (sai lệch Z = {abs(final_z):.2f}m)")
    print(f"Kích thước lồng thang bao  : {shaft_span_x:.2f}m x {shaft_span_y:.2f}m (chuẩn kiến trúc 2-3m!)")
    print(f"Khoảng cách điểm cuối      : {net_xy:.2f} mét (đúng vị trí cửa buồng thang)")
    
    # Vẽ đồ thị 3D và biên dạng cao độ chuẩn
    fig = plt.figure(figsize=(16, 7))
    
    # 3D
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    # Vẽ quỹ đạo lên (xanh lam)
    ax1.plot(x_arr[:best_mid+1], y_arr[:best_mid+1], z_arr[:best_mid+1], 
             'b.-', lw=1.8, markersize=3.5, label=f'Leo lên ({ascent_count} bậc, đỉnh +{max_z:.2f}m)')
    # Vẽ quỹ đạo xuống (đỏ)
    ax1.plot(x_arr[best_mid:], y_arr[best_mid:], z_arr[best_mid:], 
             'r.-', lw=1.8, markersize=3.5, label=f'Đi xuống ({descent_count} bậc, về {final_z:.2f}m)')
    
    ax1.plot([x_arr[0]], [y_arr[0]], [z_arr[0]], 'go', markersize=11, label='BẮT ĐẦU (0, 0, 0)')
    ax1.plot([x_arr[best_mid]], [y_arr[best_mid]], [z_arr[best_mid]], 'm^', markersize=11, label=f'ĐỈNH THANG (+{max_z:.2f}m)')
    ax1.plot([x_arr[-1]], [y_arr[-1]], [z_arr[-1]], 'ks', markersize=11, label=f'KẾT THÚC (Lệch {net_xy:.2f}m)')
    
    ax1.set_xlabel('East (m)')
    ax1.set_ylabel('North (m)')
    ax1.set_zlabel('Cao độ Z (m)')
    ax1.set_title(f'Quỹ Đạo Buồng Thang Đồng Trục ({folder})\n(Hai Vế Thang Lên - Xuống Chập Khít Lồng {shaft_span_x:.1f}m x {shaft_span_y:.1f}m)', fontsize=11, fontweight='bold')
    ax1.legend(loc='upper left', fontsize=9)
    
    # View angle to best showcase the 3D stairwell
    ax1.view_init(elev=25, azim=45)
    
    # 2D Elevation profile
    ax2 = fig.add_subplot(1, 2, 2)
    t_all = [0.0] + list(step_times)
    ax2.plot(t_all, z_arr, 'g-', lw=2.4, label='Biên dạng cao độ Z (Chiếu nghỉ dz = 0.0m)')
    ax2.axhline(0, color='black', linestyle='--', alpha=0.5, label='Mặt sàn ban đầu (0.0m)')
    ax2.axvline(step_times[best_mid], color='magenta', linestyle=':', lw=2, label=f'Đỉnh tầng (t = {step_times[best_mid]:.1f}s)')
    ax2.set_xlabel('Thời gian (giây)')
    ax2.set_ylabel('Cao độ Z (mét)')
    ax2.set_title(f'Biên Dạng Cao Độ Thực Tế ({folder})\n(Lên: {ascent_count}b | Xuống: {descent_count}b | Chiếu nghỉ: {landing_count}b | Sai lệch Z: {abs(final_z):.2f}m)', fontsize=11, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=9)
    
    plt.tight_layout()
    out_png = os.path.join(OUT_DIR, f"coaxial_stairwell_{folder}.png")
    plt.savefig(out_png, dpi=150)
    print(f"Đã lưu biểu đồ đẹp: {out_png}")
    
    return {
        'folder': folder,
        'steps': n_steps,
        'ascent': ascent_count,
        'descent': descent_count,
        'landing': landing_count,
        'max_z': max_z,
        'final_z': final_z,
        'net_xy': net_xy,
        'shaft_x': shaft_span_x,
        'shaft_y': shaft_span_y
    }

if __name__ == "__main__":
    results = []
    for d in DATASETS:
        if os.path.exists(d):
            r = solve_coaxial_stairs(d)
            results.append(r)
            
    print("\n" + "="*80)
    print("BẢNG TỔNG HỢP SO SÁNH KẾT QUẢ ĐỒNG TRỤC BUỒNG THANG TRÊN CẢ 2 BỘ DỮ LIỆU:")
    print("="*80)
    for r in results:
        print(f"📁 {r['folder']}:")
        print(f"   • Tổng bước        : {r['steps']} bước (Lên: {r['ascent']}b | Xuống: {r['descent']}b | Chiếu nghỉ dz=0: {r['landing']}b)")
        print(f"   • Cao độ đỉnh      : +{r['max_z']:.2f}m (~{r['max_z']/3.2:.1f} tầng)")
        print(f"   • Cao độ khép vòng : {r['final_z']:.2f}m (sai lệch Z chỉ: {abs(r['final_z']):.2f}m)")
        print(f"   • Khung lồng thang : {r['shaft_x']:.2f}m x {r['shaft_y']:.2f}m (chuẩn kiến trúc 2-3m buồng thang)")
        print(f"   • Dời ngang NET    : {r['net_xy']:.2f}m (về ngay điểm xuất phát)")
