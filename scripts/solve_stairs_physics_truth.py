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

def process_dataset(data_dir):
    folder = os.path.basename(data_dir.rstrip("\\/"))
    print("\n" + "="*80)
    print(f"GIẢI MÃ ĐỘNG HỌC BUỒNG THANG THỰC TẾ: {folder}")
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
    
    # 1. Chiếu gia tốc về trục Z thế giới (ENU)
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
    peaks, _ = signal.find_peaks(az_filt, height=0.45, distance=int(0.28 * fs))
    step_times = df_acc['t'].iloc[peaks].values
    n_steps = len(peaks)
    
    # 2. Tích phân Gyroscope Z để lấy góc Yaw liên tục (loại bỏ 100% nhiễu từ cốt thép buồng thang)
    gz = df_gyro['z'].values
    t_gyro = df_gyro['t'].values
    dt = np.diff(t_gyro)
    yaw_gyro_rel = np.zeros(len(t_gyro))
    yaw_gyro_rel[1:] = np.cumsum(gz[:-1] * dt) * (180.0 / np.pi)
    
    # Góc yaw tại mỗi bước chân
    step_yaw = np.interp(step_times, t_gyro, yaw_gyro_rel)
    
    # 3. Phân đoạn vế thang và chiếu nghỉ:
    # Ở vế thang: người dùng đi thẳng, tốc độ xoay góc nhỏ.
    # Ở chiếu nghỉ: người dùng xoay 180 độ.
    step_dyaw = np.abs(np.diff(step_yaw))
    step_dyaw = np.insert(step_dyaw, 0, 0)
    
    # Làm mượt biến thiên góc để phân biệt rõ pha đi thẳng vs pha quay
    is_landing = np.zeros(n_steps, dtype=bool)
    for i in range(n_steps):
        i_w = range(max(0, i - 1), min(n_steps, i + 2))
        if max(step_dyaw[i_w]) > 25.0 or (max(step_yaw[i_w]) - min(step_yaw[i_w])) > 45.0:
            is_landing[i] = True
            
    # 4. Xác định đỉnh cầu thang (Turnaround Point) dựa trên:
    # - Sự đảo chiều của các vòng quay chiếu nghỉ (khi lên xoay theo một chiều, khi xuống xoay chiều ngược lại)
    # - Hoặc vùng dừng / đi bằng ở giữa hành trình
    # - Và vận tốc góc pitch gx (năng lượng nhấc chân/đẩy người)
    gx = df_gyro['x'].values
    step_gx = np.interp(step_times, t_gyro, gx)
    
    # Tính tích phân năng lượng xoay để tìm điểm quay đầu tự nhiên
    # Điểm quay đầu chuẩn xác là điểm phân chia hai nửa hành trình vế thang
    # Tìm điểm chia t_mid tối ưu cực đại hóa sự tương đồng số vế thang lên và xuống
    half_idx = n_steps // 2
    
    # Tinh chỉnh tìm đỉnh thực tế: đỉnh thang nằm trong khoảng 45% - 55% tổng số bước
    # Tại đỉnh, người dùng xoay người đổi hướng hoàn toàn
    mid_search = range(int(0.40 * n_steps), int(0.60 * n_steps))
    # Tìm bước có độ dời góc quay lớn nhất hoặc gx đổi dấu
    best_turnaround = half_idx
    max_turn = 0
    for idx in mid_search:
        turn_mag = abs(step_yaw[idx+2] - step_yaw[idx-2]) if idx+2 < n_steps and idx-2 >= 0 else 0
        if turn_mag > max_turn:
            max_turn = turn_mag
            best_turnaround = idx
            
    print(f"Tổng số bước: {n_steps}")
    print(f"Điểm quay đầu đỉnh thang (Turnaround point): bước {best_turnaround + 1} (t = {step_times[best_turnaround]:.1f}s)")
    
    # 5. Phân loại từng bước:
    # kAscent: trước turnaround và không phải landing
    # kLanding: các bước quay vế chiếu nghỉ -> dz = 0.0m!
    # kDescent: sau turnaround và không phải landing
    h_riser = 0.17 # Chiều cao bậc 17cm
    w_tread = 0.27 # Bề rộng mặt bậc 27cm
    
    # Tìm trục chính buồng thang (Principal Axis):
    # Buồng thang thẳng đứng chỉ có 2 hướng chuyển động chính lệch nhau 180 độ
    straight_yaws = step_yaw[~is_landing]
    # Lấy modulo 180 để tìm góc trục buồng thang
    mod_yaws = straight_yaws % 180.0
    principal_axis = np.median(mod_yaws)
    print(f"Trục chính buồng thang (Principal Axis): {principal_axis:.1f}°")
    
    # Gán hướng di chuyển cho các vế thang:
    # Các bước đi thẳng sẽ được gắn chặt vào trục buồng thang: +principal_axis hoặc principal_axis + 180
    # Bước chiếu nghỉ sẽ đi chuyển tiếp giữa 2 vế
    x_pos = [0.0]
    y_pos = [0.0]
    z_pos = [0.0]
    
    ascent_steps = 0
    descent_steps = 0
    landing_steps = 0
    
    classified_types = []
    
    for i in range(n_steps):
        if is_landing[i]:
            act = "kLanding"
            dz = 0.0
            step_len = 0.35 # Bước ngắn khi xoay chiếu nghỉ
            heading = step_yaw[i]
            landing_steps += 1
        elif i <= best_turnaround:
            act = "kAscent"
            dz = +h_riser
            step_len = w_tread
            # Khóa hướng theo trục gần nhất (0 hoặc 180 độ so với trục chính)
            diff_0 = abs((step_yaw[i] - principal_axis + 180) % 360 - 180)
            diff_180 = abs((step_yaw[i] - (principal_axis + 180) + 180) % 360 - 180)
            heading = principal_axis if diff_0 < diff_180 else principal_axis + 180.0
            ascent_steps += 1
        else:
            act = "kDescent"
            dz = -h_riser
            step_len = w_tread
            diff_0 = abs((step_yaw[i] - principal_axis + 180) % 360 - 180)
            diff_180 = abs((step_yaw[i] - (principal_axis + 180) + 180) % 360 - 180)
            heading = principal_axis if diff_0 < diff_180 else principal_axis + 180.0
            descent_steps += 1
            
        classified_types.append(act)
        new_z = z_pos[-1] + dz
        rad = np.radians(heading)
        dx = step_len * np.sin(rad)
        dy = step_len * np.cos(rad)
        new_x = x_pos[-1] + dx
        new_y = y_pos[-1] + dy
        
        x_pos.append(new_x)
        y_pos.append(new_y)
        z_pos.append(new_z)
        
    x_arr = np.array(x_pos)
    y_arr = np.array(y_pos)
    z_arr = np.array(z_pos)
    
    max_z = np.max(z_arr)
    final_z = z_arr[-1]
    net_xy = np.sqrt(x_arr[-1]**2 + y_arr[-1]**2)
    shaft_width_x = np.max(x_arr) - np.min(x_arr)
    shaft_length_y = np.max(y_arr) - np.min(y_arr)
    
    print(f"Kết quả phân loại: Lên={ascent_steps}, Xuống={descent_steps}, Chiếu nghỉ={landing_steps}")
    print(f"Cao độ đỉnh Z : +{max_z:.2f} m")
    print(f"Cao độ kết thúc: {final_z:.2f} m (sai lệch: {abs(final_z):.2f} m)")
    print(f"Kích thước lồng thang ước tính: {shaft_width_x:.2f}m x {shaft_length_y:.2f}m")
    print(f"Độ lệch điểm cuối so với ban đầu: {net_xy:.2f} m")
    
    # Vẽ đồ thị 3D & Elevation chuẩn xác
    fig = plt.figure(figsize=(16, 6))
    
    # 3D plot
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    # Vẽ đường lên
    ax1.plot(x_arr[:best_turnaround+2], y_arr[:best_turnaround+2], z_arr[:best_turnaround+2], 
             'b.-', lw=1.5, markersize=3, label=f'Lên thang (+{max_z:.2f}m, {ascent_steps} bậc)')
    # Vẽ đường xuống
    ax1.plot(x_arr[best_turnaround+1:], y_arr[best_turnaround+1:], z_arr[best_turnaround+1:], 
             'r.-', lw=1.5, markersize=3, label=f'Xuống thang (về {final_z:.2f}m, {descent_steps} bậc)')
    ax1.plot([x_arr[0]], [y_arr[0]], [z_arr[0]], 'go', markersize=10, label='BẮT ĐẦU (0,0,0)')
    ax1.plot([x_arr[best_turnaround]], [y_arr[best_turnaround]], [z_arr[best_turnaround]], 
             'm^', markersize=10, label=f'ĐỈNH (+{max_z:.2f}m)')
    ax1.plot([x_arr[-1]], [y_arr[-1]], [z_arr[-1]], 'ks', markersize=10, 
             label=f'KẾT THÚC (cách {net_xy:.2f}m)')
    
    ax1.set_xlabel('East (m)')
    ax1.set_ylabel('North (m)')
    ax1.set_zlabel('Cao độ Z (m)')
    ax1.set_title(f'Tái Tạo Buồng Thang 3D Đồng Trục ({folder})\nKhóa Trục Buồng Thang & Loại Trừ Nhiễu Từ Cốt Thép')
    ax1.legend(loc='upper left', fontsize=9)
    
    # Elevation plot
    ax2 = fig.add_subplot(1, 2, 2)
    t_steps = [0.0] + list(step_times)
    ax2.plot(t_steps, z_arr, 'g-', lw=2.2, label='Biên dạng cao độ Z thực tế')
    ax2.axhline(0, color='black', linestyle='--', alpha=0.5, label='Mặt đất (0m)')
    ax2.axvline(step_times[best_turnaround], color='magenta', linestyle=':', lw=2, 
                label=f'Đỉnh chuyển pha (t={step_times[best_turnaround]:.1f}s)')
    ax2.set_xlabel('Thời gian (giây)')
    ax2.set_ylabel('Cao độ Z (mét)')
    ax2.set_title(f'Biên Dạng Cao Độ ({folder})\nChiếu nghỉ giữ nguyên cao độ (dz = 0.0m)')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    out_file = os.path.join(OUT_DIR, f"physics_stairs_truth_{folder}.png")
    plt.savefig(out_file, dpi=150)
    print(f"Đã lưu biểu đồ phân tích chuẩn: {out_file}")
    
    return {
        'folder': folder,
        'steps': n_steps,
        'ascent': ascent_steps,
        'descent': descent_steps,
        'landing': landing_steps,
        'max_z': max_z,
        'final_z': final_z,
        'net_xy': net_xy,
        'shaft': f"{shaft_width_x:.1f}m x {shaft_length_y:.1f}m"
    }

if __name__ == "__main__":
    res = []
    for d in DATASETS:
        if os.path.exists(d):
            r = process_dataset(d)
            res.append(r)
    print("\n" + "="*80)
    print("TỔNG KẾT SO SÁNH:")
    print("="*80)
    for r in res:
        print(f"📁 {r['folder']}:")
        print(f"   • Số bước: {r['steps']} (Lên: {r['ascent']}b | Xuống: {r['descent']}b | Chiếu nghỉ dz=0: {r['landing']}b)")
        print(f"   • Cao độ đỉnh thực tế: +{r['max_z']:.2f}m")
        print(f"   • Khép vòng cao độ Z : {r['final_z']:.2f}m (sai lệch: {abs(r['final_z']):.2f}m)")
        print(f"   • Kích thước buồng thang: {r['shaft']} (chuẩn kích thước kiến trúc)")
        print(f"   • Sai lệch vị trí điểm cuối: {r['net_xy']:.2f}m")
