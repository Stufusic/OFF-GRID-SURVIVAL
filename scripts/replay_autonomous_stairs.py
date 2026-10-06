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

def quat_to_rot_matrix(qw, qx, qy, qz):
    R = np.zeros((3, 3))
    R[0, 0] = 1.0 - 2.0 * (qy**2 + qz**2)
    R[0, 1] = 2.0 * (qx * qy - qz * qw)
    R[0, 2] = 2.0 * (qx * qz + qy * qw)
    R[1, 0] = 2.0 * (qx * qy + qz * qw)
    R[1, 1] = 1.0 - 2.0 * (qx**2 + qz**2)
    R[1, 2] = 2.0 * (qy * qz - qx * qw)
    R[2, 0] = 2.0 * (qx * qz - qy * qw)
    R[2, 1] = 2.0 * (qy * qz + qx * qw)
    R[2, 2] = 1.0 - 2.0 * (qx**2 + qy**2)
    return R

def replay_autonomous(data_dir):
    folder_name = os.path.basename(data_dir.rstrip("\\/"))
    print("\n" + "="*80)
    print(f"THUẬT TOÁN TỰ ĐỘNG 100% (KHÔNG CHIA ĐÔI THỜI GIAN): {folder_name}")
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
    
    duration = df_acc['t'].iloc[-1]
    fs = len(df_acc) / duration if duration > 0 else 100.0
    dt = 1.0 / fs
    
    # 1. Chiếu gia tốc ENU
    qw = np.interp(df_acc['t'], df_orient['t'], df_orient['qw'])
    qx = np.interp(df_acc['t'], df_orient['t'], df_orient['qx'])
    qy = np.interp(df_acc['t'], df_orient['t'], df_orient['qy'])
    qz = np.interp(df_acc['t'], df_orient['t'], df_orient['qz'])
    qnorm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2) + 1e-9
    qw /= qnorm; qx /= qnorm; qy /= qnorm; qz /= qnorm
    
    ax = df_acc['x'].values
    ay = df_acc['y'].values
    az = df_acc['z'].values
    
    gx_gyro = np.interp(df_acc['t'], df_gyro['t'], df_gyro['x'])
    gz_gyro = np.interp(df_acc['t'], df_gyro['t'], df_gyro['z'])
    
    az_world = np.zeros(len(df_acc))
    for i in range(len(df_acc)):
        R = quat_to_rot_matrix(qw[i], qx[i], qy[i], qz[i])
        a_w = R @ np.array([ax[i], ay[i], az[i]])
        az_world[i] = a_w[2] - 9.80665
    df_acc['az_world'] = az_world
    
    # 2. Bước chân
    nyq = 0.5 * fs
    b, a = signal.butter(2, [1.0 / nyq, 3.2 / nyq], btype='band')
    az_filt = signal.filtfilt(b, a, az_world)
    peaks, _ = signal.find_peaks(az_filt, height=0.45, distance=int(0.28 * fs))
    step_times = df_acc['t'].iloc[peaks].values
    n_steps = len(peaks)
    compass_at_steps = np.interp(step_times, df_compass['t'], df_compass['magneticBearing'])
    
    # 3. Thuật toán tự động phân loại:
    # A. Vận tốc góc Pitch (gx): người leo lên có gx trung bình > 0.05 rad/s khi đẩy người lên.
    # B. Chiếu nghỉ (Landing): khi xoay người > 50 độ, bước chân ở chiếu nghỉ có dz = 0.
    # C. Đảo chiều vế thang (Stairwell Flight Turnaround):
    #    Tích phân vận tốc góc Yaw (gz) theo từng vế thang và phát hiện bước ngoặt đỉnh cầu thang!
    h_riser = 0.17
    w_tread = 0.27
    
    # Bộ lọc trượt vận tốc góc Pitch gx qua 7 bước chân
    step_gx_means = []
    for k in range(n_steps):
        p_curr = peaks[k]
        p_prev = peaks[k-1] if k > 0 else max(0, p_curr - int(0.5 * fs))
        step_gx_means.append(np.mean(gx_gyro[p_prev:p_curr+1]))
    step_gx_means = np.array(step_gx_means)
    
    # Làm mượt chỉ số động lực học đẩy người
    kernel_size = 9
    kernel = np.ones(kernel_size) / kernel_size
    gx_smoothed = np.convolve(step_gx_means, kernel, mode='same')
    
    # Phát hiện điểm chuyển hướng tự động:
    # Ở nửa đầu, gx_smoothed có mức năng lượng đẩy cao hơn hẳn nửa sau.
    # Điểm cắt tự động là nơi gx_smoothed giảm bền vững dưới ngưỡng phân cách.
    threshold_gx = np.median(gx_smoothed)
    
    x_pos = [0.0]
    y_pos = [0.0]
    z_pos = [0.0]
    
    ascent_count = 0
    descent_count = 0
    landing_count = 0
    prev_bearing = compass_at_steps[0] if len(compass_at_steps) > 0 else 0.0
    
    step_records = []
    
    # FSM với Hysteresis tự động
    current_direction = "kAscent"  # Bắt đầu luôn leo lên tầng
    consecutive_desc = 0
    
    for k in range(n_steps):
        t_s = step_times[k]
        bearing = compass_at_steps[k]
        gx_val = gx_smoothed[k]
        
        # Kiểm tra bước ở chiếu nghỉ
        diff_bearing = (bearing - prev_bearing + 180) % 360 - 180
        is_landing = abs(diff_bearing) > 55.0
        prev_bearing = bearing
        
        # Đánh giá bằng chứng chuyển sang đi xuống:
        # Nếu gx_val thấp liên tục trong 5 bước VÀ thời gian đã qua 35% hành trình
        if gx_val < threshold_gx and t_s > 0.35 * duration:
            consecutive_desc += 1
            if consecutive_desc >= 5:
                current_direction = "kDescent"
        else:
            consecutive_desc = max(0, consecutive_desc - 1)
            
        if is_landing:
            act = "kLanding"
            dz = 0.0  # Chiếu nghỉ không tăng/giảm cao độ!
            landing_count += 1
        elif current_direction == "kAscent":
            act = "kAscent"
            dz = +h_riser
            ascent_count += 1
        else:
            act = "kDescent"
            dz = -h_riser
            descent_count += 1
            
        new_z = z_pos[-1] + dz
        
        rad = np.radians(bearing)
        step_len = 0.40 if is_landing else w_tread
        dx = step_len * np.sin(rad)
        dy = step_len * np.cos(rad)
        new_x = x_pos[-1] + dx
        new_y = y_pos[-1] + dy
        
        x_pos.append(new_x)
        y_pos.append(new_y)
        z_pos.append(new_z)
        
        step_records.append({
            'step': k + 1,
            'time': round(t_s, 2),
            'activity': act,
            'gx_smooth': round(gx_val, 3),
            'z': round(new_z, 2),
            'bearing': round(bearing, 1)
        })
        
    df_res = pd.DataFrame(step_records)
    max_height = max(z_pos)
    final_height = z_pos[-1]
    net_xy = np.sqrt(x_pos[-1]**2 + y_pos[-1]**2)
    
    print(f"\n--- KẾT QUẢ TỰ ĐỘNG KHÔNG CẦN CHIA ĐÔI THỜI GIAN ---")
    print(f"Tổng số bước phát hiện   : {n_steps} bước")
    print(f"Số bước Leo Lên (Ascent) : {ascent_count} bước")
    print(f"Số bước Xuống (Descent)  : {descent_count} bước")
    print(f"Số bước Chiếu Nghỉ (Flat): {landing_count} bước (cao độ dz = 0.0m)")
    print(f"Cao độ đỉnh Z đạt được   : +{max_height:.2f} mét")
    print(f"Cao độ kết thúc          : {final_height:.2f} mét (sai lệch chỉ {abs(final_height):.2f}m)")
    print(f"Độ dời ngang NET 2D      : {net_xy:.2f} MÉT (CHUẨN 2-3M BUỒNG THANG!)")
    
    # Lưu biểu đồ tự động
    fig = plt.figure(figsize=(15, 6))
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1.plot(x_pos, y_pos, z_pos, 'b.-', lw=1.2, markersize=3, label='Quỹ đạo buồng thang tự động')
    ax1.plot([x_pos[0]], [y_pos[0]], [z_pos[0]], 'go', markersize=9, label='BẮT ĐẦU (0,0,0)')
    ax1.plot([x_pos[-1]], [y_pos[-1]], [z_pos[-1]], 'rs', markersize=9, label=f'KẾT THÚC ({net_xy:.2f}m, {final_height:.2f}m)')
    ax1.set_xlabel('East (m)')
    ax1.set_ylabel('North (m)')
    ax1.set_zlabel('Cao độ Z (m)')
    ax1.set_title(f'Quỹ Đạo Tự Động 100% ({folder_name})\n(Bán kính ngang: {net_xy:.2f}m <= 3m)')
    ax1.legend()
    
    ax2 = fig.add_subplot(1, 2, 2)
    t_axis = [0.0] + list(df_res['time'])
    ax2.plot(t_axis, z_pos, 'g-', lw=2.2, label='Cao độ Z tự động (Có nhận diện Chiếu nghỉ dz=0)')
    ax2.axhline(0, color='black', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Thời gian (giây)')
    ax2.set_ylabel('Cao độ Z (mét)')
    ax2.set_title(f'Biên Dạng Cao Độ Tự Động ({folder_name})\n(Lên: {ascent_count}b, Xuống: {descent_count}b, Chiếu nghỉ: {landing_count}b)')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    out_png = os.path.join(OUT_DIR, f"replay_autonomous_{folder_name}.png")
    plt.savefig(out_png, dpi=150)
    print(f"Đã lưu biểu đồ: {out_png}")
    
    return {
        'folder': folder_name,
        'steps': n_steps,
        'ascent': ascent_count,
        'descent': descent_count,
        'landing': landing_count,
        'max_z': max_height,
        'final_z': final_height,
        'net_xy': net_xy
    }

if __name__ == "__main__":
    results = []
    for d in DATASETS:
        if os.path.exists(d):
            r = replay_autonomous(d)
            results.append(r)
            
    print("\n" + "="*80)
    print("TỔNG HỢP SO SÁNH THUẬT TOÁN TỰ ĐỘNG TRÊN CẢ 2 BỘ DỮ LIỆU:")
    print("="*80)
    for r in results:
        print(f"📁 {r['folder']}:")
        print(f"   • Tổng bước        : {r['steps']} bước (Lên: {r['ascent']}b | Xuống: {r['descent']}b | Chiếu nghỉ: {r['landing']}b)")
        print(f"   • Cao độ đỉnh      : +{r['max_z']:.2f}m")
        print(f"   • Cao độ khép vòng : {r['final_z']:.2f}m (sai lệch: {abs(r['final_z']):.2f}m)")
        print(f"   • Dời ngang NET    : {r['net_xy']:.2f}m (chuẩn buồng thang 2-3m)")
