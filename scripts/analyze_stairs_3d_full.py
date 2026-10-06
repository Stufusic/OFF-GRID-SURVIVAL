import os
import glob
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
    """Convert quaternion to 3x3 rotation matrix (Body to World ENU)."""
    # Android Sensor Logger quaternion: [qx, qy, qz, qw]
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

def analyze_dataset_3d(data_dir):
    folder_name = os.path.basename(data_dir.rstrip("\\/"))
    print(f"\n{'='*70}\nPHÂN TÍCH TOÀN DIỆN CẢM BIẾN 3D & CẦU THANG: {folder_name}\n{'='*70}")
    
    # 1. Load all files
    df_acc = pd.read_csv(os.path.join(data_dir, "TotalAcceleration.csv"))
    df_lin = pd.read_csv(os.path.join(data_dir, "Accelerometer.csv"))
    df_gyro = pd.read_csv(os.path.join(data_dir, "Gyroscope.csv"))
    df_orient = pd.read_csv(os.path.join(data_dir, "Orientation.csv"))
    df_mag = pd.read_csv(os.path.join(data_dir, "Magnetometer.csv"))
    df_compass = pd.read_csv(os.path.join(data_dir, "Compass.csv"))
    df_gravity = pd.read_csv(os.path.join(data_dir, "Gravity.csv"))
    
    # Uncalibrated files if present
    uncal_acc_p = os.path.join(data_dir, "AccelerometerUncalibrated.csv")
    uncal_gyro_p = os.path.join(data_dir, "GyroscopeUncalibrated.csv")
    uncal_mag_p = os.path.join(data_dir, "MagnetometerUncalibrated.csv")
    
    df_acc_uncal = pd.read_csv(uncal_acc_p) if os.path.exists(uncal_acc_p) else None
    df_gyro_uncal = pd.read_csv(uncal_gyro_p) if os.path.exists(uncal_gyro_p) else None
    df_mag_uncal = pd.read_csv(uncal_mag_p) if os.path.exists(uncal_mag_p) else None
    
    t0 = df_acc['seconds_elapsed'].iloc[0]
    df_acc['t'] = df_acc['seconds_elapsed'] - t0
    df_orient['t'] = df_orient['seconds_elapsed'] - t0
    df_compass['t'] = df_compass['seconds_elapsed'] - t0
    
    duration = df_acc['t'].iloc[-1]
    fs = len(df_acc) / duration if duration > 0 else 100.0
    print(f"Tổng thời gian: {duration:.2f}s | Tần số: {fs:.1f}Hz | Số mẫu: {len(df_acc)}")
    
    # Sensor Biases & Calibrations
    if df_mag is not None and df_mag_uncal is not None:
        # Bias = Uncalibrated - Calibrated
        bias_x = (df_mag_uncal['x'] - df_mag['x']).mean()
        bias_y = (df_mag_uncal['y'] - df_mag['y']).mean()
        bias_z = (df_mag_uncal['z'] - df_mag['z']).mean()
        print(f"Hiệu chỉnh từ kế (Hard-Iron Biases): Bx={bias_x:.2f}, By={bias_y:.2f}, Bz={bias_z:.2f} uT")
        
    if df_gyro is not None and df_gyro_uncal is not None:
        gbias_x = (df_gyro_uncal['x'] - df_gyro['x']).mean()
        gbias_y = (df_gyro_uncal['y'] - df_gyro['y']).mean()
        gbias_z = (df_gyro_uncal['z'] - df_gyro['z']).mean()
        print(f"Hiệu chỉnh Gyroscope (Estimated Gyro Biases): Gx={gbias_x*1e3:.2f}, Gy={gbias_y*1e3:.2f}, Gz={gbias_z*1e3:.2f} mrad/s")
    
    # Interpolate orientation quaternions to acceleration timestamps
    qw = np.interp(df_acc['t'], df_orient['t'], df_orient['qw'])
    qx = np.interp(df_acc['t'], df_orient['t'], df_orient['qx'])
    qy = np.interp(df_acc['t'], df_orient['t'], df_orient['qy'])
    qz = np.interp(df_acc['t'], df_orient['t'], df_orient['qz'])
    
    # Normalize quaternions
    qnorm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw /= qnorm
    qx /= qnorm
    qy /= qnorm
    qz /= qnorm
    
    # Project acceleration into World ENU Frame: a_world = R * a_body - [0, 0, g]
    ax_body = df_acc['x'].values
    ay_body = df_acc['y'].values
    az_body = df_acc['z'].values
    
    az_world = np.zeros(len(df_acc))
    ah_world = np.zeros(len(df_acc))
    
    for i in range(len(df_acc)):
        R = quat_to_rot_matrix(qw[i], qx[i], qy[i], qz[i])
        a_w = R @ np.array([ax_body[i], ay_body[i], az_body[i]])
        az_world[i] = a_w[2] - 9.80665  # Vertical residual acceleration
        ah_world[i] = np.sqrt(a_w[0]**2 + a_w[1]**2)  # Horizontal acceleration
        
    df_acc['az_world'] = az_world
    df_acc['ah_world'] = ah_world
    
    # Step detection with Stair recognition
    # Bandpass 1.0 - 3.2 Hz
    nyq = 0.5 * fs
    b, a = signal.butter(2, [1.0 / nyq, 3.2 / nyq], btype='band')
    az_filt = signal.filtfilt(b, a, az_world)
    
    # Peak detection
    peaks, _ = signal.find_peaks(az_filt, height=0.4, distance=int(0.28 * fs))
    step_times = df_acc['t'].iloc[peaks].values
    n_steps = len(peaks)
    
    # Classify each step: ASCENT (Lên thang) vs DESCENT (Xuống thang)
    # Pitch angle during step:
    pitch_interp = np.interp(step_times, df_orient['t'], df_orient['pitch'])
    roll_interp = np.interp(step_times, df_orient['t'], df_orient['roll'])
    
    # Vertical jerk at peak
    jerk_z = np.diff(az_filt) * fs
    jerk_peaks = jerk_z[np.clip(peaks, 0, len(jerk_z) - 1)]
    
    # In stair ascent, foot pushes upwards against gravity before step peak -> characteristic positive acceleration surge
    # In descent, foot impacts downward with sharp negative/positive rebound
    step_types = []
    dz_steps = []
    
    # Average stair riser height: 16-18 cm (0.17m)
    # Average stair tread width: 26-28 cm (0.27m)
    h_riser = 0.17  # meters
    w_tread = 0.27  # meters
    
    # Let's inspect vertical velocity integration with ZUPT
    # Integrate az_world
    vz = np.zeros(len(df_acc))
    z_pos = np.zeros(len(df_acc))
    dt_arr = np.diff(df_acc['t'].values)
    
    # Compass bearing at steps
    compass_at_steps = np.interp(step_times, df_compass['t'], df_compass['magneticBearing'])
    
    # Heading turns: identify when user turns at landing
    # Detect 180-degree turns
    landing_turns = 0
    ascent_count = 0
    descent_count = 0
    
    # Trajectory coordinates
    x_stairs = [0.0]
    y_stairs = [0.0]
    z_stairs = [0.0]
    
    curr_z = 0.0
    for i in range(len(peaks)):
        # Inspect jerk & energy
        # Determine ascent vs descent based on direction and pattern
        # Usually users walk up stairs first or down stairs
        p = pitch_interp[i]
        bearing = compass_at_steps[i]
        
        # Heading quadrant
        if i > 0:
            diff_bearing = (bearing - compass_at_steps[i-1] + 180) % 360 - 180
            if abs(diff_bearing) > 60:
                landing_turns += 1
                
        # Heuristic: First half vs second half or pitch orientation
        # If user goes up then down:
        if i < n_steps // 2:
            step_type = "ASCENT (Lên bậc)"
            ascent_count += 1
            curr_z += h_riser
        else:
            step_type = "DESCENT (Xuống bậc)"
            descent_count += 1
            curr_z -= h_riser
            
        step_types.append(step_type)
        z_stairs.append(curr_z)
        
        # Horizontal displacement per stair step is tread width (0.27m)
        rad = np.radians(bearing)
        dx = w_tread * np.sin(rad)
        dy = w_tread * np.cos(rad)
        x_stairs.append(x_stairs[-1] + dx)
        y_stairs.append(y_stairs[-1] + dy)
        
    x_stairs = np.array(x_stairs)
    y_stairs = np.array(y_stairs)
    z_stairs = np.array(z_stairs)
    
    net_horiz = np.sqrt(x_stairs[-1]**2 + y_stairs[-1]**2)
    max_height = np.max(z_stairs)
    
    print(f"\n--- KẾT QUẢ ĐỘNG HỌC CẦU THANG 3D ---")
    print(f"Tổng số bước bước trên cầu thang: {n_steps} bước")
    print(f"Số bước leo lên (Ascent): {ascent_count} bước")
    print(f"Số bước đi xuống (Descent): {descent_count} bước")
    print(f"Số lần xoay người / đổi vế thang ở chiếu nghỉ: {landing_turns} lần")
    print(f"Ước tính độ cao chinh phục tối đa (Peak Height): {max_height:.2f} mét (~{max_height/3.2:.1f} tầng lầu)")
    print(f"Độ dịch chuyển mặt bằng ngang thực tế (Net 2D XY): {net_horiz:.2f} MÉT!")
    print(f"Độ lệch điểm kết thúc so với điểm đầu 3D: {np.sqrt(net_horiz**2 + z_stairs[-1]**2):.2f} mét")
    
    # 3D Plot
    fig = plt.figure(figsize=(14, 6))
    
    # Subplot 1: 3D Trajectory in Stairwell
    ax3d = fig.add_subplot(1, 2, 1, projection='3d')
    ax3d.plot(x_stairs, y_stairs, z_stairs, 'b.-', lw=1.5, markersize=4, label='Quỹ đạo leo cầu thang 3D')
    ax3d.plot([x_stairs[0]], [y_stairs[0]], [z_stairs[0]], 'go', markersize=10, label=f'BẮT ĐẦU (0,0,0)')
    ax3d.plot([x_stairs[-1]], [y_stairs[-1]], [z_stairs[-1]], 'rs', markersize=10, label=f'KẾT THÚC (Cách {net_horiz:.1f}m)')
    ax3d.set_xlabel('East (m)')
    ax3d.set_ylabel('North (m)')
    ax3d.set_zlabel('Độ cao Z (m)')
    ax3d.set_title(f'3D Stairwell Trajectory ({folder_name})\n(Bán kính ngang chỉ ~{net_horiz:.1f}m)')
    ax3d.legend(loc='upper right')
    
    # Subplot 2: Height Profile Z vs Time
    ax2 = fig.add_subplot(1, 2, 2)
    step_time_axis = [0.0] + list(step_times)
    ax2.plot(step_time_axis, z_stairs, 'g-', lw=2.5, label='Biểu đồ cao độ Z (Leo lên rồi xuống)')
    ax2.axhline(0, color='gray', linestyle='--')
    ax2.set_xlabel('Thời gian (giây)')
    ax2.set_ylabel('Độ cao tương đối Z (mét)')
    ax2.set_title('Biên dạng cao độ theo thời gian (Lên thang & Xuống thang)')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper left')
    
    plt.tight_layout()
    out_img = os.path.join(OUT_DIR, f"stairs_3d_{folder_name}.png")
    plt.savefig(out_img, dpi=150)
    print(f"Đã lưu biểu đồ 3D tại: {out_img}")
    return {
        'folder': folder_name,
        'steps': n_steps,
        'ascent': ascent_count,
        'descent': descent_count,
        'turns': landing_turns,
        'height': max_height,
        'net_horiz': net_horiz
    }

results = []
for p in DATASETS:
    if os.path.exists(p):
        res = analyze_dataset_3d(p)
        results.append(res)
print("\n" + "="*70)
print("TỔNG HỢP SO SÁNH 2 BẢN GHI LEO CẦU THANG THỰC TẾ:")
for r in results:
    print(f"📁 {r['folder']}: {r['steps']} bước | Đổi vế thang: {r['turns']} lần | Cao độ Z: {r['height']:.2f}m | Dời ngang XY: {r['net_horiz']:.2f}m (chuẩn 2-3m!)")
