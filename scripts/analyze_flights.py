import pandas as pd
import numpy as np

df = pd.read_csv(r"c:\New PJ\OFF-GRID SURVIVAL\scripts\dataset1_steps_detailed.csv")

# Let's inspect the compass bearing changes
compass = df['compass'].values
# Unwrap compass to track continuous rotation
diffs = (np.diff(compass) + 180) % 360 - 180
unwrapped = [compass[0]]
for d in diffs:
    unwrapped.append(unwrapped[-1] + d)
df['compass_unwrapped'] = unwrapped

# Let's also do unwrapped orient_yaw
orient_yaw = df['orient_yaw_deg'].values
diffs_yaw = (np.diff(orient_yaw) + 180) % 360 - 180
unwrapped_yaw = [orient_yaw[0]]
for d in diffs_yaw:
    unwrapped_yaw.append(unwrapped_yaw[-1] + d)
df['yaw_unwrapped'] = unwrapped_yaw

print("STEP TIMELINE OF TURNS AND FLIGHTS:")
# Print every 5 steps or where diff is large
for i in range(len(df)):
    step = df.iloc[i]
    d_comp = diffs[i-1] if i > 0 else 0
    d_yaw = diffs_yaw[i-1] if i > 0 else 0
    is_turn = abs(d_comp) > 40 or abs(d_yaw) > 40
    if is_turn or i % 10 == 0:
        print(f"Step {int(step['step']):3d} | t={step['time']:5.1f}s | Compass={step['compass']:5.1f}° (unwrapped={step['compass_unwrapped']:6.1f}°) | Yaw={step['orient_yaw_deg']:5.1f}° (unwrapped={step['yaw_unwrapped']:6.1f}°) | Turn={d_comp:+5.1f}° | gx={step['gyro_x']:+5.2f}")
