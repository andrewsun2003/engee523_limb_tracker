import pandas as pd

# === Step 1: Load CSV ===
df = pd.read_csv("/Users/andrewsun/Downloads/engee523_limb_tracker-main/sensor_fusion/figure4.csv")

# === Step 2: Define known/reference angles from calibration unit ===
reference_angles = {
    "DMP_Roll": 60.0,   # degrees, set based on calibration unit
    "DMP_Pitch": 0.0,
    "DMP_Yaw": 180.0
}

# === Step 3: Compute precision & bias ===
def analyse(axis_label, data, reference):
    mean = data.mean()
    std_dev = data.std()
    bias = mean - reference
    print(f"{axis_label}:")
    print(f"  Reference Angle = {reference:.2f}°")
    print(f"  Mean Sensor Reading = {mean:.2f}°")
    print(f"  Bias = {bias:+.2f}°")
    print(f"  Precision (Std Dev) = ±{std_dev:.2f}°\n")
    return {
        "axis": axis_label,
        "reference": reference,
        "mean": mean,
        "bias": bias,
        "precision": std_dev
    }

# === Step 4: Run analysis for all 3 axes ===
results = []
for axis in ["DMP_Roll", "DMP_Pitch", "DMP_Yaw"]:
    results.append(analyse(axis, df[axis], reference_angles[axis]))
