import pandas as pd
import matplotlib.pyplot as plt
import re
import glob
import os

# Folder containing your CSVs
data_folder = "data"

# Find all matching CSV files
csv_files = sorted(glob.glob(os.path.join(data_folder, "gas_sensor_reading*.csv")))

if not csv_files:
    raise FileNotFoundError(f"No CSV files found in {data_folder} starting with 'gas_reading'")

plt.figure(figsize=(12, 6))

for file in csv_files:
    # Extract readable label (file name without extension)
    label = os.path.basename(file).replace(".csv", "").replace("_", " ").title()

    # Load CSV
    df = pd.read_csv(file)

    # Extract numeric VOC values (in kΩ)
    df["VOC_kOhm"] = df["voc_message"].apply(
        lambda x: float(re.search(r"VOC:\s*([\d.]+)", x).group(1)) if isinstance(x, str) else None
    )

    # Normalize VOC readings by baseline (first valid value)
    baseline = df["VOC_kOhm"].dropna().iloc[0]
    df["VOC_norm"] = df["VOC_kOhm"] / baseline

    # Plot normalized data
    plt.plot(df["time"], df["VOC_norm"], label=label, linewidth=1.5)

plt.title("Normalized VOC Sensor Response — All Perfume Tests")
plt.xlabel("Time (ROS2 Timestamp)")
plt.ylabel("Normalized VOC Ratio (R/R₀)")
plt.legend()
plt.grid(True, linestyle="--", alpha=0.6)
plt.tight_layout()
plt.show()
