import pandas as pd
import matplotlib.pyplot as plt
import re
import glob, os

data_folder = "data"
csv_files = sorted(glob.glob(os.path.join(data_folder, "gas_sensor_reading*.csv")))

plt.figure(figsize=(12,6))

for file in csv_files:
    label = os.path.basename(file).replace(".csv", "").replace("_", " ").title()
    df = pd.read_csv(file)

    df["VOC_kOhm"] = df["voc_message"].apply(
        lambda x: float(re.search(r"VOC:\s*([\d.]+)", x).group(1)) if isinstance(x,str) else None
    )

    # Compute relative concentration (1/R) and normalize
    df["VOC_conc"] = 1 / df["VOC_kOhm"]
    df["VOC_conc_norm"] = df["VOC_conc"] / df["VOC_conc"].iloc[0]

    plt.plot(df["time"], df["VOC_conc_norm"], label=label, linewidth=1.5)

plt.title("Relative VOC Concentration (Inverse of Sensor Resistance)")
plt.xlabel("Time (ROS2 Timestamp)")
plt.ylabel("Normalized VOC Concentration (1/R)")
plt.legend()
plt.grid(True, linestyle="--", alpha=0.6)
plt.tight_layout()
plt.show()
