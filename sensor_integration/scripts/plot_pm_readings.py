import matplotlib.pyplot as plt

df = pd.read_csv("pm_readings.csv")
plt.plot(df["time"], df["pm_message"].str.extract(r"PM2\.5:\s*(\d+)").astype(float))
plt.xlabel("Time")
plt.ylabel("PM2.5 (µg/m³)")
plt.title("Dust plume response during robot movement")
plt.show()
