import numpy as np
import matplotlib.pyplot as plt

s = np.genfromtxt("step_response.csv", delimiter=",", names=True)
t, speed, duty = s["time_ms"], s["speed"], s["duty_cycle"]

fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
ax[0].plot(t / 1000, speed)
ax[0].set_ylabel("Speed (pulses/s)")
ax[0].set_title("Step response")
ax[1].plot(t / 1000, duty, color="tab:orange")
ax[1].set_ylabel("Duty cycle")
ax[1].set_xlabel("Time (s)")
for a in ax:
    a.grid(True)
fig.savefig("step_response.png", dpi=150)

c = np.genfromtxt("speed_control.csv", delimiter=",", names=True)
plt.figure(figsize=(10, 5))
plt.plot(c["time_ms"] / 1000, c["target_speed"], "k", label="Target speed")
plt.plot(c["time_ms"] / 1000, c["measured_speed"], label="Measured speed")
plt.xlabel("Time (s)")
plt.ylabel("Speed (pulses/s)")
plt.title("Speed control, 3 triangular cycles")
plt.legend()
plt.grid(True)
plt.savefig("speed_control.png", dpi=150)

for name, start, d in [("Forward", 1000, 0.6), ("Reverse", 4000, -0.6)]:
    seg = speed[start:start + 1000]
    ss = seg[-300:].mean()
    sm = np.convolve(seg, np.ones(5) / 5, mode="full")[:len(seg)]
    frac = sm / ss
    t10 = np.argmax(frac >= 0.1)
    t90 = np.argmax(frac >= 0.9)
    t63 = np.argmax(frac >= 0.632)
    out2 = np.flatnonzero(np.abs(sm - ss) > 0.02 * abs(ss))
    out5 = np.flatnonzero(np.abs(sm - ss) > 0.05 * abs(ss))
    print(f"{name} step ({d:+.2f} duty):")
    print(f"  steady-state speed  {ss:.2f} pulses/s")
    print(f"  motor gain          {ss / d:.2f} pulses/s per unit duty")
    print(f"  rise time 10-90%    {t90 - t10} ms")
    print(f"  time constant 63%   {t63} ms")
    print(f"  settling time 2%    {out2[-1] + 1 if out2.size else 0} ms")
    print(f"  settling time 5%    {out5[-1] + 1 if out5.size else 0} ms")
    print(f"  noise std           {seg[-300:].std():.2f} pulses/s "
          f"({seg[-300:].std() / abs(ss) * 100:.1f}% of steady state)")

err = c["target_speed"] - c["measured_speed"]
print(f"Speed control RMS error {np.sqrt(np.mean(err ** 2)):.2f} pulses/s, "
      f"measured range {c['measured_speed'].min():.1f} to {c['measured_speed'].max():.1f}")