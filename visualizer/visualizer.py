import serial
import collections
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


SERIAL_PORT = "/dev/cu.usbmodem2103"   
BAUD_RATE = 115200


WINDOW_SIZE = 60  


tremor_raw  = collections.deque([0.0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
dysk_raw    = collections.deque([0.0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
freeze_raw  = collections.deque([0.0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)


ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


fig, ax = plt.subplots()
x_vals = list(range(WINDOW_SIZE))

line_tremor, = ax.plot(x_vals, [0.0]*WINDOW_SIZE, label="Tremor (normalized)")
line_dysk,   = ax.plot(x_vals, [0.0]*WINDOW_SIZE, label="Dyskinesia (normalized)")
line_freeze, = ax.plot(x_vals, [0.0]*WINDOW_SIZE, label="Freeze Index (normalized)")

ax.set_ylim(0, 1.05)
ax.set_title("Real-time Parkinsonian Movement Indicators (Normalized)")
ax.set_xlabel("Batch (time)")
ax.set_ylabel("Relative Strength (0~1)")
ax.legend(loc="upper right")

status_text = ax.text(0.02, 0.95, "Status: Normal",
                      transform=ax.transAxes, fontsize=11,
                      verticalalignment="top")

def normalize_series(data_deque):
    """把一条数据队列缩放到 0~1 之间"""
    max_val = max(abs(v) for v in data_deque)
    if max_val < 1e-9:
        
        return [0.0 for _ in data_deque]
    return [v / max_val for v in data_deque]

def update(frame):
    try:
        line = ser.readline().decode(errors="ignore").strip()
    except Exception:
        return line_tremor, line_dysk, line_freeze, status_text


    if line.startswith("DATA"):
   
        parts = line.split(",")
        if len(parts) == 7:
            try:
                tremor = float(parts[1])
                dysk   = float(parts[2])
                freeze = float(parts[3])
                is_trem = parts[4] == "1"
                is_dysk = parts[5] == "1"
                is_free = parts[6] == "1"

                tremor_raw.append(tremor)
                dysk_raw.append(dysk)
                freeze_raw.append(freeze)

              
                tremor_norm  = normalize_series(tremor_raw)
                dysk_norm    = normalize_series(dysk_raw)
                freeze_norm  = normalize_series(freeze_raw)

                line_tremor.set_ydata(tremor_norm)
                line_dysk.set_ydata(dysk_norm)
                line_freeze.set_ydata(freeze_norm)

           
                states = []
                if is_trem: states.append("Tremor")
                if is_dysk: states.append("Dyskinesia")
                if is_free: states.append("Freezing gait")
                status_text.set_text("Status: " + (" + ".join(states) if states else "Normal"))

            except ValueError:
                pass

    return line_tremor, line_dysk, line_freeze, status_text

ani = FuncAnimation(fig, update, interval=200)

print("Visualizer started (normalized 0~1). Close the window to stop.")
plt.show()
ser.close()
