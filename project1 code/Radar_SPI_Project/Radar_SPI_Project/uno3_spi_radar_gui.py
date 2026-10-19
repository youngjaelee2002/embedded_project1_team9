#!/usr/bin/env python3
"""Read-only UNO3 SPI scanner monitor. Python stdlib tkinter + pyserial.

USB port is opened for incoming ASCII telemetry only: there are NO serial writes.
Run: python uno3_spi_radar_gui.py [--simulate]
"""
import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

BAUD = 115200

# Physical 2D scanner layout (world coordinates in millimetres).
WORLD_SIZE_MM = 600
SCANNER_X_MM = 300
SCANNER_Y_MM = 300
RADAR_BEAM_MM = WORLD_SIZE_MM // 2
BG, PANEL, TEXT = "#0b1522", "#152537", "#e7f3fe"
MUTED, GREEN, AMBER = "#9bb1c6", "#4ce2b2", "#f3bb76"
MODE = {0: "STOP", 1: "SEARCH SLOW", 2: "SEARCH FAST", 3: "TRACK TARGET"}
STATE = {0: "CALIBRATION", 1: "IDLE", 2: "MOVING", 3: "FAULT"}
ERRORS = {0: "none", 1: "RX CRC", 2: "RX header", 3: "bad command",
          4: "not homed", 5: "ToF unavailable", 6: "SPI timeout",
          7: "ToF stale", 8: "Hall drift", 9: "target lost"}


def parse_object(parts):
    """OBJECT|id|x|y|r|theta_cdeg|vx|vy|vr|vt|quality|age|flags."""
    if len(parts) != 13:
        raise ValueError("OBJECT requires 12 fields")
    ident, x, y, r, theta, vx, vy, vr, vt, quality, age, flags = map(int, parts[1:])
    if not 1 <= ident <= 255 or not 0 <= theta < 36000:
        raise ValueError("object ID or centidegrees out of bounds")
    return dict(id=ident, x=x, y=y, r=r, theta=theta / 100.0,
                vx=vx, vy=vy, vr=vr, vt=vt, quality=quality, age=age,
                flags=flags, updated=time.monotonic())


def parse_status(parts):
    """STATUS + 22 fields; defined in SPI_PROTOCOL.md. Pure parsing for tests."""
    if len(parts) != 23:
        raise ValueError("STATUS requires 22 fields")
    (ms, mode, state, homed, step, hall, dout, aout, tof, strikes,
     target, count, torque, good, bad, ack, coord, error, range_mm,
     samples, invalid, timeout) = map(int, parts[1:])
    return locals()


class Monitor:
    def __init__(self, root, simulate=False):
        self.root = root
        self.simulate = simulate
        root.title("UNO 3 | Distributed SPI Radar — USB Monitor")
        root.geometry("1280x820")
        root.minsize(980, 650)
        root.configure(bg=BG)
        self.port_name = tk.StringVar()
        self.connection = tk.StringVar(value="SIMULATION" if simulate else "DISCONNECTED")
        self.detail = tk.StringVar(value="Awaiting scanner telemetry")
        self.calibration = tk.StringVar(value="Motor torque releases automatically during calibration")
        self.status = {name: tk.StringVar(value="—") for name in
                       ("MODE", "STATE", "HOMED", "ANGLE", "HALL", "TOF", "TORQUE",
                        "OBJECTS", "TARGET ID", "STRIKES", "SPI RX OK", "SPI RX BAD",
                        "ACK SEQ", "SPI FORMAT", "ERROR", "LAST RANGE", "SAMPLES", "BAD SAMPLES")}
        self.objects = {}
        self.serial_port = None
        self.reader = None
        self.stop = threading.Event()
        self.inbox = queue.Queue(maxsize=1000)
        self.last_rx = 0
        self.step = 0
        self.samples = 0
        self.invalid_samples = 0
        self.log_count = 0
        self.writer = None
        self.csv_file = None
        self.since = time.monotonic()
        self.port_list = []
        self._make_ui()
        self.refresh_ports()
        root.after(70, self.poll)
        root.after(100, self.draw)
        if simulate:
            root.after(200, self.simulate_telemetry)
        root.protocol("WM_DELETE_WINDOW", self.close)

    def _make_ui(self):
        s = ttk.Style()
        s.theme_use("clam")
        s.configure("Treeview", background="#0e1c2c", foreground=TEXT,
                    fieldbackground="#0e1c2c", rowheight=27)
        s.configure("Treeview.Heading", background="#2a425c", foreground=TEXT)
        header = tk.Frame(self.root, bg=BG)
        header.pack(fill="x", padx=18, pady=(15, 7))
        tk.Label(header, text="UNO 3   /   DISTRIBUTED SPI RADAR", fg=TEXT,
                 bg=BG, font=("Segoe UI", 18, "bold")).pack(side="left")
        tk.Label(header, textvariable=self.connection, fg=GREEN, bg=PANEL,
                 padx=12, pady=7).pack(side="right")
        ports = tk.Frame(self.root, bg=BG)
        ports.pack(fill="x", padx=18, pady=(0, 10))
        tk.Label(ports, text="USB SERIAL", bg=BG, fg=MUTED).pack(side="left", padx=(0, 8))
        self.combo = ttk.Combobox(ports, width=22, textvariable=self.port_name)
        self.combo.pack(side="left", padx=(0, 6))
        ttk.Button(ports, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(ports, text="Connect", command=self.connect)
        self.connect_button.pack(side="left", padx=5)
        ttk.Button(ports, text="Record CSV", command=self.record).pack(side="left", padx=9)
        tk.Label(ports, text="READ ONLY  •  115200 baud  •  Commands only via UNO 1 SPI",
                 bg=BG, fg=MUTED).pack(side="right")
        if self.simulate:
            self.combo.configure(state="disabled")
            self.connect_button.configure(state="disabled")

        pane = tk.PanedWindow(self.root, orient="horizontal", bg=BG,
                             sashwidth=6, showhandle=False)
        pane.pack(fill="both", expand=True, padx=18, pady=(0, 16))
        lhs = tk.Frame(pane, bg=BG)
        rhs = tk.Frame(pane, bg=BG)
        pane.add(lhs, minsize=400)
        pane.add(rhs, minsize=500)
        self.canvas = tk.Canvas(lhs, bg="#091e25", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        tk.Label(lhs, text=f"World {WORLD_SIZE_MM} × {WORLD_SIZE_MM} mm  •  "
                           f"Scanner at ({SCANNER_X_MM}, {SCANNER_Y_MM}) mm",
                 bg=BG, fg=MUTED).pack(pady=(7, 0))

        info = tk.Frame(rhs, bg=PANEL, padx=13, pady=10)
        info.pack(fill="x")
        labels = list(self.status)
        for i, name in enumerate(labels):
            row, column = divmod(i, 3)
            sub = tk.Frame(info, bg=PANEL)
            sub.grid(row=row, column=column, sticky="ew", padx=6, pady=5)
            tk.Label(sub, text=name, bg=PANEL, fg=MUTED,
                     font=("Segoe UI", 8)).pack(anchor="w")
            tk.Label(sub, textvariable=self.status[name], bg=PANEL, fg=GREEN,
                     font=("Consolas", 10, "bold")).pack(anchor="w")
        for column in range(3):
            info.grid_columnconfigure(column, weight=1)
        tk.Label(rhs, textvariable=self.calibration, bg=BG, fg=AMBER,
                 font=("Segoe UI", 10), anchor="w", wraplength=650).pack(fill="x", pady=(8, 3))
        tk.Label(rhs, text="DETECTED OBJECTS — CARTESIAN + CYLINDRICAL",
                 bg=BG, fg=TEXT, font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(7, 5))
        cols = ("id", "x", "y", "r", "theta", "vx", "vy", "vr", "vt", "q", "age")
        self.tree = ttk.Treeview(rhs, columns=cols, show="headings", height=5)
        titles = {"id": "ID", "x": "X mm", "y": "Y mm", "r": "r mm",
                  "theta": "θ °", "vx": "Vx", "vy": "Vy", "vr": "Vr", "vt": "Vt", "q": "Q", "age": "Age"}
        for key in cols:
            self.tree.heading(key, text=titles[key])
            self.tree.column(key, width=48 if key in ("id", "q", "age") else 61,
                             stretch=True, anchor="center")
        self.tree.pack(fill="x")
        tk.Label(rhs, textvariable=self.detail, bg=BG, fg=MUTED,
                 anchor="w").pack(fill="x", pady=5)
        tk.Label(rhs, text="EVENTS AND DIAGNOSTICS", bg=BG, fg=TEXT,
                 font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(5, 4))
        self.log = tk.Text(rhs, bg="#0a1a28", fg="#a7c3da", height=9,
                           relief="flat", wrap="word", font=("Consolas", 9), state="disabled")
        self.log.pack(fill="both", expand=True)

    def refresh_ports(self):
        self.port_list = [p.device for p in list_ports.comports()] if list_ports else []
        self.combo.configure(values=self.port_list)
        if self.port_list and self.port_name.get() not in self.port_list:
            self.port_name.set(self.port_list[0])

    def connect(self):
        if self.serial_port:
            self.disconnect()
            return
        if serial is None:
            messagebox.showerror("Missing pyserial", "Run: python -m pip install -r requirements.txt")
            return
        if not self.port_name.get():
            messagebox.showwarning("Missing port", "Select the UNO 3 USB port")
            return
        try:
            # Request inactive DTR/RTS before opening where OS/drivers permit.
            # Some Uno USB stacks still pulse DTR and reset; see README safety note.
            port = serial.Serial(port=None, baudrate=BAUD, timeout=0.25)
            port.dtr = False
            port.rts = False
            port.port = self.port_name.get()
            port.open()
            self.serial_port = port
        except (OSError, serial.SerialException) as exc:
            messagebox.showerror("Serial error", str(exc))
            return
        self.stop.clear()
        self.connection.set("CONNECTED — " + self.port_name.get())
        self.connect_button.configure(text="Disconnect")
        self.reader = threading.Thread(target=self.read_lines, daemon=True)
        self.reader.start()
        self.log_line("Serial opened (Uno may automatically reset)")

    def read_lines(self):
        port = self.serial_port
        while not self.stop.is_set():
            try:
                raw = port.readline()
            except (OSError, serial.SerialException) as exc:
                if not self.stop.is_set():
                    self.enqueue("DISCONNECT|" + str(exc))
                return
            if raw:
                self.enqueue(raw.decode("utf-8", "replace").strip())

    def enqueue(self, text):
        try:
            self.inbox.put_nowait(text)
        except queue.Full:
            pass

    def disconnect(self):
        self.stop.set()
        old, self.serial_port = self.serial_port, None
        if old:
            try:
                old.close()
            except OSError:
                pass
        self.connection.set("DISCONNECTED")
        self.connect_button.configure(text="Connect")

    def record(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = self.writer = None
            self.log_line("CSV recording stopped")
            return
        filename = filedialog.asksaveasfilename(defaultextension=".csv",
                  filetypes=[("CSV", "*.csv")], initialfile="uno3_spi_telemetry.csv")
        if not filename:
            return
        try:
            self.csv_file = Path(filename).open("w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow(["pc_time", "telemetry_line"])
        except OSError as exc:
            self.csv_file = self.writer = None
            messagebox.showerror("CSV error", str(exc))
            return
        self.log_line("CSV recording started — click Record CSV again to stop")

    def poll(self):
        for _ in range(100):
            try:
                self.handle(self.inbox.get_nowait())
            except queue.Empty:
                break
        if not self.simulate and self.serial_port and self.last_rx and time.monotonic()-self.last_rx>3:
            self.connection.set("SERIAL STALE")
        self.root.after(60, self.poll)

    def handle(self, line):
        if not line:
            return
        if line.startswith("DISCONNECT|"):
            self.disconnect()
            self.log_line(line)
            return
        self.last_rx = time.monotonic()
        if self.serial_port and self.connection.get() == "SERIAL STALE":
            self.connection.set("CONNECTED — " + self.port_name.get())
        if self.writer:
            try:
                self.writer.writerow((datetime.now().isoformat(timespec="milliseconds"), line))
            except OSError:
                self.log_line("CSV write failed")
                self.csv_file.close()
                self.csv_file = self.writer = None
        p = line.split("|")
        try:
            if p[0] == "STATUS":
                s = parse_status(p)
                values = {
                    "MODE": MODE.get(s["mode"], "UNKNOWN"), "STATE": STATE.get(s["state"], "UNKNOWN"),
                    "HOMED": "YES" if s["homed"] else "NO",
                    "ANGLE": f"{-s['step'] * 360 / 3200:.1f}°",
                    "HALL": f"{'MAGNET' if s['hall'] else 'clear'}  A{s['aout']}",
                    "TOF": "READY" if s["tof"] else "OFFLINE",
                    "TORQUE": "HOLD (commanded)" if s["torque"] else "FREE (commanded)",
                    "OBJECTS": str(s["count"]), "TARGET ID": str(s["target"]),
                    "STRIKES": str(s["strikes"]), "SPI RX OK": str(s["good"]),
                    "SPI RX BAD": str(s["bad"]), "ACK SEQ": str(s["ack"]),
                    "SPI FORMAT": "Cylindrical" if s["coord"] else "Cartesian",
                    "ERROR": ERRORS.get(s["error"], str(s["error"])),
                    "LAST RANGE": f"{s['range_mm']} mm", "SAMPLES": str(s["samples"]),
                    "BAD SAMPLES": str(s["invalid"]),
                }
                for key, value in values.items():
                    self.status[key].set(value)
                self.step = s["step"]
                self.samples = s["samples"]
                self.invalid_samples = s["invalid"]
                self.detail.set("Read-only USB  •  SPI errors: " + str(s["bad"]) +
                    ("  •  COMMAND LINK TIMEOUT" if s["timeout"] else ""))
            elif p[0] == "OBJECT":
                obj = parse_object(p)
                self.objects[obj["id"]] = obj
                self.refresh_table()
            elif p[0] == "LOST" and len(p) >= 2:
                self.objects.pop(int(p[1]), None)
                self.refresh_table()
            elif p[0] == "RESET":
                self.objects.clear()
                self.refresh_table()
            elif p[0] == "CALIBRATION":
                if len(p) > 2 and p[1] == "magnet-detected":
                    self.calibration.set("Magnet detected — automatic HOLD in " + p[2] + " s")
                elif len(p) > 1 and p[1] == "reference-locked":
                    self.calibration.set("Reference locked — motor automatically HOLDING")
                else:
                    self.calibration.set("Manual calibration: motor FREE, align magnet and hold for 5 s")
            if p[0] not in ("STATUS", "OBJECT"):
                self.log_line(line)
        except (ValueError, IndexError) as exc:
            self.log_line("Malformed telemetry: " + str(exc) + " — " + line)

    def refresh_table(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        for ident, o in sorted(self.objects.items()):
            self.tree.insert("", "end", values=(ident, o["x"], o["y"], o["r"],
                       f"{o['theta']:.2f}", o["vx"], o["vy"], o["vr"], o["vt"],
                       o["quality"], o["age"]))

    def log_line(self, line):
        self.log.configure(state="normal")
        self.log.insert("end", datetime.now().strftime("%H:%M:%S") + "  " + line + "\n")
        self.log_count += 1
        if self.log_count > 250:
            self.log.delete("1.0", "50.0")
            self.log_count -= 49
        self.log.see("end")
        self.log.configure(state="disabled")

    def draw(self):
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        side = max(180, min(w-30, h-36))
        scale = side / WORLD_SIZE_MM
        cx, cy = w/2, h/2
        def pt(x,y):
            return cx + (x - SCANNER_X_MM) * scale, cy - (y - SCANNER_Y_MM) * scale
        a, b = pt(0, WORLD_SIZE_MM), pt(WORLD_SIZE_MM, 0)
        c.create_rectangle(*a,*b,outline="#28685c",width=2)
        for radius in range(100, RADAR_BEAM_MM + 1, 100):
            r = radius*scale
            c.create_oval(cx-r,cy-r,cx+r,cy+r,outline="#16483f")
        c.create_line(cx,b[1],cx,a[1],fill="#2d4b52",dash=(3,5))
        c.create_line(a[0],cy,b[0],cy,fill="#2d4b52",dash=(3,5))
        theta = -self.step*2*math.pi/3200
        x,y = pt(SCANNER_X_MM + RADAR_BEAM_MM * math.cos(theta),
                 SCANNER_Y_MM + RADAR_BEAM_MM * math.sin(theta))
        c.create_line(cx,cy,x,y,fill="#277b6b",width=2)
        c.create_oval(cx-5,cy-5,cx+5,cy+5,fill=GREEN,outline="")
        for ob in self.objects.values():
            px,py=pt(ob["x"],ob["y"])
            c.create_oval(px-9,py-9,px+9,py+9,outline=AMBER,width=2)
            c.create_line(px-14,py,px+14,py,fill=AMBER)
            c.create_line(px,py-14,px,py+14,fill=AMBER)
            c.create_text(px+11,py-12,text=f"ID {ob['id']} ({ob['x']},{ob['y']})",
                          anchor="sw",fill=TEXT,font=("Consolas",9,"bold"))
        c.create_text(a[0]+9,b[1]-10,anchor="sw",fill=MUTED,
                      text=f"{self.samples} ToF samples  /  {self.invalid_samples} invalid")
        self.root.after(100, self.draw)

    def simulate_telemetry(self):
        t=time.monotonic()-self.since
        pos=(int(t*900) % 6400)
        pos=pos if pos < 3200 else 6400-pos
        self.handle(f"STATUS|{int(t*1000)}|1|2|1|{pos}|0|1|512|1|0|0|1|1|250|0|2|0|0|280|{int(t*40)}|0|0")
        x = SCANNER_X_MM + 240 + round(15 * math.sin(t / 4))
        y = SCANNER_Y_MM + 60 + round(15 * math.cos(t / 3))
        dx, dy = x - SCANNER_X_MM, y - SCANNER_Y_MM
        theta = math.degrees(math.atan2(dy, dx)) % 360
        r = round(math.hypot(dx, dy))
        self.handle(f"OBJECT|1|{x}|{y}|{r}|{round(theta*100)}|0|0|0|0|220|0|2")
        self.root.after(350,self.simulate_telemetry)

    def close(self):
        self.stop.set()
        if self.csv_file:
            self.csv_file.close()
        if self.serial_port:
            self.serial_port.close()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulate", action="store_true", help="clearly labeled GUI demo without hardware")
    args = parser.parse_args()
    root = tk.Tk()
    Monitor(root, simulate=args.simulate)
    root.mainloop()


if __name__ == "__main__":
    main()
