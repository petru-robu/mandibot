import ttkbootstrap as ttk
from ttkbootstrap.constants import *
import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import font as tkfont
from ttkbootstrap.style import ThemeDefinition
from ttkbootstrap import ToolTip
import serial
import serial.tools.list_ports
import time
import cv2
import numpy as np
import threading
import queue
import os
import math
from datetime import datetime
from PIL import Image, ImageColor, ImageDraw, ImageTk

# --- CONFIGURATION ---
SERIAL_PORT = None  # None = auto-detect (first /dev/ttyUSB* or /dev/ttyACM*)
BAUD_RATE = 115200
SERIAL_FLUSH_MS = 20  # serial writes are coalesced and sent at most this often
AUDIO_HOLD_S = 0.03  # motor writes pause this long after an audio command (the Arduino's SoftwareSerial
                     # blocks its UART interrupt for ~5 ms while sending it, which would drop motor bytes)

CAMERA_INDEX = 1
MARKER_SIZE_MM = 20.0  # real side length of the printed ArUco markers (used for the mm unit)
TOP_MARKER_ID = 15     # ArUco marker on the upper (fixed) part: the reference
BOTTOM_MARKER_ID = 13  # ArUco marker on the mandible: its motion relative to the top marker is tracked
TRAJ_SMOOTHING = 0.3   # weight of each new camera sample in the trajectory (lower = smoother, but lags more)
TRAJ_MIN_STEP_PX = 0.5  # a trajectory point is added only once the jaw moved this far (fewer, cleaner points)

NUM_MOTORS = 8
MOTOR_NAMES = [
    "TEMP STG\n[0]", "TEMP DRP\n[1]",
    "MAS STG\n[2]", "MAS DRP\n[3]",
    "LAT STG\n[4]", "LAT DRP\n[5]",
    "MED STG\n[6]", "MED DRP\n[7]"
]

# Instrument-style palette: deep navy, steel blue and teal on a cool light grey
THEME_NAME = "mandibot"
THEME_COLORS = {
    "primary":   "#1f3a5f",
    "secondary": "#6b7785",
    "success":   "#2a7f78",
    "info":      "#33658a",
    "warning":   "#a8681a",
    "danger":    "#b23a30",
    "light":     "#f3f5f8",
    "dark":      "#2b3643",
    "bg":        "#e8ebf0",
    "fg":        "#1b2530",
    "selectbg":  "#33658a",
    "selectfg":  "#ffffff",
    "border":    "#c6ccd5",
    "inputfg":   "#1b2530",
    "inputbg":   "#ffffff",
    "active":    "#d8dde5",
}
UI_FONTS = ["Inter", "Segoe UI", "Noto Sans", "Cantarell", "DejaVu Sans", "Helvetica"]  # first one installed wins
APP_NAME = "Mandibot"
LOGO_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "res", "logo.png")
LOGO_HEIGHT = 90  # px, the logo is scaled to this height in the header

PRESET_POSITIONS = {
    "Fully Closed":        [100, 100, 100, 100, 10, 10, 100, 100],
    "Open: Phase 1 (Rot)": [25, 25, 25, 25, 10, 10, 25, 25],
    "Open: Phase 2 (Tra)": [10, 10, 5, 5, 85, 85, 5, 5],
    "Protrusion":          [20, 20, 15, 15, 100, 100, 15, 15],
    "Left Laterality":     [90, 45, 85, 45, 10, 100, 30, 65],
    "Right Laterality":    [30, 82, 30, 80, 100, 0, 70, 15]
}

MASTICATION_REPEATS = 3

# One left-side chewing stroke (raw poses, independent of the preset buttons):
# drop -> open while shifting left -> close up onto the left teeth (grind) -> back to centre
MASTICATION_CYCLE = [
    [25, 25, 25, 25, 10, 10, 25, 25],     # Drop: slight hinge opening
    [45, 5, 40, 5, 10, 100, 5, 35],      # Open wider while shifting left (teeth clear)
    [100, 55, 95, 55, 10, 100, 40, 75],   # Close onto the left side (grind)
    [100, 100, 100, 100, 10, 10, 100, 100],  # Return to centric closure
]

# Opening = rotation followed by translation
OPEN_SEQUENCE = ["Open: Phase 1 (Rot)", "Open: Phase 2 (Tra)"]

# Fill colour of each action button
BUTTON_STYLES = {
    "Fully Closed":        SECONDARY,
    "Open: Phase 1 (Rot)": INFO,
    "Open: Phase 2 (Tra)": INFO,
    "Open (Rot + Tra)":    INFO,
    "Protrusion":          PRIMARY,
    "Left Laterality":     PRIMARY,
    "Right Laterality":    PRIMARY,
}


# --- ARDUINO CONNECTION ---
class SerialLink:
    """Arduino connection with a non-blocking connect and change-only, throttled writes."""

    def __init__(self, port=None, baud=BAUD_RATE):
        self.port_override = port
        self.baud = baud
        self.ser = None
        self.state = "disconnected"  # disconnected | connecting | connected
        self.port = None
        self.error = None
        self.targets = [None] * NUM_MOTORS    # latest wanted value per motor (0-100)
        self.last_sent = [None] * NUM_MOTORS  # last value the Arduino was given
        self.hold_until = 0.0                 # no motor writes before this time (see AUDIO_HOLD_S)

    def find_port(self):
        if self.port_override:
            return self.port_override
        ports = sorted(p.device for p in serial.tools.list_ports.comports()
                       if "ttyUSB" in p.device or "ttyACM" in p.device)
        return ports[0] if ports else None

    def connect(self):
        if self.state == "connecting":
            return
        self.close()
        self.state = "connecting"
        self.error = None
        threading.Thread(target=self._connect_worker, daemon=True).start()

    def _connect_worker(self):
        try:
            port = self.find_port()
            if port is None:
                raise serial.SerialException("no Arduino serial port found")
            ser = serial.Serial(port, self.baud, timeout=0.1)
            time.sleep(2)  # the Arduino resets when the port is opened
            ser.reset_input_buffer()
            self.ser = ser
            self.port = port
            self.last_sent = [None] * NUM_MOTORS  # resend every motor so the Arduino matches the GUI
            self.state = "connected"
            print(f"Successfully connected to {port}")
        except Exception as e:
            self.error = str(e)
            self.port = None
            self.state = "disconnected"
            print(f"Error connecting to Arduino: {e}")
            print("Running in UI-only test mode (no data will be sent).")

    def close(self):
        ser, self.ser = self.ser, None
        self.state = "disconnected"
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def set_target(self, motor_index, value):
        self.targets[motor_index] = int(round(value))

    def send_line(self, line):
        """Send a one-off command (e.g. audio) immediately. Returns False when not connected."""
        if self.state != "connected" or self.ser is None:
            return False
        try:
            self.ser.write(f"{line}\n".encode("utf-8"))
        except (serial.SerialException, OSError) as e:
            self.error = str(e)
            self.close()
            return False
        self.hold_until = time.monotonic() + AUDIO_HOLD_S
        return True

    def flush(self):
        """Send only the motors whose value changed since the last send, in one write."""
        if self.state != "connected" or self.ser is None:
            return
        if time.monotonic() < self.hold_until:  # changed values stay queued and go out on a later tick
            return
        changed = [(i, v) for i, v in enumerate(self.targets)
                   if v is not None and v != self.last_sent[i]]
        if not changed:
            return
        try:
            self.ser.write("".join(f"{i}:{v}\n" for i, v in changed).encode("utf-8"))
        except (serial.SerialException, OSError) as e:
            self.error = str(e)
            self.close()
            return
        for i, v in changed:
            self.last_sent[i] = v


# --- VIDEO CAPTURE THREAD ---
class VideoCaptureThread(threading.Thread):
    def __init__(self, frame_q, traj_q, track_q):
        super().__init__()
        self.frame_q = frame_q
        self.traj_q = traj_q
        self.track_q = track_q
        self.running = True
        self.daemon = True

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.history = []
        self.max_history = 50
        self.smoothed = None    # smoothed (dX, dY) of the bottom marker, in px
        self.traj_size = (450, 300)
        self.zoom = 3

        self.is_recording = False
        self.reset_req = False

        self.units = "px"       # "px" or "mm", set from the GUI
        self.mm_per_px = None   # smoothed scale estimated from the marker size

    @staticmethod
    def marker_side(pts):
        return float(np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)]))

    def draw_overlay(self, canvas, move):
        """Draw the current offset and a scale bar in the selected unit."""
        factor, unit = 1.0, "px"
        if self.units == "mm":
            if self.mm_per_px:
                factor, unit = self.mm_per_px, "mm"
            else:
                cv2.putText(canvas, "mm: waiting for markers", (8, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1, cv2.LINE_AA)

        h, w = canvas.shape[:2]
        if move is not None:
            text = f"dX {move[0] * factor:+.1f} {unit}   dY {move[1] * factor:+.1f} {unit}"
            cv2.putText(canvas, text, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        best = None
        for n in (1, 2, 5, 10, 20, 50, 100, 200):
            length = n / factor * self.zoom
            if length > 140:
                break
            best = (n, int(length))
        if best and best[1] >= 10:
            x1, y = w - 10, h - 30
            cv2.line(canvas, (x1 - best[1], y), (x1, y), (200, 200, 200), 2)
            cv2.putText(canvas, f"{best[0]} {unit}", (x1 - best[1], y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

    def run(self):
        cap = cv2.VideoCapture(CAMERA_INDEX)
        local_baseline_X = None
        local_baseline_Y = None

        while self.running:
            if not self.track_q.empty():
                msg = self.track_q.get_nowait()
                if msg == "RESET_BASELINE":
                    self.reset_req = True
                elif msg == "START_RECORD":
                    self.is_recording = True
                    self.history = []
                    self.reset_req = True
                elif msg == "STOP_RECORD":
                    self.is_recording = False
                    while len(self.history) > self.max_history:
                        self.history.pop(0)

            ret, frame = cap.read()
            if not ret: continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.detector.detectMarkers(gray)

            traj_canvas = np.full((self.traj_size[1], self.traj_size[0], 3), (24, 20, 17), dtype=np.uint8)
            center_x, center_y = self.traj_size[0] // 2, self.traj_size[1] // 2

            cv2.line(traj_canvas, (0, center_y), (self.traj_size[0], center_y), (52, 46, 40), 1)
            cv2.line(traj_canvas, (center_x, 0), (center_x, self.traj_size[1]), (52, 46, 40), 1)

            last_move = None
            if ids is not None and TOP_MARKER_ID in ids and BOTTOM_MARKER_ID in ids:
                idx_0 = np.where(ids == TOP_MARKER_ID)[0][0]
                idx_1 = np.where(ids == BOTTOM_MARKER_ID)[0][0]
                c0 = np.mean(corners[idx_0][0], axis=0)  # sub-pixel centres (no rounding: no stair steps)
                c1 = np.mean(corners[idx_1][0], axis=0)

                cv2.drawMarker(frame, tuple(c0.astype(int)), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.drawMarker(frame, tuple(c1.astype(int)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

                side_px = (self.marker_side(corners[idx_0][0]) + self.marker_side(corners[idx_1][0])) / 2
                if side_px > 0:
                    mm = MARKER_SIZE_MM / side_px
                    self.mm_per_px = mm if self.mm_per_px is None else 0.9 * self.mm_per_px + 0.1 * mm

                rel_X = c1[0] - c0[0]
                rel_Y = c1[1] - c0[1]

                if self.reset_req:
                    local_baseline_X = rel_X
                    local_baseline_Y = rel_Y
                    self.history = []
                    self.smoothed = None
                    self.reset_req = False

                if local_baseline_X is None:
                    local_baseline_X = rel_X
                    local_baseline_Y = rel_Y

                # smooth out the detection jitter, then keep a point only when the jaw really moved
                move = (rel_X - local_baseline_X, rel_Y - local_baseline_Y)
                if self.smoothed is None:
                    self.smoothed = move
                else:
                    self.smoothed = tuple(s + TRAJ_SMOOTHING * (m - s) for s, m in zip(self.smoothed, move))
                move_X, move_Y = self.smoothed
                last_move = (move_X, move_Y)

                if not self.history or np.hypot(move_X - self.history[-1][0],
                                                move_Y - self.history[-1][1]) >= TRAJ_MIN_STEP_PX:
                    self.history.append((move_X, move_Y))

                if not self.is_recording:
                    if len(self.history) > self.max_history:
                        self.history.pop(0)

                if len(self.history) > 1:
                    pts = np.array(self.history) * self.zoom + (center_x, center_y)
                    # anti-aliased, drawn with 1/16 px precision (shift=4)
                    cv2.polylines(traj_canvas, [np.round(pts * 16).astype(np.int32)], False,
                                  (60, 220, 200), 2, cv2.LINE_AA, shift=4)

            self.draw_overlay(traj_canvas, last_move)

            frame = cv2.resize(frame, (400, 300))
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb_traj = cv2.cvtColor(traj_canvas, cv2.COLOR_BGR2RGB)

            if not self.frame_q.empty():
                try: self.frame_q.get_nowait()
                except queue.Empty: pass
            self.frame_q.put(rgb_frame)

            if not self.traj_q.empty():
                try: self.traj_q.get_nowait()
                except queue.Empty: pass
            self.traj_q.put(rgb_traj)

        cap.release()

    def stop(self):
        self.running = False


# --- APPLICATION ---
class App:
    def __init__(self):
        self.link = SerialLink(SERIAL_PORT)

        self.values = [50.0] * NUM_MOTORS   # current motor percentages
        self.selected = 0                   # motor targeted by the keyboard
        self._guard = False                 # True while we move sliders ourselves

        self.active_transition_id = None
        self.transition_progress = 1.0      # 0-1 progress of the running transition
        self.seq_state = "idle"             # idle | running | paused
        self.seq_name = ""
        self.seq_poses = []                 # target values of each step
        self.seq_step = -1                  # current step (-1 = moving to the closed start)
        self.seq_resume_scale = 1.0         # share of the step's time left when it was stopped


        self.is_recording = False
        self.app_mode = "LIVE"
        self.recorded = []                  # (camera frame, trajectory frame, timestamp)

        self.root = ttk.Window(themename="flatly")
        self.root.style.register_theme(ThemeDefinition(THEME_NAME, THEME_COLORS, mode="light"))
        self.root.style.theme_use(THEME_NAME)
        self.apply_style()
        self.root.title("Mandibot")

        self.linked_vars = [tk.BooleanVar(value=False) for _ in range(NUM_MOTORS)]
        self.entry_vars = [tk.StringVar(value="50") for _ in range(NUM_MOTORS)]
        self.units_var = tk.StringVar(value="px")

        self.build_header()
        self.build_main()
        self.build_shortcuts()

        # Open at the size the content needs (the camera + trajectory panels are fixed-size images)
        self.root.update_idletasks()
        width = min(self.root.winfo_reqwidth(), self.root.winfo_screenwidth() - 60)
        height = min(self.root.winfo_reqheight() + 20, self.root.winfo_screenheight() - 100)
        self.root.geometry(f"{width}x{height}")

        self.frame_queue = queue.Queue(maxsize=2)
        self.traj_queue = queue.Queue(maxsize=2)
        self.tracking_queue = queue.Queue(maxsize=5)
        self.video_thread = VideoCaptureThread(self.frame_queue, self.traj_queue, self.tracking_queue)
        self.video_thread.start()

        for i in range(NUM_MOTORS):
            self.set_motor(i, 50)
        self.select_motor(0)

        self.link.connect()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(15, self.poll_video_queue)
        self.root.after(SERIAL_FLUSH_MS, self.flush_serial)
        self.refresh_status()

    def run(self):
        self.root.mainloop()

    # ---------------------------------------------------------------- layout
    def apply_style(self):
        """One UI font everywhere, and small bold section headings."""
        families = set(tkfont.families(self.root))
        self.ui_font = next((f for f in UI_FONTS if f in families), "Helvetica")
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont", "TkCaptionFont", "TkTooltipFont"):
            tkfont.nametofont(name).configure(family=self.ui_font, size=10)
        self.root.style.configure("TLabelframe.Label", font=(self.ui_font, 9, "bold"))

    def make_title(self, parent):
        """The logo (res/logo.png) scaled to LOGO_HEIGHT; falls back to the app name as text."""
        try:
            img = Image.open(LOGO_PATH).convert("RGBA")
            img = img.crop(img.getchannel("A").getbbox())  # drop the transparent margin
            width = round(img.width * LOGO_HEIGHT / img.height)
            self.title_photo = ImageTk.PhotoImage(img.resize((width, LOGO_HEIGHT), Image.LANCZOS))
            return ttk.Label(parent, image=self.title_photo)
        except Exception:
            return ttk.Label(parent, text=APP_NAME, font=(self.ui_font, 40, "bold"), bootstyle=PRIMARY)

    def make_icon(self, kind, size=28):
        """Small vector-style icon ('speaker' or 'stop') in the primary colour, drawn with PIL."""
        k = 4  # supersample for smooth edges
        mask = Image.new("L", (size * k, size * k), 0)
        d = ImageDraw.Draw(mask)
        u = size * k / 28
        if kind == "speaker":
            d.rectangle([4 * u, 11 * u, 9 * u, 17 * u], fill=255)
            d.polygon([(9 * u, 11 * u), (15 * u, 6 * u), (15 * u, 22 * u), (9 * u, 17 * u)], fill=255)
            d.arc([11 * u, 9 * u, 21 * u, 19 * u], -55, 55, fill=255, width=int(2 * u))
            d.arc([10 * u, 4 * u, 26 * u, 24 * u], -55, 55, fill=255, width=int(2 * u))
        else:
            d.rounded_rectangle([7 * u, 7 * u, 21 * u, 21 * u], radius=2 * u, fill=255)
        mask = mask.resize((size, size), Image.LANCZOS)
        color = ImageColor.getrgb(self.root.style.colors.primary)
        img = Image.new("RGBA", (size, size), color + (0,))
        img.putalpha(mask)
        return ImageTk.PhotoImage(img)

    def build_header(self):
        header = ttk.Frame(self.root)
        header.pack(side="top", fill="x", padx=20, pady=(10, 0))

        self.make_title(header).pack(side="left")

        right = ttk.Frame(header)
        right.pack(side="right")

        # Audio (top right): plays the track on the module's SD card
        self.audio_playing = False
        self.icon_speaker = self.make_icon("speaker")
        self.icon_stop = self.make_icon("stop")
        self.btn_audio = ttk.Button(right, image=self.icon_speaker, bootstyle=(SECONDARY, OUTLINE),
                                    command=self.toggle_audio)
        self.btn_audio.pack(side="right", padx=(10, 0))
        try:
            ToolTip(self.btn_audio, text="Play / stop the audio track (SD card)")
        except Exception:
            pass
        ttk.Button(right, text="■ EMERGENCY STOP (Esc)", bootstyle=DANGER,
                   command=self.emergency_stop).pack(side="right")
        ttk.Button(right, text="Reconnect", bootstyle=(SECONDARY, OUTLINE),
                   command=self.link.connect).pack(side="right", padx=10)
        self.status_label = ttk.Label(right, text="", font=(self.ui_font, 11, "bold"))
        self.status_label.pack(side="right")

        ttk.Separator(self.root).pack(side="top", fill="x", padx=20, pady=(8, 0))

    def build_main(self):
        footer = ttk.Frame(self.root)
        footer.pack(side="bottom", fill="x", padx=20, pady=(0, 8))
        ttk.Separator(footer).pack(fill="x", pady=(0, 6))
        hint = ("Space  stop / resume sequence   ·   Esc  emergency stop   ·   R  record   ·   "
                "←/→  select motor   ·   ↑/↓  nudge (Shift ×10)   ·   Link  tick motors to move them together")
        ttk.Label(footer, text=hint, font=(self.ui_font, 9), bootstyle=SECONDARY).pack(side="left")

        main_pane = ttk.Frame(self.root)
        main_pane.pack(fill=BOTH, expand=True, padx=20, pady=10)
        main_pane.columnconfigure(0, weight=1)
        main_pane.rowconfigure(0, weight=3, uniform="main")
        main_pane.rowconfigure(1, weight=2, uniform="main")

        top_pane = ttk.Frame(main_pane)
        top_pane.grid(row=0, column=0, sticky="nsew")
        top_pane.rowconfigure(0, weight=1)
        top_pane.columnconfigure(0, weight=1, uniform="top")
        top_pane.columnconfigure(1, weight=2, uniform="top")

        self.build_sequences(top_pane)
        self.build_tracking(top_pane)
        self.build_sliders(main_pane)

    def build_sequences(self, parent):
        top_left = ttk.Labelframe(parent, text=" SEQUENCES ", padding=15)
        top_left.grid(row=0, column=0, sticky="nsew")

        time_frame = ttk.Frame(top_left)
        time_frame.pack(fill="x", pady=(0, 15))
        ttk.Label(time_frame, text="Transition Time (s): ", font=(self.ui_font, 10)).pack(side="left")

        self.time_slider = ttk.Scale(time_frame, from_=0.5, to=5.0, orient="horizontal", bootstyle=SUCCESS)
        self.time_slider.set(1.5)
        self.time_slider.pack(side="left", expand=True, fill="x", padx=10)

        time_val_lbl = ttk.Label(time_frame, text="1.5s", font=(self.ui_font, 10, "bold"), width=4)
        time_val_lbl.pack(side="left")
        self.time_slider.configure(command=lambda val: time_val_lbl.config(text=f"{float(val):.1f}s"))

        # Sequence progress + stop (packed before the button grid so it stays at the bottom)
        seq_frame = ttk.Frame(top_left)
        seq_frame.pack(side="bottom", fill="x", pady=(10, 0))
        self.seq_label = ttk.Label(seq_frame, text="No sequence running", font=(self.ui_font, 10))
        self.seq_label.pack(anchor="w")
        seq_row = ttk.Frame(seq_frame)
        seq_row.pack(fill="x", pady=(4, 0))
        self.seq_bar = ttk.Progressbar(seq_row, maximum=1, value=0, bootstyle=SUCCESS)
        self.seq_bar.pack(side="left", expand=True, fill="x", padx=(0, 8))
        self.btn_seq_resume = ttk.Button(seq_row, text="▶ Resume", bootstyle=SUCCESS,
                                         command=self.resume_sequence, state=DISABLED)
        self.btn_seq_resume.pack(side="right")
        self.btn_seq_stop = ttk.Button(seq_row, text="■ Stop (Space)", bootstyle=WARNING,
                                       command=self.stop_sequence, state=DISABLED)
        self.btn_seq_stop.pack(side="right", padx=(0, 6))

        btn_frame = ttk.Frame(top_left)
        btn_frame.pack(expand=True, fill="both")

        buttons = []
        for btn_name, target_values in PRESET_POSITIONS.items():
            buttons.append((btn_name, BUTTON_STYLES[btn_name], lambda vals=target_values: self.start_transition(vals)))
            # Combined opening (rotation then translation) goes right after Phase 2
            if btn_name == OPEN_SEQUENCE[-1]:
                buttons.append(("Open (Rot + Tra)", BUTTON_STYLES["Open (Rot + Tra)"], self.start_open_sequence))
        buttons.append(("▶ Mastication Cycle", SUCCESS, self.start_mastication_cycle))

        for n, (text, style, command) in enumerate(buttons):
            btn = ttk.Button(btn_frame, text=text, bootstyle=style, command=command)
            btn.grid(row=n // 3, column=n % 3, padx=5, pady=5, sticky="nsew", ipady=5)

        for i in range(3): btn_frame.columnconfigure(i, weight=1)
        for i in range(4): btn_frame.rowconfigure(i, weight=1)

    def build_tracking(self, parent):
        top_right = ttk.Labelframe(parent, text=" TRACKING ", padding=10)
        top_right.grid(row=0, column=1, sticky="nsew")

        media_pane = ttk.Frame(top_right)
        media_pane.pack(side="top", expand=True, fill="both", pady=(0, 10))
        media_pane.rowconfigure(0, weight=1)
        media_pane.columnconfigure(0, weight=1, uniform="media")
        media_pane.columnconfigure(1, weight=1, uniform="media")

        vid_container = ttk.Labelframe(media_pane, text=" CAMERA ", padding=5)
        vid_container.grid(row=0, column=0, sticky="nsew")
        self.video_label = ttk.Label(vid_container)
        self.video_label.pack(expand=True, anchor="center")

        traj_container = ttk.Labelframe(media_pane, text=" TRAJECTORY ", padding=5)
        traj_container.grid(row=0, column=1, sticky="nsew")
        self.traj_label = ttk.Label(traj_container)
        self.traj_label.pack(expand=True, anchor="center")

        controls = ttk.Frame(top_right)
        controls.pack(side="bottom", fill="x")

        self.scrub_slider = ttk.Scale(controls, from_=0, to=100, orient="horizontal", command=self.on_scrub_move)
        self.scrub_slider.pack(fill="x", pady=5)
        self.scrub_slider.config(state=DISABLED)

        row1 = ttk.Frame(controls)
        row1.pack(fill="x", pady=2)
        ttk.Button(row1, text="Set Center", bootstyle=INFO, command=self.reset_baseline).pack(expand=True, fill="x")

        row2 = ttk.Frame(controls)
        row2.pack(fill="x", pady=2)
        self.btn_record = ttk.Button(row2, text="Start Recording (R)", bootstyle=DANGER, command=self.toggle_record)
        self.btn_record.pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(row2, text="Back to Live", bootstyle=WARNING, command=self.set_live).pack(
            side="right", expand=True, fill="x", padx=(2, 0))

        row3 = ttk.Frame(controls)
        row3.pack(fill="x", pady=2)
        self.btn_export = ttk.Button(row3, text="Export MP4", bootstyle=SUCCESS, command=self.export_mp4,
                                     state=DISABLED)
        self.btn_export.pack(side="left", expand=True, fill="x", padx=(0, 8))
        ttk.Label(row3, text="Units:").pack(side="left", padx=(0, 4))
        for unit in ("px", "mm"):
            ttk.Radiobutton(row3, text=unit, value=unit, variable=self.units_var,
                            command=self.on_units_change).pack(side="left", padx=2)

    def build_sliders(self, parent):
        bottom_frame = ttk.Labelframe(parent, text=" MOTOR CONTROL ", padding=(10, 5))
        bottom_frame.grid(row=1, column=0, sticky="nsew")

        slider_frame = ttk.Frame(bottom_frame)
        slider_frame.pack(expand=True, fill="both")

        self.sliders = []
        self.entries = []
        self.name_labels = []

        for i in range(NUM_MOTORS):
            col = ttk.Frame(slider_frame)
            col.pack(side="left", expand=True, fill="y")

            val_row = ttk.Frame(col)
            val_row.pack(pady=(4, 2))
            entry = ttk.Entry(val_row, textvariable=self.entry_vars[i], width=4, justify="center",
                              font=(self.ui_font, 11, "bold"))
            entry.pack(side="left")
            ttk.Label(val_row, text="%", font=(self.ui_font, 11, "bold")).pack(side="left")
            entry.bind("<Return>", lambda e, i=i: self.commit_entry(i, refocus=True))
            entry.bind("<FocusOut>", lambda e, i=i: self.commit_entry(i))
            entry.bind("<FocusIn>", lambda e, i=i: self.select_motor(i))

            slider = ttk.Scale(col, from_=100, to=0, orient="vertical", length=150, bootstyle=INFO,
                               takefocus=False,
                               command=lambda val, i=i: self.on_slider_user(i, val))
            slider.pack(pady=4)
            slider.bind("<ButtonPress-1>", lambda e, i=i: self.on_slider_press(i), add="+")

            name_label = ttk.Label(col, text=MOTOR_NAMES[i], justify="center", font=(self.ui_font, 9))
            name_label.pack(pady=(2, 2))

            ttk.Checkbutton(col, text="Link", variable=self.linked_vars[i]).pack(pady=(0, 4))

            self.sliders.append(slider)
            self.entries.append(entry)
            self.name_labels.append(name_label)

    def build_shortcuts(self):
        def guarded(fn):
            # Plain-letter shortcuts must not fire while typing in a numeric entry
            def handler(event):
                if self.root.focus_get() is not None and self.root.focus_get().winfo_class() in ("TEntry", "Entry"):
                    return
                fn()
            return handler

        self.root.bind("<space>", guarded(self.toggle_sequence_pause))
        self.root.bind("<Escape>", lambda e: self.emergency_stop())
        self.root.bind("<Key-r>", guarded(self.toggle_record))
        self.root.bind("<Key-R>", guarded(self.toggle_record))
        self.root.bind("<Left>", guarded(lambda: self.select_motor((self.selected - 1) % NUM_MOTORS)))
        self.root.bind("<Right>", guarded(lambda: self.select_motor((self.selected + 1) % NUM_MOTORS)))
        self.root.bind("<Up>", guarded(lambda: self.nudge(1)))
        self.root.bind("<Down>", guarded(lambda: self.nudge(-1)))
        self.root.bind("<Shift-Up>", guarded(lambda: self.nudge(10)))
        self.root.bind("<Shift-Down>", guarded(lambda: self.nudge(-10)))

    # ----------------------------------------------------------- connection
    def refresh_status(self):
        link = self.link
        if link.state == "connected":
            text, style = f"● Connected: {link.port}", SUCCESS
        elif link.state == "connecting":
            text, style = "● Connecting…", WARNING
        else:
            text = "● Test mode: no Arduino"
            if link.error:
                text += f" ({link.error})"
            style = DANGER
        self.status_label.config(text=text, bootstyle=style)
        if link.state != "connected" and self.audio_playing:  # the Arduino reset or went away
            self.audio_playing = False
            self.btn_audio.config(image=self.icon_speaker)
        self.root.after(300, self.refresh_status)

    def toggle_audio(self):
        """Click to play the SD track; click again to stop (the GUI can't tell when the track ends)."""
        command = "STOP" if self.audio_playing else "PLAY"
        if not self.link.send_line(command):
            messagebox.showinfo("Audio", "Connect the Arduino first to play audio.", parent=self.root)
            return
        self.audio_playing = not self.audio_playing
        self.btn_audio.config(image=self.icon_stop if self.audio_playing else self.icon_speaker)

    def flush_serial(self):
        self.link.flush()
        self.root.after(SERIAL_FLUSH_MS, self.flush_serial)

    # --------------------------------------------------------------- motors
    def set_motor(self, i, val):
        """Move motor i (slider, readout and serial command) without any pair linking."""
        val = max(0.0, min(100.0, float(val)))
        self.values[i] = val
        self._guard = True
        try:
            self.sliders[i].set(val)
        finally:
            self._guard = False
        self.update_readout(i)
        self.link.set_target(i, val)

    def update_readout(self, i):
        if self.root.focus_get() is not self.entries[i]:  # don't overwrite what is being typed
            self.entry_vars[i].set(str(int(round(self.values[i]))))

    def user_set_motor(self, i, val):
        """User-driven change. All motors with Link ticked move together (keeping their
        offsets) when one of them is moved; an unticked motor moves on its own."""
        if self.linked_vars[i].get():
            group = [m for m in range(NUM_MOTORS) if self.linked_vars[m].get()]
        else:
            group = [i]
        delta = float(val) - self.values[i]
        # move the group as a block: stop when the first motor reaches 0 or 100
        delta = max(max(-self.values[m] for m in group), min(min(100 - self.values[m] for m in group), delta))
        for m in group:
            self.set_motor(m, self.values[m] + delta)

    def on_slider_user(self, i, val):
        if not self._guard:
            self.user_set_motor(i, val)

    def on_slider_press(self, i):
        self.root.focus_set()  # leave any numeric entry
        self.select_motor(i)

    def commit_entry(self, i, refocus=False):
        try:
            val = float(self.entry_vars[i].get())
        except ValueError:
            val = None
        if val is not None:
            self.user_set_motor(i, val)
        if refocus:
            self.root.focus_set()
        self.update_readout(i)

    def select_motor(self, i):
        self.selected = i
        for n, label in enumerate(self.name_labels):
            label.config(bootstyle="inverse-primary" if n == i else "default")

    def nudge(self, step):
        self.user_set_motor(self.selected, self.values[self.selected] + step)

    # ---------------------------------------------------------- transitions
    def cancel_transition(self):
        if self.active_transition_id is not None:
            self.root.after_cancel(self.active_transition_id)
            self.active_transition_id = None

    def cancel_sequence(self, text="No sequence running"):
        self.seq_poses = []
        self.seq_bar.configure(value=0)
        self.set_seq_state("idle", text)

    def set_seq_state(self, state, text=None):
        self.seq_state = state
        if text is not None:
            self.seq_label.config(text=text)
        self.btn_seq_stop.config(state=NORMAL if state == "running" else DISABLED)
        self.btn_seq_resume.config(state=NORMAL if state == "paused" else DISABLED)

    def start_transition(self, target_vals, from_sequence=False, duration_scale=1.0, on_done=None):
        # If the user clicks a standard preset button, cancel any running sequence
        if not from_sequence:
            self.cancel_sequence()
        self.cancel_transition()

        start_vals = list(self.values)
        duration_sec = self.time_slider.get() * duration_scale

        update_interval_ms = 33
        total_steps = max(1, int((duration_sec * 1000) / update_interval_ms))

        self.transition_progress = 0.0
        self.animate_step(start_vals, target_vals, 0, total_steps, update_interval_ms, on_done)

    def animate_step(self, start_vals, target_vals, current_step, total_steps, interval, on_done=None):
        if current_step >= total_steps:
            for i, target in enumerate(target_vals):
                self.set_motor(i, target)
            self.active_transition_id = None
            self.transition_progress = 1.0
            if on_done:
                on_done()
            return

        progress = current_step / total_steps
        self.transition_progress = progress
        for i in range(NUM_MOTORS):
            start = start_vals[i]
            target = target_vals[i]
            self.set_motor(i, start + (target - start) * progress)

        self.active_transition_id = self.root.after(
            interval, self.animate_step, start_vals, target_vals, current_step + 1, total_steps, interval, on_done)

    # ------------------------------------------------------------ sequences
    def run_sequence(self, poses, name):
        self.cancel_transition()
        self.seq_name = name
        self.seq_poses = [PRESET_POSITIONS[p] if isinstance(p, str) else p for p in poses]
        self.seq_bar.configure(maximum=len(self.seq_poses), value=0)
        self.set_seq_state("running")
        self.begin_step(-1)

    def begin_step(self, step, scale=1.0):
        """Run one step (-1 = move to the closed start), then chain to the next one."""
        total = len(self.seq_poses)
        if step >= total:
            self.seq_bar.configure(value=total)
            self.set_seq_state("idle", f"{self.seq_name}: done")
            return
        self.seq_step = step
        if step < 0:
            target_vals = PRESET_POSITIONS["Fully Closed"]
            text = f"{self.seq_name}: moving to closed start position"
        else:
            target_vals = self.seq_poses[step]
            text = f"{self.seq_name}: step {step + 1}/{total}"
        self.seq_label.config(text=text)
        self.seq_bar.configure(value=max(step, 0))
        self.start_transition(target_vals, from_sequence=True, duration_scale=scale,
                              on_done=lambda: self.begin_step(step + 1))

    def start_open_sequence(self):
        self.run_sequence(OPEN_SEQUENCE, "Open (Rot + Tra)")

    def start_mastication_cycle(self):
        self.run_sequence(MASTICATION_CYCLE * MASTICATION_REPEATS, "Mastication Cycle")

    def stop_sequence(self):
        """Halt where the motors are; a running sequence can then be resumed."""
        if self.seq_state != "running":
            self.cancel_transition()
            return
        self.seq_resume_scale = max(0.05, 1.0 - self.transition_progress)
        self.cancel_transition()
        where = "the closed start" if self.seq_step < 0 else f"step {self.seq_step + 1}/{len(self.seq_poses)}"
        self.set_seq_state("paused", f"{self.seq_name}: stopped during {where}. Press Resume to continue.")

    def resume_sequence(self):
        """Continue the interrupted step from the current position, then the rest."""
        if self.seq_state != "paused":
            return
        self.set_seq_state("running")
        self.begin_step(self.seq_step, self.seq_resume_scale)

    def toggle_sequence_pause(self):
        if self.seq_state == "paused":
            self.resume_sequence()
        else:
            self.stop_sequence()

    def emergency_stop(self):
        """Halt everything (not resumable) and send every motor straight to 50%."""
        self.cancel_transition()
        self.cancel_sequence(text="EMERGENCY STOP: motors reset to 50%")
        for i in range(NUM_MOTORS):
            self.set_motor(i, 50)
        self.link.flush()

    # ------------------------------------------------------------- tracking
    def reset_baseline(self):
        self.tracking_queue.put("RESET_BASELINE")

    def on_units_change(self):
        self.video_thread.units = self.units_var.get()

    def toggle_record(self):
        if not self.is_recording:
            self.is_recording = True
            self.app_mode = "LIVE"
            self.recorded = []
            self.tracking_queue.put("START_RECORD")
            self.btn_record.config(text="Stop & Review (R)", bootstyle=SECONDARY)
            self.btn_export.config(state=DISABLED)
            self.scrub_slider.config(state=DISABLED)
        else:
            self.is_recording = False
            self.app_mode = "PLAYBACK"
            self.tracking_queue.put("STOP_RECORD")
            self.btn_record.config(text="Start Recording (R)", bootstyle=DANGER)

            if len(self.recorded) > 0:
                self.scrub_slider.config(state=NORMAL, to=len(self.recorded) - 1)
                self.scrub_slider.set(0)
                self.btn_export.config(state=NORMAL)

    def set_live(self):
        self.app_mode = "LIVE"
        self.is_recording = False
        self.tracking_queue.put("STOP_RECORD")
        self.btn_record.config(text="Start Recording (R)", bootstyle=DANGER)
        self.scrub_slider.config(state=DISABLED)

    def on_scrub_move(self, val):
        if self.app_mode == "PLAYBACK" and len(self.recorded) > 0:
            vid, traj, _ = self.recorded[int(float(val))]
            self.update_image_labels(vid, traj)

    def update_image_labels(self, rgb_vid, rgb_traj):
        img_v = Image.fromarray(rgb_vid)
        imgtk_v = ImageTk.PhotoImage(image=img_v)
        self.video_label.imgtk = imgtk_v
        self.video_label.configure(image=imgtk_v)

        img_t = Image.fromarray(rgb_traj)
        imgtk_t = ImageTk.PhotoImage(image=img_t)
        self.traj_label.imgtk = imgtk_t
        self.traj_label.configure(image=imgtk_t)

    def poll_video_queue(self):
        vid_frame, traj_frame = None, None
        try:
            vid_frame = self.frame_queue.get_nowait()
        except queue.Empty: pass

        try:
            traj_frame = self.traj_queue.get_nowait()
        except queue.Empty: pass

        if self.app_mode == "LIVE":
            if vid_frame is not None and traj_frame is not None:
                if self.is_recording:
                    # Store clean frames (for the MP4); the live preview gets the overlay
                    self.recorded.append((vid_frame.copy(), traj_frame.copy(), time.monotonic()))
                    vid_show = vid_frame.copy()
                    cv2.putText(vid_show, "RECORDING...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    self.update_image_labels(vid_show, traj_frame)
                else:
                    self.update_image_labels(vid_frame, traj_frame)

        self.root.after(15, self.poll_video_queue)

    @staticmethod
    def compose_frame(record):
        """Camera feed and trajectory side by side, as a BGR frame for OpenCV."""
        vid, traj, _ = record
        return cv2.cvtColor(np.hstack([vid, traj]), cv2.COLOR_RGB2BGR)

    def export_mp4(self):
        if not self.recorded:
            return
        path = filedialog.asksaveasfilename(
            parent=self.root, title="Export recording", defaultextension=".mp4",
            initialfile=datetime.now().strftime("mandibot_%Y%m%d_%H%M%S.mp4"),
            filetypes=[("MP4 video", "*.mp4")])
        if not path:
            return

        duration = self.recorded[-1][2] - self.recorded[0][2]
        fps = len(self.recorded) / duration if duration > 0 else 30.0
        fps = max(1.0, min(60.0, fps))

        height, width = self.compose_frame(self.recorded[0]).shape[:2]
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
        if not writer.isOpened():
            messagebox.showerror("Export failed", "OpenCV could not open an MP4 writer on this system.")
            return
        try:
            for record in self.recorded:
                writer.write(self.compose_frame(record))
        finally:
            writer.release()
        messagebox.showinfo("Export", f"Saved {len(self.recorded)} frames ({len(self.recorded) / fps:.1f}s) to\n{path}")

    # ---------------------------------------------------------------- close
    def on_close(self):
        self.video_thread.stop()
        self.video_thread.join(timeout=1)
        self.link.close()
        self.root.destroy()


if __name__ == "__main__":
    App().run()
