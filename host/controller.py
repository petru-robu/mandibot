import ttkbootstrap as ttk
from ttkbootstrap.constants import *
import serial
import time
import cv2
import numpy as np
import threading
import queue
import os
from PIL import Image, ImageTk

# --- CONFIGURATION ---
SERIAL_PORT = "/dev/ttyUSB0"  
BAUD_RATE = 115200

MOTOR_NAMES = [
    "TEMP STG\n[0]", "TEMP DRP\n[1]", 
    "MAS STG\n[2]", "MAS DRP\n[3]", 
    "LAT STG\n[4]", "LAT DRP\n[5]", 
    "MED STG\n[6]", "MED DRP\n[7]"
]

PRESET_POSITIONS = {
    "Fully Closed":        [100, 100, 100, 100, 10, 10, 100, 100],
    "Open: Phase 1 (Rot)": [25, 25, 25, 25, 10, 10, 25, 25],
    "Open: Phase 2 (Tra)": [20, 20, 15, 15, 100, 100, 15, 15],
    "Max Protrusion":      [0, 0, 30, 30, 100, 100, 40, 40],
    "Retrusion":           [100, 100, 80, 80, 0, 0, 80, 80],
    "Left Laterality":     [90, 45, 85, 45, 10, 100, 30, 65],
    "Right Laterality":    [45, 90, 45, 85, 100, 10, 65, 30]
}

try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) 
    print(f"Successfully connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Error connecting to {SERIAL_PORT}: {e}")
    print("Running in UI-only test mode (no data will be sent).")
    arduino = None

active_transition_id = None
active_sequence_ids = []  # Tracks queued steps for ANY automated cycle

def on_slider_change(val, motor_index):
    val = int(float(val))
    if arduino and arduino.is_open:
        command = f"{motor_index}:{val}\n"
        arduino.write(command.encode('utf-8'))

def start_transition(target_vals, from_sequence=False):
    global active_transition_id, active_sequence_ids
    
    # If the user clicks a standard preset button, cancel any running sequence
    if not from_sequence:
        for seq_id in active_sequence_ids:
            root.after_cancel(seq_id)
        active_sequence_ids.clear()

    if active_transition_id is not None:
        root.after_cancel(active_transition_id)
    
    start_vals = [s.get() for s in sliders]
    duration_sec = time_slider.get()
    
    update_interval_ms = 33 
    total_steps = max(1, int((duration_sec * 1000) / update_interval_ms))
    
    animate_step(start_vals, target_vals, 0, total_steps, update_interval_ms)

def animate_step(start_vals, target_vals, current_step, total_steps, interval):
    global active_transition_id
    
    if current_step >= total_steps:
        for i, target in enumerate(target_vals):
            sliders[i].set(target)
            val_labels[i].config(text=f"{int(target)}%")
        active_transition_id = None
        return

    progress = current_step / total_steps
    for i in range(8):
        start = start_vals[i]
        target = target_vals[i]
        current_val = start + (target - start) * progress
        sliders[i].set(current_val)
        val_labels[i].config(text=f"{int(current_val)}%")
        
    active_transition_id = root.after(interval, animate_step, start_vals, target_vals, current_step + 1, total_steps, interval)

def start_mastication_cycle():
    global active_sequence_ids
    
    for seq_id in active_sequence_ids:
        root.after_cancel(seq_id)
    active_sequence_ids.clear()
    
    # Biomechanical cycle: Drop -> Shift -> Close (Grind) -> Drop -> Shift -> Close
    cycle = [
        "Open: Phase 1 (Rot)",
        "Open: Phase 2 (Tra)",
        "Right Laterality",
        "Fully Closed",
        "Open: Phase 1 (Rot)",
        "Open: Phase 2 (Tra)",
        "Left Laterality",
        "Fully Closed"
    ]
    
    duration_ms = int(time_slider.get() * 1000)
    
    start_transition(PRESET_POSITIONS["Fully Closed"], from_sequence=True)
    
    for i, pose_name in enumerate(cycle):
        target_vals = PRESET_POSITIONS[pose_name]
        seq_id = root.after((i + 1) * duration_ms, start_transition, target_vals, True)
        active_sequence_ids.append(seq_id)

def start_posselt_envelope():
    global active_sequence_ids
    
    for seq_id in active_sequence_ids:
        root.after_cancel(seq_id)
    active_sequence_ids.clear()
    
    # Posselt's Envelope: Sagittal border movements
    # Start closed -> Retrude -> Rotate open -> Translate open -> Close protruding -> Slide back
    envelope = [
        "Retrusion",           # Centric Relation
        "Open: Phase 1 (Rot)", # Terminal Hinge Axis opening
        "Open: Phase 2 (Tra)", # Maximal Opening
        "Max Protrusion",      # Anterior border path
        "Fully Closed"         # Slide back to Maximum Intercuspation
    ]
    
    duration_ms = int(time_slider.get() * 1000)
    
    start_transition(PRESET_POSITIONS["Fully Closed"], from_sequence=True)
    
    for i, pose_name in enumerate(envelope):
        target_vals = PRESET_POSITIONS[pose_name]
        seq_id = root.after((i + 1) * duration_ms, start_transition, target_vals, True)
        active_sequence_ids.append(seq_id)

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
        self.traj_size = (450, 300) 
        self.zoom = 3
        
        self.is_recording = False
        self.reset_req = False

    def run(self):
        cap = cv2.VideoCapture(1) 
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
            
            traj_canvas = np.zeros((self.traj_size[1], self.traj_size[0], 3), dtype=np.uint8)
            center_x, center_y = self.traj_size[0] // 2, self.traj_size[1] // 2
            
            cv2.line(traj_canvas, (0, center_y), (self.traj_size[0], center_y), (40, 40, 40), 1)
            cv2.line(traj_canvas, (center_x, 0), (center_x, self.traj_size[1]), (40, 40, 40), 1)
            
            if ids is not None and len(ids) >= 2 and 0 in ids and 1 in ids:
                idx_0 = np.where(ids == 0)[0][0]
                idx_1 = np.where(ids == 1)[0][0]
                c0 = np.mean(corners[idx_0][0], axis=0).astype(int) 
                c1 = np.mean(corners[idx_1][0], axis=0).astype(int) 
                
                cv2.drawMarker(frame, tuple(c0), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.drawMarker(frame, tuple(c1), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                
                rel_X = c1[0] - c0[0]
                rel_Y = c1[1] - c0[1]

                if self.reset_req:
                    local_baseline_X = rel_X
                    local_baseline_Y = rel_Y
                    self.history = []
                    self.reset_req = False

                if local_baseline_X is None:
                    local_baseline_X = rel_X
                    local_baseline_Y = rel_Y

                move_X = rel_X - local_baseline_X
                move_Y = rel_Y - local_baseline_Y
                
                self.history.append((move_X, move_Y))
                
                if not self.is_recording:
                    if len(self.history) > self.max_history:
                        self.history.pop(0)

                if len(self.history) > 1:
                    for i in range(1, len(self.history)):
                        pt_start = (
                            int(self.history[i-1][0] * self.zoom) + center_x,
                            int(self.history[i-1][1] * self.zoom) + center_y
                        )
                        pt_end = (
                            int(self.history[i][0] * self.zoom) + center_x,
                            int(self.history[i][1] * self.zoom) + center_y
                        )
                        cv2.line(traj_canvas, pt_start, pt_end, (0, 255, 255), 2)
            
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


# --- GUI STATE & FUNCTIONS ---
is_recording = False
app_mode = "LIVE"
recorded_data = []

def reset_baseline():
    tracking_queue.put("RESET_BASELINE")

def toggle_record():
    global is_recording, app_mode, recorded_data
    if not is_recording:
        is_recording = True
        app_mode = "LIVE"
        recorded_data = []
        tracking_queue.put("START_RECORD")
        btn_record.config(text="Stop & Review", bootstyle=SECONDARY)
        scrub_slider.config(state=DISABLED)
    else:
        is_recording = False
        app_mode = "PLAYBACK"
        tracking_queue.put("STOP_RECORD")
        btn_record.config(text="Start Recording", bootstyle=DANGER)
        
        if len(recorded_data) > 0:
            scrub_slider.config(state=NORMAL, to=len(recorded_data)-1)
            scrub_slider.set(0)

def set_live():
    global app_mode, is_recording
    app_mode = "LIVE"
    is_recording = False
    tracking_queue.put("STOP_RECORD")
    btn_record.config(text="Start Recording", bootstyle=DANGER)
    scrub_slider.config(state=DISABLED)

def on_scrub_move(val):
    if app_mode == "PLAYBACK" and len(recorded_data) > 0:
        idx = int(float(val))
        vid, traj = recorded_data[idx]
        update_image_labels(vid, traj)

def update_image_labels(rgb_vid, rgb_traj):
    img_v = Image.fromarray(rgb_vid)
    imgtk_v = ImageTk.PhotoImage(image=img_v)
    video_label.imgtk = imgtk_v
    video_label.configure(image=imgtk_v)

    img_t = Image.fromarray(rgb_traj)
    imgtk_t = ImageTk.PhotoImage(image=img_t)
    traj_label.imgtk = imgtk_t
    traj_label.configure(image=imgtk_t)

def poll_video_queue():
    vid_frame, traj_frame = None, None
    try:
        vid_frame = frame_queue.get_nowait()
    except queue.Empty: pass
    
    try:
        traj_frame = traj_queue.get_nowait()
    except queue.Empty: pass

    if app_mode == "LIVE":
        if vid_frame is not None and traj_frame is not None:
            if is_recording:
                vid_copy = vid_frame.copy()
                cv2.putText(vid_copy, "RECORDING...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                recorded_data.append((vid_copy, traj_frame.copy()))
                update_image_labels(vid_copy, traj_frame)
            else:
                update_image_labels(vid_frame, traj_frame)

    root.after(15, poll_video_queue)

def on_close():
    video_thread.stop()
    video_thread.join(timeout=1)
    if arduino and arduino.is_open: arduino.close()
    root.destroy()


# --- SETUP THEMED UI ---
root = ttk.Window(themename="united") 
root.title("Mandible Servo Controller & Tracker")
root.geometry("1100x850") 

# --- VERY TOP LOGO LAYER ---
logo_frame = ttk.Frame(root)
logo_frame.pack(side="top", fill="x", pady=(10, 0))

logo_path = "./host/logo.png" 
if os.path.exists(logo_path):
    try:
        raw_image = Image.open(logo_path)
        aspect_ratio = raw_image.width / raw_image.height
        new_height = 50
        new_width = int(new_height * aspect_ratio)
        resized_image = raw_image.resize((new_width, new_height), Image.LANCZOS)
        logo_photo = ImageTk.PhotoImage(resized_image)
        logo_label = ttk.Label(logo_frame, image=logo_photo)
        logo_label.image = logo_photo 
    except Exception:
        logo_label = ttk.Label(logo_frame, text="[ LOGO ERROR ]", font=("Helvetica", 16, "bold"))
else:
    logo_label = ttk.Label(logo_frame, text="[ PLACE LOGO HERE ]", font=("Helvetica", 16, "bold"))
logo_label.pack(expand=True, anchor="center")

# ==============================================================================
# MAIN PANED WINDOW (VERTICAL DRAGGABLE WALL BETWEEN TOP AND BOTTOM)
# ==============================================================================
main_pane = ttk.Panedwindow(root, orient=VERTICAL)
main_pane.pack(fill=BOTH, expand=True, padx=20, pady=10)

# ==============================================================================
# TOP PANED WINDOW: Sequences & Tracking
# ==============================================================================
top_pane = ttk.Panedwindow(main_pane, orient=HORIZONTAL)
main_pane.add(top_pane, weight=2)

# Top Left: Transitions
top_left = ttk.Labelframe(top_pane, text=" Biomechanics Sequences ", padding=15)
top_pane.add(top_left, weight=1)

time_frame = ttk.Frame(top_left)
time_frame.pack(fill="x", pady=(0, 15))
ttk.Label(time_frame, text="Transition Time (s): ", font=("Helvetica", 10)).pack(side="left")

time_slider = ttk.Scale(time_frame, from_=0.5, to=5.0, orient="horizontal", bootstyle=SUCCESS)
time_slider.set(2.0)
time_slider.pack(side="left", expand=True, fill="x", padx=10)

time_val_lbl = ttk.Label(time_frame, text="2.0s", font=("Helvetica", 10, "bold"), width=4)
time_val_lbl.pack(side="left")
time_slider.configure(command=lambda val: time_val_lbl.config(text=f"{float(val):.1f}s"))

btn_frame = ttk.Frame(top_left)
btn_frame.pack(expand=True, fill="both")

row_idx, col_idx = 0, 0
for btn_name, target_values in PRESET_POSITIONS.items():
    btn = ttk.Button(btn_frame, text=btn_name, bootstyle=(PRIMARY, OUTLINE),
                     command=lambda vals=target_values: start_transition(vals))
    btn.grid(row=row_idx, column=col_idx, padx=5, pady=5, sticky="nsew", ipady=5)
    
    col_idx += 1
    if col_idx > 2:  
        col_idx = 0
        row_idx += 1

# Mastication Cycle Button
btn_cycle = ttk.Button(btn_frame, text="▶ Mastication Cycle", bootstyle=(SUCCESS, OUTLINE),
                       command=start_mastication_cycle)
btn_cycle.grid(row=row_idx, column=col_idx, padx=5, pady=5, sticky="nsew", ipady=5)

col_idx += 1
if col_idx > 2:  
    col_idx = 0
    row_idx += 1

# Posselt Envelope Button
btn_posselt = ttk.Button(btn_frame, text="▶ Posselt's Envelope", bootstyle=(INFO, OUTLINE),
                         command=start_posselt_envelope)
btn_posselt.grid(row=row_idx, column=col_idx, padx=5, pady=5, sticky="nsew", ipady=5)

for i in range(3): btn_frame.columnconfigure(i, weight=1)
for i in range(3): btn_frame.rowconfigure(i, weight=1)

# Top Right: VIDEO, TRAJ & CONTROLS
top_right = ttk.Labelframe(top_pane, text=" Tracking & Trajectory ", padding=10)
top_pane.add(top_right, weight=2)

# --- MEDIA PANED WINDOW (HORIZONTAL DRAGGABLE WALL BETWEEN CAMERA AND TRAJECTORY) ---
media_pane = ttk.Panedwindow(top_right, orient=HORIZONTAL)
media_pane.pack(side="top", expand=True, fill="both", pady=(0, 10))

# Sub-container for Video
vid_container = ttk.Labelframe(media_pane, text="Camera Feed", padding=5)
media_pane.add(vid_container, weight=1)
video_label = ttk.Label(vid_container)
video_label.pack(expand=True, anchor="center")

# Sub-container for Trajectory
traj_container = ttk.Labelframe(media_pane, text="Trajectory Drawing", padding=5)
media_pane.add(traj_container, weight=1)
traj_label = ttk.Label(traj_container)
traj_label.pack(expand=True, anchor="center")

# Top Right Controls
controls_container = ttk.Frame(top_right)
controls_container.pack(side="bottom", fill="x")

# Timeline Slider
scrub_slider = ttk.Scale(controls_container, from_=0, to=100, orient="horizontal", command=on_scrub_move)
scrub_slider.pack(fill="x", pady=5)
scrub_slider.config(state=DISABLED)

# Control Rows
control_row1 = ttk.Frame(controls_container)
control_row1.pack(fill="x", pady=2)
btn_reset = ttk.Button(control_row1, text="Set Center", bootstyle=INFO, command=reset_baseline)
btn_reset.pack(expand=True, fill="x")

control_row2 = ttk.Frame(controls_container)
control_row2.pack(fill="x", pady=2)
btn_record = ttk.Button(control_row2, text="Start Recording", bootstyle=DANGER, command=toggle_record)
btn_record.pack(side="left", expand=True, fill="x", padx=(0, 2))
btn_live = ttk.Button(control_row2, text="Back to Live", bootstyle=WARNING, command=set_live)
btn_live.pack(side="right", expand=True, fill="x", padx=(2, 0))

# ==============================================================================
# BOTTOM PANED WINDOW: Sliders
# ==============================================================================
bottom_frame = ttk.Labelframe(main_pane, text=" Direct Motor Control ", padding=(20, 10))
main_pane.add(bottom_frame, weight=1)

slider_frame = ttk.Frame(bottom_frame)
slider_frame.pack(expand=True, fill="both")

sliders = []
val_labels = []

for i, name in enumerate(MOTOR_NAMES):
    col = ttk.Frame(slider_frame)
    col.pack(side="left", expand=True, fill="y")
    
    val_label = ttk.Label(col, text="100%", font=("Helvetica", 11, "bold"), bootstyle=INFO)
    val_label.pack(pady=(10, 5))
    val_labels.append(val_label)
    
    slider = ttk.Scale(col, from_=100, to=0, orient="vertical", length=180, bootstyle=INFO,
                      command=lambda val, idx=i, lbl=val_label: [
                          lbl.config(text=f"{int(float(val))}%"),
                          on_slider_change(val, idx)
                      ])
    slider.set(100)
    slider.pack(pady=10)
    
    name_label = ttk.Label(col, text=name, justify="center", font=("Helvetica", 9))
    name_label.pack(pady=(5, 10))
    
    sliders.append(slider)


# --- START TASKS ---
frame_queue = queue.Queue(maxsize=2)
traj_queue = queue.Queue(maxsize=2)
tracking_queue = queue.Queue(maxsize=5)

video_thread = VideoCaptureThread(frame_queue, traj_queue, tracking_queue)
video_thread.start()

root.protocol("WM_DELETE_WINDOW", on_close)
root.after(15, poll_video_queue) 
root.mainloop()