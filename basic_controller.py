# # import cv2
# # import numpy as np
# # import json
# # import math
# # import serial
# # import time
# # import tkinter as tk
# # from tkinter import ttk
# # import matplotlib.pyplot as plt
# # from threading import Thread
# # import queue
# # from ball_detection import detect_ball_x


# # class BasicPIDController:
# #     def __init__(self, config_file="config.json"):
# #         """Initialize controller, load config, set defaults and queues."""

# #         # Load experiment and hardware config from JSON file
# #         with open(config_file, 'r') as f:
# #             self.config = json.load(f)

# #         # ============================================================
# #         # ACTIVE PID PARAMETERS
# #         # These are the values actually being used by the controller.
# #         # They only change when "Apply Parameters" is pressed.
# #         # ============================================================
# #         self.Kp = 10.0
# #         self.Ki = 0.0
# #         self.Kd = 0.0
# #         self.setpoint = 0.0
# #         # Deadband: errors smaller than this (in meters) are treated as zero
# #         self.deadband_m = 0.00   # ±2 mm

# #         # ============================================================
# #         # PENDING PID PARAMETERS
# #         # These are changed by the GUI sliders / entry boxes.
# #         # They do NOT affect the controller until Apply is pressed.
# #         # ============================================================
# #         self.pending_Kp = self.Kp
# #         self.pending_Ki = self.Ki
# #         self.pending_Kd = self.Kd
# #         self.pending_setpoint = self.setpoint

# #         # Scale factor for converting from pixels to meters
# #         self.scale_factor = (
# #             self.config['calibration']['pixel_to_meter_ratio']
# #             * self.config['camera']['frame_width'] / 2
# #         )

# #         # Servo port name and center angle
# #         self.servo_port = self.config['servo']['port']
# #         self.neutral_angle = self.config['servo']['neutral_angle']
# #         self.servo = None

# #         # Controller-internal state
# #         self.integral = 0.0
# #         self.prev_error = 0.0

# #         # Data logs for plotting results
# #         self.time_log = []
# #         self.position_log = []
# #         self.setpoint_log = []
# #         self.control_log = []
# #         self.start_time = None

# #         # Thread-safe queue for most recent ball position measurement
# #         self.position_queue = queue.Queue(maxsize=1)

# #         # Main run flag for clean shutdown
# #         self.running = False

# #         # Keep Tk variables alive
# #         self._gui_vars = []

# #     # ================================================================
# #     # SERVO
# #     # ================================================================

# #     def connect_servo(self):
# #         """Try to open serial connection to servo."""
# #         try:
# #             self.servo = serial.Serial(self.servo_port, 9600)
# #             time.sleep(2)
# #             print("[SERVO] Connected")
# #             return True
# #         except Exception as e:
# #             print(f"[SERVO] Failed: {e}")
# #             return False

# #     def send_servo_angle(self, angle):
# #         """Send angle command to servo motor."""
# #         if self.servo:
# #             servo_angle = self.neutral_angle + angle
# #             servo_angle = int(np.clip(servo_angle, 42, 72))

# #             try:
# #                 self.servo.write(bytes([servo_angle]))
# #             except Exception:
# #                 print("[SERVO] Send failed")

# #     # ================================================================
# #     # PID
# #     # ================================================================

# #     def update_pid(self, position, dt=0.033):
# #         """Perform PID calculation using ACTIVE parameters."""

# #         error_m = self.setpoint - position

# #         # Deadband: ignore errors within ±deadband_m (in real meters,
# #         # applied BEFORE the x100 tuning scale)
# #         if abs(error_m) <= self.deadband_m:
# #             return 0.0

# #         # Scale error for easier tuning
# #         error = error_m * 100

# #         # Proportional term
# #         P = self.Kp * error

# #         # Integral term (does not accumulate while inside the deadband)
# #         self.integral += error * dt
# #         I = self.Ki * self.integral

# #         # Derivative term
# #         derivative = (error - self.prev_error) / dt
# #         D = self.Kd * derivative

# #         self.prev_error = error

# #         # PID output
# #         output = P + I + D

# #         # Limit to safe beam range
# #         output = np.clip(output, -15, 15)

# #         print(error)

# #         return output

# #     # ================================================================
# #     # CAMERA THREAD
# #     # ================================================================

# #     def camera_thread(self):
# #         """Dedicated thread for video capture and ball detection."""

# #         cap = cv2.VideoCapture(
# #             self.config['camera']['index'],
# #             cv2.CAP_DSHOW
# #         )

# #         cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# #         while self.running:
# #             ret, frame = cap.read()

# #             if not ret:
# #                 continue

# #             frame = cv2.resize(frame, (320, 240))

# #             # Detect ball position
# #             found, x_normalized, vis_frame = detect_ball_x(frame)

# #             if found:
# #                 # Convert normalized position to meters
# #                 position_m = x_normalized * self.scale_factor

# #                 # Keep latest measurement only
# #                 try:
# #                     if self.position_queue.full():
# #                         self.position_queue.get_nowait()

# #                     self.position_queue.put_nowait(position_m)

# #                 except Exception:
# #                     pass

# #             # Show processed video
# #             cv2.imshow("Ball Tracking", vis_frame)

# #             # ESC exits
# #             if cv2.waitKey(1) & 0xFF == 27:
# #                 self.running = False
# #                 break

# #         cap.release()
# #         cv2.destroyAllWindows()

# #     # ================================================================
# #     # CONTROL THREAD
# #     # ================================================================

# #     def control_thread(self):
# #         """Runs PID control loop in parallel with GUI and camera."""

# #         if not self.connect_servo():
# #             print("[ERROR] No servo - running in simulation mode")

# #         self.start_time = time.time()

# #         while self.running:
# #             try:
# #                 # Get latest ball position
# #                 position = self.position_queue.get(timeout=0.1)

# #                 # Calculate PID output using ACTIVE parameters
# #                 control_output = self.update_pid(position)

# #                 # Send command to servo
# #                 self.send_servo_angle(control_output)

# #                 # Log results
# #                 current_time = time.time() - self.start_time

# #                 self.time_log.append(current_time)
# #                 self.position_log.append(position)
# #                 self.setpoint_log.append(self.setpoint)
# #                 self.control_log.append(control_output)

# #                 print(
# #                     f"Pos: {position:.3f}m, "
# #                     f"Output: {control_output:.1f}°"
# #                 )

# #             except queue.Empty:
# #                 continue

# #             except Exception as e:
# #                 print(f"[CONTROL] Error: {e}")
# #                 break

# #         # Return servo to neutral on exit
# #         if self.servo:
# #             self.send_servo_angle(0)
# #             self.servo.close()

# #     # ================================================================
# #     # GUI PARAMETER ROW
# #     # ================================================================

# #     def _make_param_row(
# #         self,
# #         parent,
# #         title,
# #         attr,
# #         slider_lo,
# #         slider_hi,
# #         fmt,
# #         hard_min=None,
# #         hard_max=None
# #     ):
# #         """
# #         Build one parameter row.

# #         IMPORTANT:
# #         The GUI modifies pending_<parameter>, NOT the active
# #         controller parameter.

# #         Example:
# #             slider changes pending_Kp
# #             Apply button copies pending_Kp -> Kp
# #         """

# #         def clamp(v, lo, hi):
# #             if lo is not None:
# #                 v = max(lo, v)

# #             if hi is not None:
# #                 v = min(hi, v)

# #             return v

# #         row = ttk.Frame(parent)
# #         row.pack(fill=tk.X, padx=15, pady=6)

# #         row.columnconfigure(0, weight=1)

# #         ttk.Label(
# #             row,
# #             text=title,
# #             font=("Arial", 12)
# #         ).grid(
# #             row=0,
# #             column=0,
# #             sticky="w"
# #         )

# #         # ------------------------------------------------------------
# #         # Use PENDING value for GUI
# #         # ------------------------------------------------------------

# #         pending_attr = "pending_" + attr

# #         initial_value = getattr(self, pending_attr)

# #         entry_var = tk.StringVar(
# #             value=fmt.format(initial_value)
# #         )

# #         slider_var = tk.DoubleVar(
# #             value=clamp(
# #                 initial_value,
# #                 slider_lo,
# #                 slider_hi
# #             )
# #         )

# #         self._gui_vars.extend([
# #             entry_var,
# #             slider_var
# #         ])

# #         tolerance = max(
# #             (slider_hi - slider_lo) * 1e-6,
# #             1e-12
# #         )

# #         # ------------------------------------------------------------
# #         # Slider changed
# #         # ------------------------------------------------------------

# #         def on_slider(value):
# #             """
# #             Slider changes ONLY the pending value.
# #             It does not change the active PID parameter.
# #             """

# #             v = float(value)

# #             current_pending = getattr(
# #                 self,
# #                 pending_attr
# #             )

# #             if abs(
# #                 v - clamp(
# #                     current_pending,
# #                     slider_lo,
# #                     slider_hi
# #                 )
# #             ) <= tolerance:
# #                 return

# #             # IMPORTANT:
# #             # Change pending value, NOT active value
# #             setattr(
# #                 self,
# #                 pending_attr,
# #                 v
# #             )

# #             entry_var.set(
# #                 fmt.format(v)
# #             )

# #         # ------------------------------------------------------------
# #         # Entry changed
# #         # ------------------------------------------------------------

# #         def commit(_event=None):
# #             """
# #             Typed value is stored as a pending value.
# #             The active PID controller is unchanged.
# #             """

# #             try:
# #                 v = float(entry_var.get())

# #                 if not math.isfinite(v):
# #                     raise ValueError

# #             except ValueError:

# #                 # Revert to current pending value
# #                 entry_var.set(
# #                     fmt.format(
# #                         getattr(
# #                             self,
# #                             pending_attr
# #                         )
# #                     )
# #                 )

# #                 return

# #             # Apply hard limits
# #             v = clamp(
# #                 v,
# #                 hard_min,
# #                 hard_max
# #             )

# #             # IMPORTANT:
# #             # Store as pending value
# #             setattr(
# #                 self,
# #                 pending_attr,
# #                 v
# #             )

# #             entry_var.set(
# #                 fmt.format(v)
# #             )

# #             # Move slider to match typed value
# #             slider_var.set(
# #                 clamp(
# #                     v,
# #                     slider_lo,
# #                     slider_hi
# #                 )
# #             )

# #         # Entry box
# #         entry = ttk.Entry(
# #             row,
# #             textvariable=entry_var,
# #             width=9,
# #             justify="right",
# #             font=("Arial", 11)
# #         )

# #         entry.grid(
# #             row=0,
# #             column=1,
# #             sticky="e"
# #         )

# #         entry.bind("<Return>", commit)
# #         entry.bind("<KP_Enter>", commit)
# #         entry.bind("<FocusOut>", commit)

# #         # Slider
# #         slider = ttk.Scale(
# #             row,
# #             from_=slider_lo,
# #             to=slider_hi,
# #             variable=slider_var,
# #             orient=tk.HORIZONTAL,
# #             command=on_slider
# #         )

# #         slider.grid(
# #             row=1,
# #             column=0,
# #             columnspan=2,
# #             sticky="ew",
# #             pady=(2, 0)
# #         )

# #     # ================================================================
# #     # APPLY PARAMETERS
# #     # ================================================================

# #     def apply_parameters(self):
# #         """
# #         Copy pending GUI parameters into the ACTIVE PID controller.

# #         This is the ONLY place where the active PID parameters are
# #         changed.
# #         """

# #         self.Kp = self.pending_Kp
# #         self.Ki = self.pending_Ki
# #         self.Kd = self.pending_Kd
# #         self.setpoint = self.pending_setpoint

# #         # Reset integral when parameters are changed
# #         self.integral = 0.0

# #         # Reset previous error so derivative does not jump
# #         self.prev_error = 0.0

# #         print(
# #             "[APPLY] Parameters updated:"
# #         )

# #         print(
# #             f"       Kp = {self.Kp:.4f}"
# #         )

# #         print(
# #             f"       Ki = {self.Ki:.4f}"
# #         )

# #         print(
# #             f"       Kd = {self.Kd:.4f}"
# #         )

# #         print(
# #             f"       Setpoint = {self.setpoint:.4f} m"
# #         )

# #         # Update the on-screen status text with the active values
# #         if hasattr(self, "status_label"):
# #             self.status_label.config(
# #                 text=(
# #                     f"Active parameters: Kp={self.Kp:.4f}, "
# #                     f"Ki={self.Ki:.4f}, Kd={self.Kd:.4f}, "
# #                     f"Setpoint={self.setpoint:.4f}"
# #                 )
# #             )

# #     # ================================================================
# #     # GUI
# #     # ================================================================

# #     def create_gui(self):
# #         """Build Tkinter GUI."""

# #         self.root = tk.Tk()

# #         self.root.title(
# #             "Basic PID Controller"
# #         )

# #         self.root.geometry(
# #             "560x580"
# #         )

# #         # ------------------------------------------------------------
# #         # Title
# #         # ------------------------------------------------------------

# #         ttk.Label(
# #             self.root,
# #             text="PID Gains",
# #             font=("Arial", 18, "bold")
# #         ).pack(pady=10)

# #         # ------------------------------------------------------------
# #         # PID gains
# #         # ------------------------------------------------------------

# #         self._make_param_row(
# #             self.root,
# #             "Kp (Proportional)",
# #             "Kp",
# #             0,
# #             100,
# #             "{:.4f}",
# #             hard_min=0
# #         )

# #         self._make_param_row(
# #             self.root,
# #             "Ki (Integral)",
# #             "Ki",
# #             0,
# #             10,
# #             "{:.4f}",
# #             hard_min=0
# #         )

# #         self._make_param_row(
# #             self.root,
# #             "Kd (Derivative)",
# #             "Kd",
# #             0,
# #             20,
# #             "{:.4f}",
# #             hard_min=0
# #         )

# #         # ------------------------------------------------------------
# #         # Setpoint limits
# #         # ------------------------------------------------------------

# #         cal = self.config['calibration']

# #         half_beam = (
# #             self.config.get(
# #                 'beam_length_m',
# #                 0.168
# #             ) / 2
# #         )

# #         pos_min = cal.get(
# #             'position_min_m'
# #         )

# #         pos_max = cal.get(
# #             'position_max_m'
# #         )

# #         if pos_min is None:
# #             pos_min = -half_beam

# #         if pos_max is None:
# #             pos_max = half_beam

# #         # ------------------------------------------------------------
# #         # Setpoint
# #         # ------------------------------------------------------------

# #         self._make_param_row(
# #             self.root,
# #             "Setpoint (meters)",
# #             "setpoint",
# #             pos_min,
# #             pos_max,
# #             "{:.4f}",
# #             hard_min=pos_min,
# #             hard_max=pos_max
# #         )

# #         # ------------------------------------------------------------
# #         # Apply button
# #         # ------------------------------------------------------------

# #         apply_frame = ttk.Frame(
# #             self.root
# #         )

# #         apply_frame.pack(
# #             pady=15
# #         )

# #         ttk.Button(
# #             apply_frame,
# #             text="Apply Parameters",
# #             command=self.apply_parameters
# #         ).pack(
# #             side=tk.LEFT,
# #             padx=5
# #         )

# #         # ------------------------------------------------------------
# #         # Other buttons
# #         # ------------------------------------------------------------

# #         button_frame = ttk.Frame(
# #             self.root
# #         )

# #         button_frame.pack(
# #             pady=10
# #         )

# #         ttk.Button(
# #             button_frame,
# #             text="Reset Integral",
# #             command=self.reset_integral
# #         ).pack(
# #             side=tk.LEFT,
# #             padx=5
# #         )

# #         ttk.Button(
# #             button_frame,
# #             text="Plot Results",
# #             command=self.plot_results
# #         ).pack(
# #             side=tk.LEFT,
# #             padx=5
# #         )

# #         ttk.Button(
# #             button_frame,
# #             text="Stop",
# #             command=self.stop
# #         ).pack(
# #             side=tk.LEFT,
# #             padx=5
# #         )

# #         # ------------------------------------------------------------
# #         # Status text (shows the ACTIVE parameters)
# #         # ------------------------------------------------------------

# #         self.status_label = ttk.Label(
# #             self.root,
# #             text=(
# #                 f"Active parameters: Kp={self.Kp:.4f}, "
# #                 f"Ki={self.Ki:.4f}, Kd={self.Kd:.4f}, "
# #                 f"Setpoint={self.setpoint:.4f}"
# #             ),
# #             font=("Arial", 10)
# #         )

# #         self.status_label.pack(
# #             pady=10
# #         )

# #     # ================================================================
# #     # RESET INTEGRAL
# #     # ================================================================

# #     def reset_integral(self):
# #         """Clear integral error."""

# #         self.integral = 0.0

# #         print(
# #             "[RESET] Integral term reset"
# #         )

# #     # ================================================================
# #     # PLOT
# #     # ================================================================

# #     def plot_results(self):
# #         """Show matplotlib plots of position and control logs."""

# #         if not self.time_log:
# #             print(
# #                 "[PLOT] No data to plot"
# #             )
# #             return

# #         fig, (ax1, ax2) = plt.subplots(
# #             2,
# #             1,
# #             figsize=(10, 8)
# #         )

# #         # ------------------------------------------------------------
# #         # Ball position
# #         # ------------------------------------------------------------

# #         ax1.plot(
# #             self.time_log,
# #             self.position_log,
# #             label="Ball Position",
# #             linewidth=2
# #         )

# #         ax1.plot(
# #             self.time_log,
# #             self.setpoint_log,
# #             label="Setpoint",
# #             linestyle="--",
# #             linewidth=2
# #         )

# #         ax1.set_ylabel(
# #             "Position (m)"
# #         )

# #         ax1.set_title(
# #             f"Basic PID Control "
# #             f"(Kp={self.Kp:.4f}, "
# #             f"Ki={self.Ki:.4f}, "
# #             f"Kd={self.Kd:.4f})"
# #         )

# #         ax1.legend()
# #         ax1.grid(
# #             True,
# #             alpha=0.3
# #         )

# #         # ------------------------------------------------------------
# #         # Control output
# #         # ------------------------------------------------------------

# #         ax2.plot(
# #             self.time_log,
# #             self.control_log,
# #             label="Control Output",
# #             color="orange",
# #             linewidth=2
# #         )

# #         ax2.set_xlabel(
# #             "Time (s)"
# #         )

# #         ax2.set_ylabel(
# #             "Beam Angle (degrees)"
# #         )

# #         ax2.legend()

# #         ax2.grid(
# #             True,
# #             alpha=0.3
# #         )

# #         plt.tight_layout()

# #         plt.show()

# #     # ================================================================
# #     # STOP
# #     # ================================================================

# #     def stop(self):
# #         """Stop everything and clean up."""

# #         self.running = False

# #         try:
# #             self.root.quit()
# #             self.root.destroy()
# #         except Exception:
# #             pass

# #     # ================================================================
# #     # RUN
# #     # ================================================================

# #     def run(self):
# #         """Entry point."""

# #         print(
# #             "[INFO] Starting Basic PID Controller"
# #         )

# #         print(
# #             "Change PID values using the GUI."
# #         )

# #         print(
# #             "Press 'Apply Parameters' to activate changes."
# #         )

# #         print(
# #             "Close camera window or click Stop to exit."
# #         )

# #         self.running = True

# #         # ------------------------------------------------------------
# #         # Start camera thread
# #         # ------------------------------------------------------------

# #         cam_thread = Thread(
# #             target=self.camera_thread,
# #             daemon=True
# #         )

# #         # ------------------------------------------------------------
# #         # Start control thread
# #         # ------------------------------------------------------------

# #         ctrl_thread = Thread(
# #             target=self.control_thread,
# #             daemon=True
# #         )

# #         cam_thread.start()
# #         ctrl_thread.start()

# #         # ------------------------------------------------------------
# #         # Build GUI
# #         # ------------------------------------------------------------

# #         self.create_gui()

# #         self.root.mainloop()

# #         # ------------------------------------------------------------
# #         # After GUI closes
# #         # ------------------------------------------------------------

# #         self.running = False

# #         print(
# #             "[INFO] Controller stopped"
# #         )


# # # ================================================================
# # # MAIN
# # # ================================================================

# # if __name__ == "__main__":

# #     try:
# #         controller = BasicPIDController()

# #         controller.run()

# #     except FileNotFoundError:
# #         print(
# #             "[ERROR] config.json not found. "
# #             "Run simple_autocal.py first."
# #         )

# #     except Exception as e:
# #         print(
# #             f"[ERROR] {e}"
# #         )

# import cv2
# import numpy as np
# import json
# import math
# import serial
# import time
# import tkinter as tk
# from tkinter import ttk
# import matplotlib.pyplot as plt
# from threading import Thread
# import queue
# from ball_detection import detect_ball_x


# class BasicPIDController:
#     def __init__(self, config_file="config.json"):
#         """Initialize controller, load config, set defaults and queues."""

#         # Load experiment and hardware config from JSON file
#         with open(config_file, 'r') as f:
#             self.config = json.load(f)

#         # ============================================================
#         # ACTIVE PID PARAMETERS
#         # These are the values actually being used by the controller.
#         # They only change when "Apply Parameters" is pressed.
#         # ============================================================
#         self.Kp = 10.0
#         self.Ki = 0.0
#         self.Kd = 0.0
#         self.setpoint = 0.0
#         # Deadband: errors smaller than this (in meters) are treated as zero
#         self.deadband_m = 0.00   # ±2 mm

#         # ============================================================
#         # PENDING PID PARAMETERS
#         # These are changed by the GUI sliders / entry boxes.
#         # They do NOT affect the controller until Apply is pressed.
#         # ============================================================
#         self.pending_Kp = self.Kp
#         self.pending_Ki = self.Ki
#         self.pending_Kd = self.Kd
#         self.pending_setpoint = self.setpoint

#         # Scale factor for converting from pixels to meters
#         self.scale_factor = (
#             self.config['calibration']['pixel_to_meter_ratio']
#             * self.config['camera']['frame_width'] / 2
#         )

#         # Servo port name and center angle
#         self.servo_port = self.config['servo']['port']
#         self.neutral_angle = self.config['servo']['neutral_angle']
#         self.servo = None

#         # Controller-internal state
#         self.integral = 0.0
#         self.prev_error = 0.0

#         # Jump rejection (camera glitch filter)
#         self.last_pos_m = None
#         self.max_jump_m = 0.03    # reject single-frame jumps bigger than this
#         self.max_rejects = 5      # after this many in a row, accept the new position
#         self.reject_count = 0

#         # Data logs for plotting results
#         self.time_log = []
#         self.position_log = []
#         self.setpoint_log = []
#         self.control_log = []
#         self.apply_log = []   # (time, Kp, Ki, Kd) each time gains are applied
#         self.start_time = None

#         # Thread-safe queue for most recent ball position measurement
#         self.position_queue = queue.Queue(maxsize=1)

#         # Main run flag for clean shutdown
#         self.running = False

#         # Keep Tk variables alive
#         self._gui_vars = []

#     # ================================================================
#     # SERVO
#     # ================================================================

#     def connect_servo(self):
#         """Try to open serial connection to servo."""
#         try:
#             self.servo = serial.Serial(self.servo_port, 9600)
#             time.sleep(2)
#             print("[SERVO] Connected")
#             return True
#         except Exception as e:
#             print(f"[SERVO] Failed: {e}")
#             return False

#     def send_servo_angle(self, angle):
#         """Send angle command to servo motor."""
#         if self.servo:
#             servo_angle = self.neutral_angle + angle
#             servo_angle = int(round(np.clip(servo_angle, 42, 72)))

#             try:
#                 self.servo.write(bytes([servo_angle]))
#             except Exception:
#                 print("[SERVO] Send failed")

#     # ================================================================
#     # PID
#     # ================================================================

#     def update_pid(self, position, dt=0.033):
#         """Perform PID calculation using ACTIVE parameters."""

#         error_m = self.setpoint - position

#         # Deadband: ignore errors within ±deadband_m (in real meters,
#         # applied BEFORE the x100 tuning scale)
#         if abs(error_m) <= self.deadband_m:
#             return 0.0

#         # Scale error for easier tuning
#         error = error_m * 100

#         # Proportional term
#         P = self.Kp * error

#         # Integral term (does not accumulate while inside the deadband)
#         self.integral += error * dt
#         I = self.Ki * self.integral

#         # Derivative term
#         derivative = (error - self.prev_error) / dt
#         D = self.Kd * derivative

#         self.prev_error = error

#         # PID output
#         output = P + I + D

#         # Limit to safe beam range
#         output = np.clip(output, -15, 15)

#         print(error)

#         return output

#     # ================================================================
#     # CAMERA THREAD
#     # ================================================================

#     def camera_thread(self):
#         """Dedicated thread for video capture and ball detection."""

#         cap = cv2.VideoCapture(
#             self.config['camera']['index'],
#             cv2.CAP_DSHOW
#         )

#         cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

#         while self.running:
#             ret, frame = cap.read()

#             if not ret:
#                 continue

#             frame = cv2.resize(frame, (320, 240))

#             # Detect ball position
#             found, x_normalized, vis_frame = detect_ball_x(frame)

#             if found:
#                 # Convert normalized position to meters
#                 position_m = x_normalized * self.scale_factor

#                 # Jump rejection: skip sudden glitch readings, but accept
#                 # the new position if it persists for max_rejects frames
#                 accept = True

#                 if (
#                     self.last_pos_m is not None
#                     and abs(position_m - self.last_pos_m) > self.max_jump_m
#                 ):
#                     self.reject_count += 1

#                     if self.reject_count < self.max_rejects:
#                         accept = False

#                 if accept:
#                     self.reject_count = 0
#                     self.last_pos_m = position_m

#                     # Keep latest measurement only
#                     try:
#                         if self.position_queue.full():
#                             self.position_queue.get_nowait()

#                         self.position_queue.put_nowait(position_m)

#                     except Exception:
#                         pass

#             # Show processed video
#             cv2.imshow("Ball Tracking", vis_frame)

#             # ESC exits
#             if cv2.waitKey(1) & 0xFF == 27:
#                 self.running = False
#                 break

#         cap.release()
#         cv2.destroyAllWindows()

#     # ================================================================
#     # CONTROL THREAD
#     # ================================================================

#     def control_thread(self):
#         """Runs PID control loop in parallel with GUI and camera."""

#         if not self.connect_servo():
#             print("[ERROR] No servo - running in simulation mode")

#         self.start_time = time.time()

#         # Record the starting gains so the first segment is labeled too
#         self.apply_log.append((0.0, self.Kp, self.Ki, self.Kd))

#         while self.running:
#             try:
#                 # Get latest ball position
#                 position = self.position_queue.get(timeout=0.1)

#                 # Calculate PID output using ACTIVE parameters
#                 control_output = self.update_pid(position)

#                 # Send command to servo
#                 self.send_servo_angle(control_output)

#                 # Log results
#                 current_time = time.time() - self.start_time

#                 self.time_log.append(current_time)
#                 self.position_log.append(position)
#                 self.setpoint_log.append(self.setpoint)
#                 self.control_log.append(control_output)

#                 print(
#                     f"Pos: {position:.3f}m, "
#                     f"Output: {control_output:.1f}°"
#                 )

#             except queue.Empty:
#                 continue

#             except Exception as e:
#                 print(f"[CONTROL] Error: {e}")
#                 break

#         # Return servo to neutral on exit
#         if self.servo:
#             self.send_servo_angle(0)
#             self.servo.close()

#     # ================================================================
#     # GUI PARAMETER ROW
#     # ================================================================

#     def _make_param_row(
#         self,
#         parent,
#         title,
#         attr,
#         slider_lo,
#         slider_hi,
#         fmt,
#         hard_min=None,
#         hard_max=None
#     ):
#         """
#         Build one parameter row.

#         IMPORTANT:
#         The GUI modifies pending_<parameter>, NOT the active
#         controller parameter.

#         Example:
#             slider changes pending_Kp
#             Apply button copies pending_Kp -> Kp
#         """

#         def clamp(v, lo, hi):
#             if lo is not None:
#                 v = max(lo, v)

#             if hi is not None:
#                 v = min(hi, v)

#             return v

#         row = ttk.Frame(parent)
#         row.pack(fill=tk.X, padx=15, pady=6)

#         row.columnconfigure(0, weight=1)

#         ttk.Label(
#             row,
#             text=title,
#             font=("Arial", 12)
#         ).grid(
#             row=0,
#             column=0,
#             sticky="w"
#         )

#         # ------------------------------------------------------------
#         # Use PENDING value for GUI
#         # ------------------------------------------------------------

#         pending_attr = "pending_" + attr

#         initial_value = getattr(self, pending_attr)

#         entry_var = tk.StringVar(
#             value=fmt.format(initial_value)
#         )

#         slider_var = tk.DoubleVar(
#             value=clamp(
#                 initial_value,
#                 slider_lo,
#                 slider_hi
#             )
#         )

#         self._gui_vars.extend([
#             entry_var,
#             slider_var
#         ])

#         tolerance = max(
#             (slider_hi - slider_lo) * 1e-6,
#             1e-12
#         )

#         # ------------------------------------------------------------
#         # Slider changed
#         # ------------------------------------------------------------

#         def on_slider(value):
#             """
#             Slider changes ONLY the pending value.
#             It does not change the active PID parameter.
#             """

#             v = float(value)

#             current_pending = getattr(
#                 self,
#                 pending_attr
#             )

#             if abs(
#                 v - clamp(
#                     current_pending,
#                     slider_lo,
#                     slider_hi
#                 )
#             ) <= tolerance:
#                 return

#             # IMPORTANT:
#             # Change pending value, NOT active value
#             setattr(
#                 self,
#                 pending_attr,
#                 v
#             )

#             entry_var.set(
#                 fmt.format(v)
#             )

#         # ------------------------------------------------------------
#         # Entry changed
#         # ------------------------------------------------------------

#         def commit(_event=None):
#             """
#             Typed value is stored as a pending value.
#             The active PID controller is unchanged.
#             """

#             try:
#                 v = float(entry_var.get())

#                 if not math.isfinite(v):
#                     raise ValueError

#             except ValueError:

#                 # Revert to current pending value
#                 entry_var.set(
#                     fmt.format(
#                         getattr(
#                             self,
#                             pending_attr
#                         )
#                     )
#                 )

#                 return

#             # Apply hard limits
#             v = clamp(
#                 v,
#                 hard_min,
#                 hard_max
#             )

#             # IMPORTANT:
#             # Store as pending value
#             setattr(
#                 self,
#                 pending_attr,
#                 v
#             )

#             entry_var.set(
#                 fmt.format(v)
#             )

#             # Move slider to match typed value
#             slider_var.set(
#                 clamp(
#                     v,
#                     slider_lo,
#                     slider_hi
#                 )
#             )

#         # Entry box
#         entry = ttk.Entry(
#             row,
#             textvariable=entry_var,
#             width=9,
#             justify="right",
#             font=("Arial", 11)
#         )

#         entry.grid(
#             row=0,
#             column=1,
#             sticky="e"
#         )

#         entry.bind("<Return>", commit)
#         entry.bind("<KP_Enter>", commit)
#         entry.bind("<FocusOut>", commit)

#         # Slider
#         slider = ttk.Scale(
#             row,
#             from_=slider_lo,
#             to=slider_hi,
#             variable=slider_var,
#             orient=tk.HORIZONTAL,
#             command=on_slider
#         )

#         slider.grid(
#             row=1,
#             column=0,
#             columnspan=2,
#             sticky="ew",
#             pady=(2, 0)
#         )

#     # ================================================================
#     # APPLY PARAMETERS
#     # ================================================================

#     def apply_parameters(self):
#         """
#         Copy pending GUI parameters into the ACTIVE PID controller.

#         This is the ONLY place where the active PID parameters are
#         changed.
#         """

#         self.Kp = self.pending_Kp
#         self.Ki = self.pending_Ki
#         self.Kd = self.pending_Kd
#         self.setpoint = self.pending_setpoint

#         # Reset integral when parameters are changed
#         self.integral = 0.0

#         # Reset previous error so derivative does not jump
#         self.prev_error = 0.0

#         # Remember when these gains were applied (for plot markers)
#         apply_time = (
#             time.time() - self.start_time
#             if self.start_time is not None
#             else 0.0
#         )

#         self.apply_log.append(
#             (apply_time, self.Kp, self.Ki, self.Kd)
#         )

#         print(
#             "[APPLY] Parameters updated:"
#         )

#         print(
#             f"       Kp = {self.Kp:.4f}"
#         )

#         print(
#             f"       Ki = {self.Ki:.4f}"
#         )

#         print(
#             f"       Kd = {self.Kd:.4f}"
#         )

#         print(
#             f"       Setpoint = {self.setpoint:.4f} m"
#         )

#         # Update the on-screen status text with the active values
#         if hasattr(self, "status_label"):
#             self.status_label.config(
#                 text=(
#                     f"Active parameters: Kp={self.Kp:.4f}, "
#                     f"Ki={self.Ki:.4f}, Kd={self.Kd:.4f}, "
#                     f"Setpoint={self.setpoint:.4f}"
#                 )
#             )

#     # ================================================================
#     # GUI
#     # ================================================================

#     def create_gui(self):
#         """Build Tkinter GUI."""

#         self.root = tk.Tk()

#         self.root.title(
#             "Basic PID Controller"
#         )

#         self.root.geometry(
#             "560x580"
#         )

#         # ------------------------------------------------------------
#         # Title
#         # ------------------------------------------------------------

#         ttk.Label(
#             self.root,
#             text="PID Gains",
#             font=("Arial", 18, "bold")
#         ).pack(pady=10)

#         # ------------------------------------------------------------
#         # PID gains
#         # ------------------------------------------------------------

#         self._make_param_row(
#             self.root,
#             "Kp (Proportional)",
#             "Kp",
#             0,
#             100,
#             "{:.4f}",
#             hard_min=0
#         )

#         self._make_param_row(
#             self.root,
#             "Ki (Integral)",
#             "Ki",
#             0,
#             10,
#             "{:.4f}",
#             hard_min=0
#         )

#         self._make_param_row(
#             self.root,
#             "Kd (Derivative)",
#             "Kd",
#             0,
#             20,
#             "{:.4f}",
#             hard_min=0
#         )

#         # ------------------------------------------------------------
#         # Setpoint limits
#         # ------------------------------------------------------------

#         cal = self.config['calibration']

#         half_beam = (
#             self.config.get(
#                 'beam_length_m',
#                 0.168
#             ) / 2
#         )

#         pos_min = cal.get(
#             'position_min_m'
#         )

#         pos_max = cal.get(
#             'position_max_m'
#         )

#         if pos_min is None:
#             pos_min = -half_beam

#         if pos_max is None:
#             pos_max = half_beam

#         # ------------------------------------------------------------
#         # Setpoint
#         # ------------------------------------------------------------

#         self._make_param_row(
#             self.root,
#             "Setpoint (meters)",
#             "setpoint",
#             pos_min,
#             pos_max,
#             "{:.4f}",
#             hard_min=pos_min,
#             hard_max=pos_max
#         )

#         # ------------------------------------------------------------
#         # Apply button
#         # ------------------------------------------------------------

#         apply_frame = ttk.Frame(
#             self.root
#         )

#         apply_frame.pack(
#             pady=15
#         )

#         ttk.Button(
#             apply_frame,
#             text="Apply Parameters",
#             command=self.apply_parameters
#         ).pack(
#             side=tk.LEFT,
#             padx=5
#         )

#         # ------------------------------------------------------------
#         # Other buttons
#         # ------------------------------------------------------------

#         button_frame = ttk.Frame(
#             self.root
#         )

#         button_frame.pack(
#             pady=10
#         )

#         ttk.Button(
#             button_frame,
#             text="Reset Integral",
#             command=self.reset_integral
#         ).pack(
#             side=tk.LEFT,
#             padx=5
#         )

#         ttk.Button(
#             button_frame,
#             text="Plot Results",
#             command=self.plot_results
#         ).pack(
#             side=tk.LEFT,
#             padx=5
#         )

#         ttk.Button(
#             button_frame,
#             text="Stop",
#             command=self.stop
#         ).pack(
#             side=tk.LEFT,
#             padx=5
#         )

#         # ------------------------------------------------------------
#         # Status text (shows the ACTIVE parameters)
#         # ------------------------------------------------------------

#         self.status_label = ttk.Label(
#             self.root,
#             text=(
#                 f"Active parameters: Kp={self.Kp:.4f}, "
#                 f"Ki={self.Ki:.4f}, Kd={self.Kd:.4f}, "
#                 f"Setpoint={self.setpoint:.4f}"
#             ),
#             font=("Arial", 10)
#         )

#         self.status_label.pack(
#             pady=10
#         )

#     # ================================================================
#     # RESET INTEGRAL
#     # ================================================================

#     def reset_integral(self):
#         """Clear integral error."""

#         self.integral = 0.0

#         print(
#             "[RESET] Integral term reset"
#         )

#     # ================================================================
#     # PLOT
#     # ================================================================

#     def plot_results(self):
#         """Show matplotlib plots of position and control logs."""

#         if not self.time_log:
#             print(
#                 "[PLOT] No data to plot"
#             )
#             return

#         fig, (ax1, ax2) = plt.subplots(
#             2,
#             1,
#             figsize=(10, 8)
#         )

#         # ------------------------------------------------------------
#         # Ball position
#         # ------------------------------------------------------------

#         ax1.plot(
#             self.time_log,
#             self.position_log,
#             label="Ball Position",
#             linewidth=2
#         )

#         ax1.plot(
#             self.time_log,
#             self.setpoint_log,
#             label="Setpoint",
#             linestyle="--",
#             linewidth=2
#         )

#         ax1.set_ylabel(
#             "Position (m)"
#         )

#         ax1.set_title(
#             f"Basic PID Control "
#             f"(Kp={self.Kp:.4f}, "
#             f"Ki={self.Ki:.4f}, "
#             f"Kd={self.Kd:.4f})"
#         )

#         ax1.legend()
#         ax1.grid(
#             True,
#             alpha=0.3
#         )

#         # ------------------------------------------------------------
#         # Control output
#         # ------------------------------------------------------------

#         ax2.plot(
#             self.time_log,
#             self.control_log,
#             label="Control Output",
#             color="orange",
#             linewidth=2
#         )

#         ax2.set_xlabel(
#             "Time (s)"
#         )

#         ax2.set_ylabel(
#             "Beam Angle (degrees)"
#         )

#         ax2.legend()

#         ax2.grid(
#             True,
#             alpha=0.3
#         )

#         # ------------------------------------------------------------
#         # Markers where gains were applied (labeled with the new gains)
#         # ------------------------------------------------------------

#         for t, kp, ki, kd in self.apply_log:
#             for ax in (ax1, ax2):
#                 ax.axvline(
#                     t,
#                     color="gray",
#                     linestyle=":",
#                     linewidth=1.5,
#                     alpha=0.8
#                 )

#             ax1.text(
#                 t,
#                 0.02,
#                 f"Kp={kp:.2f} Ki={ki:.3f} Kd={kd:.2f}",
#                 transform=ax1.get_xaxis_transform(),
#                 rotation=90,
#                 va="bottom",
#                 ha="right",
#                 fontsize=8,
#                 color="dimgray"
#             )

#         plt.tight_layout()

#         plt.show()

#     # ================================================================
#     # STOP
#     # ================================================================

#     def stop(self):
#         """Stop everything and clean up."""

#         self.running = False

#         try:
#             self.root.quit()
#             self.root.destroy()
#         except Exception:
#             pass

#     # ================================================================
#     # RUN
#     # ================================================================

#     def run(self):
#         """Entry point."""

#         print(
#             "[INFO] Starting Basic PID Controller"
#         )

#         print(
#             "Change PID values using the GUI."
#         )

#         print(
#             "Press 'Apply Parameters' to activate changes."
#         )

#         print(
#             "Close camera window or click Stop to exit."
#         )

#         self.running = True

#         # ------------------------------------------------------------
#         # Start camera thread
#         # ------------------------------------------------------------

#         cam_thread = Thread(
#             target=self.camera_thread,
#             daemon=True
#         )

#         # ------------------------------------------------------------
#         # Start control thread
#         # ------------------------------------------------------------

#         ctrl_thread = Thread(
#             target=self.control_thread,
#             daemon=True
#         )

#         cam_thread.start()
#         ctrl_thread.start()

#         # ------------------------------------------------------------
#         # Build GUI
#         # ------------------------------------------------------------

#         self.create_gui()

#         self.root.mainloop()

#         # ------------------------------------------------------------
#         # After GUI closes
#         # ------------------------------------------------------------

#         self.running = False

#         print(
#             "[INFO] Controller stopped"
#         )


# # ================================================================
# # MAIN
# # ================================================================

# if __name__ == "__main__":

#     try:
#         controller = BasicPIDController()

#         controller.run()

#     except FileNotFoundError:
#         print(
#             "[ERROR] config.json not found. "
#             "Run simple_autocal.py first."
#         )

#     except Exception as e:
#         print(
#             f"[ERROR] {e}"
#         )

import cv2
import numpy as np
import json
import math
import serial
import time
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from threading import Thread
import queue
from ball_detection import detect_ball_x


class BasicPIDController:
    def __init__(self, config_file="config.json"):
        """Initialize controller, load config, set defaults and queues."""

        # Load experiment and hardware config from JSON file
        with open(config_file, 'r') as f:
            self.config = json.load(f)

        # ============================================================
        # ACTIVE PID PARAMETERS
        # These are the values actually being used by the controller.
        # They only change when "Apply Parameters" is pressed.
        # ============================================================
        self.Kp = 10.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.setpoint = 0.0
        # Deadband: errors smaller than this (in meters) are treated as zero
        self.deadband_m = 0.00   # ±2 mm

        # ============================================================
        # PENDING PID PARAMETERS
        # These are changed by the GUI sliders / entry boxes.
        # They do NOT affect the controller until Apply is pressed.
        # ============================================================
        self.pending_Kp = self.Kp
        self.pending_Ki = self.Ki
        self.pending_Kd = self.Kd
        self.pending_setpoint = self.setpoint

        # Scale factor for converting from pixels to meters
        self.scale_factor = (
            self.config['calibration']['pixel_to_meter_ratio']
            * self.config['camera']['frame_width'] / 2
        )

        # Servo port name and center angle
        self.servo_port = self.config['servo']['port']
        self.neutral_angle = self.config['servo']['neutral_angle']
        self.servo = None

        # Controller-internal state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_pos = None
        self.d_filt = 0.0
        self.alpha = 0.2      # derivative filter (lower = smoother)
        self.I_max = 200.0    # integral clamp (output limit = Ki * I_max)
        self.last_time = None  # timestamp of previous control sample
        self.dt_log = []       # measured time between samples

        # Jump rejection (camera glitch filter)
        self.last_pos_m = None
        self.max_jump_m = 0.03    # reject single-frame jumps bigger than this
        self.max_rejects = 5      # after this many in a row, accept the new position
        self.reject_count = 0

        # Data logs for plotting results
        self.time_log = []
        self.position_log = []
        self.setpoint_log = []
        self.control_log = []
        self.apply_log = []   # (time, Kp, Ki, Kd) each time gains are applied
        self.start_time = None

        # Thread-safe queue for most recent ball position measurement
        self.position_queue = queue.Queue(maxsize=1)

        # Main run flag for clean shutdown
        self.running = False

        # Keep Tk variables alive
        self._gui_vars = []

    # ================================================================
    # SERVO
    # ================================================================

    def connect_servo(self):
        """Try to open serial connection to servo."""
        try:
            self.servo = serial.Serial(self.servo_port, 9600)
            time.sleep(2)
            print("[SERVO] Connected")
            return True
        except Exception as e:
            print(f"[SERVO] Failed: {e}")
            return False

    def send_servo_angle(self, angle):
        """Send angle command to servo motor."""
        if self.servo:
            servo_angle = self.neutral_angle + angle
            servo_angle = int(round(np.clip(servo_angle, 42, 72)))

            try:
                self.servo.write(bytes([servo_angle]))
            except Exception:
                print("[SERVO] Send failed")

    # ================================================================
    # PID
    # ================================================================

    def update_pid(self, position, dt=0.033):
        """Perform PID calculation using ACTIVE parameters."""

        error_m = self.setpoint - position

        # Deadband: ignore errors within ±deadband_m (in real meters,
        # applied BEFORE the x100 tuning scale)
        if abs(error_m) <= self.deadband_m:
            return 0.0

        # Scale error for easier tuning
        error = error_m * 100

        # Proportional term
        P = self.Kp * error

        # Integral term (clamped)
        self.integral += error * dt
        self.integral = max(min(self.integral, self.I_max), -self.I_max)
        I = self.Ki * self.integral

        # Derivative term: on measurement, low-pass filtered
        pos_scaled = position * 100   # same x100 scale as the error
        if self.prev_pos is None:
            self.prev_pos = pos_scaled
        d_raw = -(pos_scaled - self.prev_pos) / dt
        self.d_filt = self.alpha * d_raw + (1 - self.alpha) * self.d_filt
        self.prev_pos = pos_scaled
        D = self.Kd * self.d_filt

        # PID output
        output = P + I + D

        # Limit to safe beam range
        output = np.clip(output, -15, 15)

        print(error)

        return output

    # ================================================================
    # CAMERA THREAD
    # ================================================================

    def camera_thread(self):
        """Dedicated thread for video capture and ball detection."""

        cap = cv2.VideoCapture(
            self.config['camera']['index'],
            cv2.CAP_DSHOW
        )

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        while self.running:
            ret, frame = cap.read()

            if not ret:
                continue

            frame = cv2.resize(frame, (320, 240))

            # Detect ball position
            found, x_normalized, vis_frame = detect_ball_x(frame)

            if found:
                # Convert normalized position to meters
                position_m = x_normalized * self.scale_factor

                # Jump rejection: skip sudden glitch readings, but accept
                # the new position if it persists for max_rejects frames
                accept = True

                if (
                    self.last_pos_m is not None
                    and abs(position_m - self.last_pos_m) > self.max_jump_m
                ):
                    self.reject_count += 1

                    if self.reject_count < self.max_rejects:
                        accept = False

                if accept:
                    self.reject_count = 0
                    self.last_pos_m = position_m

                    # Keep latest measurement only
                    try:
                        if self.position_queue.full():
                            self.position_queue.get_nowait()

                        self.position_queue.put_nowait(position_m)

                    except Exception:
                        pass

            # Show processed video
            cv2.imshow("Ball Tracking", vis_frame)

            # ESC exits
            if cv2.waitKey(1) & 0xFF == 27:
                self.running = False
                break

        cap.release()
        cv2.destroyAllWindows()

    # ================================================================
    # CONTROL THREAD
    # ================================================================

    def control_thread(self):
        """Runs PID control loop in parallel with GUI and camera."""

        if not self.connect_servo():
            print("[ERROR] No servo - running in simulation mode")

        self.start_time = time.time()

        # Record the starting gains so the first segment is labeled too
        self.apply_log.append((0.0, self.Kp, self.Ki, self.Kd))

        while self.running:
            try:
                # Get latest ball position
                position = self.position_queue.get(timeout=0.1)

                # Measure real time since the previous sample
                now = time.time()

                if self.last_time is None:
                    dt = 0.033
                else:
                    dt = min(max(now - self.last_time, 0.005), 0.2)

                self.last_time = now
                self.dt_log.append(dt)

                # Calculate PID output using ACTIVE parameters
                control_output = self.update_pid(position, dt)

                # Send command to servo
                self.send_servo_angle(control_output)

                # Log results
                current_time = time.time() - self.start_time

                self.time_log.append(current_time)
                self.position_log.append(position)
                self.setpoint_log.append(self.setpoint)
                self.control_log.append(control_output)

                print(
                    f"Pos: {position:.3f}m, "
                    f"Output: {control_output:.1f}°"
                )

            except queue.Empty:
                continue

            except Exception as e:
                print(f"[CONTROL] Error: {e}")
                break

        # Return servo to neutral on exit
        if self.servo:
            self.send_servo_angle(0)
            self.servo.close()

    # ================================================================
    # GUI PARAMETER ROW
    # ================================================================

    def _make_param_row(
        self,
        parent,
        title,
        attr,
        slider_lo,
        slider_hi,
        fmt,
        hard_min=None,
        hard_max=None
    ):
        """
        Build one parameter row.

        IMPORTANT:
        The GUI modifies pending_<parameter>, NOT the active
        controller parameter.

        Example:
            slider changes pending_Kp
            Apply button copies pending_Kp -> Kp
        """

        def clamp(v, lo, hi):
            if lo is not None:
                v = max(lo, v)

            if hi is not None:
                v = min(hi, v)

            return v

        row = ttk.Frame(parent)
        row.pack(fill=tk.X, padx=15, pady=6)

        row.columnconfigure(0, weight=1)

        ttk.Label(
            row,
            text=title,
            font=("Arial", 12)
        ).grid(
            row=0,
            column=0,
            sticky="w"
        )

        # ------------------------------------------------------------
        # Use PENDING value for GUI
        # ------------------------------------------------------------

        pending_attr = "pending_" + attr

        initial_value = getattr(self, pending_attr)

        entry_var = tk.StringVar(
            value=fmt.format(initial_value)
        )

        slider_var = tk.DoubleVar(
            value=clamp(
                initial_value,
                slider_lo,
                slider_hi
            )
        )

        self._gui_vars.extend([
            entry_var,
            slider_var
        ])

        tolerance = max(
            (slider_hi - slider_lo) * 1e-6,
            1e-12
        )

        # ------------------------------------------------------------
        # Slider changed
        # ------------------------------------------------------------

        def on_slider(value):
            """
            Slider changes ONLY the pending value.
            It does not change the active PID parameter.
            """

            v = float(value)

            current_pending = getattr(
                self,
                pending_attr
            )

            if abs(
                v - clamp(
                    current_pending,
                    slider_lo,
                    slider_hi
                )
            ) <= tolerance:
                return

            # IMPORTANT:
            # Change pending value, NOT active value
            setattr(
                self,
                pending_attr,
                v
            )

            entry_var.set(
                fmt.format(v)
            )

        # ------------------------------------------------------------
        # Entry changed
        # ------------------------------------------------------------

        def commit(_event=None):
            """
            Typed value is stored as a pending value.
            The active PID controller is unchanged.
            """

            try:
                v = float(entry_var.get())

                if not math.isfinite(v):
                    raise ValueError

            except ValueError:

                # Revert to current pending value
                entry_var.set(
                    fmt.format(
                        getattr(
                            self,
                            pending_attr
                        )
                    )
                )

                return

            # Apply hard limits
            v = clamp(
                v,
                hard_min,
                hard_max
            )

            # IMPORTANT:
            # Store as pending value
            setattr(
                self,
                pending_attr,
                v
            )

            entry_var.set(
                fmt.format(v)
            )

            # Move slider to match typed value
            slider_var.set(
                clamp(
                    v,
                    slider_lo,
                    slider_hi
                )
            )

        # Entry box
        entry = ttk.Entry(
            row,
            textvariable=entry_var,
            width=9,
            justify="right",
            font=("Arial", 11)
        )

        entry.grid(
            row=0,
            column=1,
            sticky="e"
        )

        entry.bind("<Return>", commit)
        entry.bind("<KP_Enter>", commit)
        entry.bind("<FocusOut>", commit)

        # Slider
        slider = ttk.Scale(
            row,
            from_=slider_lo,
            to=slider_hi,
            variable=slider_var,
            orient=tk.HORIZONTAL,
            command=on_slider
        )

        slider.grid(
            row=1,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(2, 0)
        )

    # ================================================================
    # APPLY PARAMETERS
    # ================================================================

    def apply_parameters(self):
        """
        Copy pending GUI parameters into the ACTIVE PID controller.

        This is the ONLY place where the active PID parameters are
        changed.
        """

        self.Kp = self.pending_Kp
        self.Ki = self.pending_Ki
        self.Kd = self.pending_Kd
        self.setpoint = self.pending_setpoint

        # Reset integral when parameters are changed
        self.integral = 0.0

        # Reset previous error so derivative does not jump
        self.prev_error = 0.0

        # Reset derivative filter state so it does not jump either
        self.prev_pos = None
        self.d_filt = 0.0

        # Remember when these gains were applied (for plot markers)
        apply_time = (
            time.time() - self.start_time
            if self.start_time is not None
            else 0.0
        )

        self.apply_log.append(
            (apply_time, self.Kp, self.Ki, self.Kd)
        )

        print(
            "[APPLY] Parameters updated:"
        )

        print(
            f"       Kp = {self.Kp:.4f}"
        )

        print(
            f"       Ki = {self.Ki:.4f}"
        )

        print(
            f"       Kd = {self.Kd:.4f}"
        )

        print(
            f"       Setpoint = {self.setpoint:.4f} m"
        )

        # Update the on-screen status text with the active values
        if hasattr(self, "status_label"):
            self.status_label.config(
                text=(
                    f"Active parameters: Kp={self.Kp:.4f}, "
                    f"Ki={self.Ki:.4f}, Kd={self.Kd:.4f}, "
                    f"Setpoint={self.setpoint:.4f}"
                )
            )

    # ================================================================
    # GUI
    # ================================================================

    def create_gui(self):
        """Build Tkinter GUI."""

        self.root = tk.Tk()

        self.root.title(
            "Basic PID Controller"
        )

        self.root.geometry(
            "560x580"
        )

        # ------------------------------------------------------------
        # Title
        # ------------------------------------------------------------

        ttk.Label(
            self.root,
            text="PID Gains",
            font=("Arial", 18, "bold")
        ).pack(pady=10)

        # ------------------------------------------------------------
        # PID gains
        # ------------------------------------------------------------

        self._make_param_row(
            self.root,
            "Kp (Proportional)",
            "Kp",
            0,
            100,
            "{:.4f}",
            hard_min=0
        )

        self._make_param_row(
            self.root,
            "Ki (Integral)",
            "Ki",
            0,
            10,
            "{:.4f}",
            hard_min=0
        )

        self._make_param_row(
            self.root,
            "Kd (Derivative)",
            "Kd",
            0,
            20,
            "{:.4f}",
            hard_min=0
        )

        # ------------------------------------------------------------
        # Setpoint limits
        # ------------------------------------------------------------

        cal = self.config['calibration']

        half_beam = (
            self.config.get(
                'beam_length_m',
                0.168
            ) / 2
        )

        pos_min = cal.get(
            'position_min_m'
        )

        pos_max = cal.get(
            'position_max_m'
        )

        if pos_min is None:
            pos_min = -half_beam

        if pos_max is None:
            pos_max = half_beam

        # ------------------------------------------------------------
        # Setpoint
        # ------------------------------------------------------------

        self._make_param_row(
            self.root,
            "Setpoint (meters)",
            "setpoint",
            pos_min,
            pos_max,
            "{:.4f}",
            hard_min=pos_min,
            hard_max=pos_max
        )

        # ------------------------------------------------------------
        # Apply button
        # ------------------------------------------------------------

        apply_frame = ttk.Frame(
            self.root
        )

        apply_frame.pack(
            pady=15
        )

        ttk.Button(
            apply_frame,
            text="Apply Parameters",
            command=self.apply_parameters
        ).pack(
            side=tk.LEFT,
            padx=5
        )

        # ------------------------------------------------------------
        # Other buttons
        # ------------------------------------------------------------

        button_frame = ttk.Frame(
            self.root
        )

        button_frame.pack(
            pady=10
        )

        ttk.Button(
            button_frame,
            text="Reset Integral",
            command=self.reset_integral
        ).pack(
            side=tk.LEFT,
            padx=5
        )

        ttk.Button(
            button_frame,
            text="Plot Results",
            command=self.plot_results
        ).pack(
            side=tk.LEFT,
            padx=5
        )

        ttk.Button(
            button_frame,
            text="Stop",
            command=self.stop
        ).pack(
            side=tk.LEFT,
            padx=5
        )

        # ------------------------------------------------------------
        # Status text (shows the ACTIVE parameters)
        # ------------------------------------------------------------

        self.status_label = ttk.Label(
            self.root,
            text=(
                f"Active parameters: Kp={self.Kp:.4f}, "
                f"Ki={self.Ki:.4f}, Kd={self.Kd:.4f}, "
                f"Setpoint={self.setpoint:.4f}"
            ),
            font=("Arial", 10)
        )

        self.status_label.pack(
            pady=10
        )

    # ================================================================
    # RESET INTEGRAL
    # ================================================================

    def reset_integral(self):
        """Clear integral error."""

        self.integral = 0.0

        print(
            "[RESET] Integral term reset"
        )

    # ================================================================
    # PLOT
    # ================================================================

    def plot_results(self):
        """Show matplotlib plots of position and control logs."""

        if not self.time_log:
            print(
                "[PLOT] No data to plot"
            )
            return

        # Report the real average loop rate
        if self.dt_log:
            avg_dt = sum(self.dt_log) / len(self.dt_log)

            print(
                f"[PLOT] Average loop time: {avg_dt * 1000:.1f} ms "
                f"({1.0 / avg_dt:.1f} Hz)"
            )

        fig, (ax1, ax2) = plt.subplots(
            2,
            1,
            figsize=(10, 8)
        )

        # ------------------------------------------------------------
        # Ball position
        # ------------------------------------------------------------

        ax1.plot(
            self.time_log,
            self.position_log,
            label="Ball Position",
            linewidth=2
        )

        ax1.plot(
            self.time_log,
            self.setpoint_log,
            label="Setpoint",
            linestyle="--",
            linewidth=2
        )

        ax1.set_ylabel(
            "Position (m)"
        )

        # Extra title padding leaves room for the gain labels above the axes
        ax1.set_title(
            f"Basic PID Control "
            f"(Kp={self.Kp:.4f}, "
            f"Ki={self.Ki:.4f}, "
            f"Kd={self.Kd:.4f})",
            pad=95
        )

        ax1.legend()
        ax1.grid(
            True,
            alpha=0.3
        )

        # ------------------------------------------------------------
        # Control output
        # ------------------------------------------------------------

        ax2.plot(
            self.time_log,
            self.control_log,
            label="Control Output",
            color="orange",
            linewidth=2
        )

        ax2.set_xlabel(
            "Time (s)"
        )

        ax2.set_ylabel(
            "Beam Angle (degrees)"
        )

        ax2.legend()

        ax2.grid(
            True,
            alpha=0.3
        )

        # ------------------------------------------------------------
        # Markers where gains were applied (labels sit above the axes)
        # ------------------------------------------------------------

        for t, kp, ki, kd in self.apply_log:
            for ax in (ax1, ax2):
                ax.axvline(
                    t,
                    color="gray",
                    linestyle=":",
                    linewidth=1.5,
                    alpha=0.8
                )

            ax1.text(
                t,
                1.01,
                f"Kp={kp:g} Ki={ki:g} Kd={kd:g}",
                transform=ax1.get_xaxis_transform(),
                rotation=90,
                va="bottom",
                ha="center",
                fontsize=7,
                color="dimgray",
                clip_on=False
            )

        plt.tight_layout()

        plt.show()

    # ================================================================
    # STOP
    # ================================================================

    def stop(self):
        """Stop everything and clean up."""

        self.running = False

        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    # ================================================================
    # RUN
    # ================================================================

    def run(self):
        """Entry point."""

        print(
            "[INFO] Starting Basic PID Controller"
        )

        print(
            "Change PID values using the GUI."
        )

        print(
            "Press 'Apply Parameters' to activate changes."
        )

        print(
            "Close camera window or click Stop to exit."
        )

        self.running = True

        # ------------------------------------------------------------
        # Start camera thread
        # ------------------------------------------------------------

        cam_thread = Thread(
            target=self.camera_thread,
            daemon=True
        )

        # ------------------------------------------------------------
        # Start control thread
        # ------------------------------------------------------------

        ctrl_thread = Thread(
            target=self.control_thread,
            daemon=True
        )

        cam_thread.start()
        ctrl_thread.start()

        # ------------------------------------------------------------
        # Build GUI
        # ------------------------------------------------------------

        self.create_gui()

        self.root.mainloop()

        # ------------------------------------------------------------
        # After GUI closes
        # ------------------------------------------------------------

        self.running = False

        print(
            "[INFO] Controller stopped"
        )


# ================================================================
# MAIN
# ================================================================

if __name__ == "__main__":

    try:
        controller = BasicPIDController()

        controller.run()

    except FileNotFoundError:
        print(
            "[ERROR] config.json not found. "
            "Run simple_autocal.py first."
        )

    except Exception as e:
        print(
            f"[ERROR] {e}"
        )