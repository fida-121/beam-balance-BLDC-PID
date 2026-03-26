import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import re
import math
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation

class BeamBalanceGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PID Beam Balance Control")
        self.root.geometry("1400x900")
        self.root.configure(bg='#1a1a2e')
        
        # Serial port variables
        self.serial_port = None
        self.is_connected = False
        self.reading_thread = None
        
        # Data storage
        self.max_points = 100
        self.time_data = deque(maxlen=self.max_points)
        self.pitch_data = deque(maxlen=self.max_points)
        self.error_data = deque(maxlen=self.max_points)
        self.rpm1_data = deque(maxlen=self.max_points)
        self.rpm2_data = deque(maxlen=self.max_points)
        self.i1_data = deque(maxlen=self.max_points)
        self.i2_data = deque(maxlen=self.max_points)
        self.time_counter = 0
        
        # Current values
        self.current_pitch = 0.0
        self.current_error = 0.0
        self.current_p1 = 1200
        self.current_p2 = 1200
        self.current_rpm1 = 0
        self.current_rpm2 = 0
        self.current_i1 = 0.0
        self.current_i2 = 0.0
        
        self.create_widgets()
        self.setup_plots()
        
    def create_widgets(self):
        # Top Frame - Connection
        top_frame = tk.Frame(self.root, bg='#1a1a2e')
        top_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(top_frame, text="Serial Port:", bg='#1a1a2e', fg='white', font=('Arial', 10)).pack(side=tk.LEFT, padx=5)
        
        self.port_combo = ttk.Combobox(top_frame, width=15, state='readonly')
        self.port_combo.pack(side=tk.LEFT, padx=5)
        self.refresh_ports()
        
        tk.Button(top_frame, text="Refresh", command=self.refresh_ports, bg='#4a5568', fg='white', 
                 relief=tk.FLAT, padx=10).pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = tk.Button(top_frame, text="Connect", command=self.toggle_connection,
                                     bg='#48bb78', fg='white', relief=tk.FLAT, padx=20, font=('Arial', 10, 'bold'))
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.status_label = tk.Label(top_frame, text="● Disconnected", bg='#1a1a2e', fg='#fc8181', font=('Arial', 10, 'bold'))
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Main container
        main_frame = tk.Frame(self.root, bg='#1a1a2e')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Left Panel - Roll Indicator
        left_panel = tk.Frame(main_frame, bg='#2d3748', relief=tk.RAISED, borderwidth=2)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, padx=5, pady=5)
        
        tk.Label(left_panel, text="Beam Roll Indicator", bg='#2d3748', fg='white', 
                font=('Arial', 14, 'bold')).pack(pady=10)
        
        self.horizon_canvas = tk.Canvas(left_panel, width=350, height=350, bg='#1a1a2e', highlightthickness=0)
        self.horizon_canvas.pack(pady=10, padx=10)
        
        # Status values below horizon
        status_frame = tk.Frame(left_panel, bg='#2d3748')
        status_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.create_status_display(status_frame)
        
        # Right Panel - Charts
        right_panel = tk.Frame(main_frame, bg='#2d3748', relief=tk.RAISED, borderwidth=2)
        right_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        tk.Label(right_panel, text="Real-time Data", bg='#2d3748', fg='white',
                font=('Arial', 14, 'bold')).pack(pady=10)
        
        # Charts container
        self.charts_frame = tk.Frame(right_panel, bg='#2d3748')
        self.charts_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
    def create_status_display(self, parent):
        # Create grid of status boxes
        labels = [
            ("Roll Angle", "pitch_val", "#4299e1"),
            ("Error", "error_val", "#f6ad55"),
            ("PWM 1", "p1_val", "#48bb78"),
            ("PWM 2", "p2_val", "#48bb78"),
            ("RPM 1", "rpm1_val", "#9f7aea"),
            ("RPM 2", "rpm2_val", "#9f7aea"),
            ("Current 1", "i1_val", "#ecc94b"),
            ("Current 2", "i2_val", "#ecc94b")
        ]
        
        self.value_labels = {}
        
        for i, (label_text, var_name, color) in enumerate(labels):
            row = i // 2
            col = i % 2
            
            frame = tk.Frame(parent, bg='#1a1a2e', relief=tk.RAISED, borderwidth=1)
            frame.grid(row=row, column=col, padx=5, pady=5, sticky='nsew')
            
            tk.Label(frame, text=label_text, bg='#1a1a2e', fg='#a0aec0',
                    font=('Arial', 9)).pack(pady=(5, 0))
            
            val_label = tk.Label(frame, text="0.00", bg='#1a1a2e', fg=color,
                                font=('Arial', 16, 'bold'))
            val_label.pack(pady=(0, 5))
            
            self.value_labels[var_name] = val_label
            
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_columnconfigure(1, weight=1)
        
    def setup_plots(self):
        # Create figure with subplots
        self.fig = Figure(figsize=(9, 8), facecolor='#2d3748')
        
        # Gyro data subplot
        self.ax1 = self.fig.add_subplot(211, facecolor='#1a1a2e')
        self.ax1.set_title('Roll Angle & Error', color='white', fontsize=10, fontweight='bold')
        self.ax1.set_ylabel('Angle (°)', color='white')
        self.ax1.tick_params(colors='white')
        self.ax1.set_ylim(-30, 30)  # Fixed angle scale
        self.line_pitch, = self.ax1.plot([], [], 'c-', linewidth=2, label='Roll')
        self.line_error, = self.ax1.plot([], [], 'r-', linewidth=2, label='Error')
        self.ax1.legend(loc='upper right', facecolor='#2d3748', edgecolor='white', labelcolor='white')
        self.ax1.grid(True, alpha=0.3, color='gray')
        
        # Combined RPM and Current subplot
        self.ax2 = self.fig.add_subplot(212, facecolor='#1a1a2e')
        self.ax2.set_title('Motor RPM & Current', color='white', fontsize=10, fontweight='bold')
        self.ax2.set_xlabel('Time', color='white')
        self.ax2.set_ylabel('RPM', color='#9f7aea')
        self.ax2.tick_params(axis='y', colors='#9f7aea')
        self.ax2.set_ylim(0, 4000)  # Fixed RPM scale
        
        self.line_rpm1, = self.ax2.plot([], [], 'm-', linewidth=2, label='RPM 1')
        self.line_rpm2, = self.ax2.plot([], [], color='#ec4899', linewidth=2, label='RPM 2')
        
        # Create second y-axis for current
        self.ax3 = self.ax2.twinx()
        self.ax3.set_ylabel('Current (A)', color='#ecc94b')
        self.ax3.tick_params(axis='y', colors='#ecc94b')
        self.ax3.set_ylim(0, 5)  # Fixed current scale
        
        self.line_i1, = self.ax3.plot([], [], 'y-', linewidth=2, label='Current 1')
        self.line_i2, = self.ax3.plot([], [], color='#f59e0b', linewidth=2, label='Current 2')
        
        # Combine legends
        lines1, labels1 = self.ax2.get_legend_handles_labels()
        lines2, labels2 = self.ax3.get_legend_handles_labels()
        self.ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right', 
                       facecolor='#2d3748', edgecolor='white', labelcolor='white')
        
        self.ax2.grid(True, alpha=0.3, color='gray')
        
        self.fig.tight_layout()
        
        # Embed in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.charts_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Start animation
        self.ani = animation.FuncAnimation(self.fig, self.update_plots, interval=100, blit=False)
        
    def draw_artificial_horizon(self, roll_angle):
        self.horizon_canvas.delete("all")
        
        width = 350
        height = 350
        cx = width / 2
        cy = height / 2
        radius = 140
        
        # Convert angle to radians (inverted)
        roll_rad = math.radians(-roll_angle)
        
        # Draw outer circle
        self.horizon_canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius,
                                       outline='white', width=3, fill='#1a1a2e')
        
        # Draw rotated horizon line
        # Calculate endpoints of the horizon line
        line_length = radius * 2
        x1 = cx - line_length * math.cos(roll_rad)
        y1 = cy - line_length * math.sin(roll_rad)
        x2 = cx + line_length * math.cos(roll_rad)
        y2 = cy + line_length * math.sin(roll_rad)
        
        # Determine which side is sky and which is ground
        # Sky is above the horizon line (counterclockwise from line)
        # Create polygon for sky
        sky_points = []
        ground_points = []
        
        # Sample points around the circle
        for angle in range(360):
            ang_rad = math.radians(angle)
            px = cx + radius * math.cos(ang_rad)
            py = cy + radius * math.sin(ang_rad)
            
            # Check if point is above or below the horizon line
            # Using cross product to determine side
            cross = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)
            
            if cross < 0:  # Above horizon (sky)
                sky_points.extend([px, py])
            else:  # Below horizon (ground)
                ground_points.extend([px, py])
        
        # Draw sky (blue) - upper half relative to roll
        if len(sky_points) >= 6:
            self.horizon_canvas.create_polygon(sky_points, fill='#4a90e2', outline='')
        
        # Draw ground (brown) - lower half relative to roll
        if len(ground_points) >= 6:
            self.horizon_canvas.create_polygon(ground_points, fill='#8b4513', outline='')
        
        # Draw the horizon line itself
        self.horizon_canvas.create_line(x1, y1, x2, y2, fill='white', width=3)
        
        # Draw roll angle marks (±10°, ±20°, ±30°)
        for mark_angle in [-30, -20, -10, 0, 10, 20, 30]:
            mark_rad = math.radians(mark_angle)
            mark_x = cx + (radius - 20) * math.sin(mark_rad)
            mark_y = cy - (radius - 20) * math.cos(mark_rad)
            
            if mark_angle == 0:
                # Triangle at top for level
                self.horizon_canvas.create_polygon(
                    mark_x, mark_y - 10,
                    mark_x - 5, mark_y,
                    mark_x + 5, mark_y,
                    fill='white', outline='white'
                )
            else:
                # Small lines for angle marks
                mark_outer_x = cx + radius * math.sin(mark_rad)
                mark_outer_y = cy - radius * math.cos(mark_rad)
                self.horizon_canvas.create_line(mark_x, mark_y, mark_outer_x, mark_outer_y,
                                               fill='white', width=2)
        
        # Re-draw outer circle to clip
        self.horizon_canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius,
                                       outline='white', width=3)
        
        # Draw center reference marker (fixed wings)
        wing_length = 50
        wing_height = 3
        self.horizon_canvas.create_rectangle(cx - wing_length, cy - wing_height,
                                            cx - 15, cy + wing_height,
                                            fill='#fbbf24', outline='')
        self.horizon_canvas.create_rectangle(cx + 15, cy - wing_height,
                                            cx + wing_length, cy + wing_height,
                                            fill='#fbbf24', outline='')
        self.horizon_canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill='#fbbf24', outline='')
        
        # Draw beam visualization (inverted)
        beam_length = 120
        beam_x1 = cx + beam_length * math.cos(roll_rad)
        beam_y1 = cy + beam_length * math.sin(roll_rad)
        beam_x2 = cx - beam_length * math.cos(roll_rad)
        beam_y2 = cy - beam_length * math.sin(roll_rad)
        
        self.horizon_canvas.create_line(beam_x1, beam_y1, beam_x2, beam_y2,
                                       fill='#00ff00', width=6)
        
        # Draw balls at beam ends
        ball_radius = 8
        self.horizon_canvas.create_oval(beam_x1 - ball_radius, beam_y1 - ball_radius,
                                       beam_x1 + ball_radius, beam_y1 + ball_radius,
                                       fill='#ff4444', outline='white', width=2)
        self.horizon_canvas.create_oval(beam_x2 - ball_radius, beam_y2 - ball_radius,
                                       beam_x2 + ball_radius, beam_y2 + ball_radius,
                                       fill='#ff4444', outline='white', width=2)
        
        # Angle text
        self.horizon_canvas.create_text(cx, height - 30, text=f"{roll_angle:.2f}°",
                                       fill='white', font=('Arial', 18, 'bold'))
        
        # Direction indicators
        if roll_angle > 0:
            direction = "→ RIGHT"
        elif roll_angle < 0:
            direction = "← LEFT"
        else:
            direction = "LEVEL"
        
        self.horizon_canvas.create_text(cx, 30, text=direction,
                                       fill='#fbbf24', font=('Arial', 12, 'bold'))
        
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)
            
    def toggle_connection(self):
        if not self.is_connected:
            self.connect_serial()
        else:
            self.disconnect_serial()
            
    def connect_serial(self):
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("Error", "Please select a port")
            return
            
        try:
            self.serial_port = serial.Serial(port, 9600, timeout=1)
            self.is_connected = True
            self.connect_btn.config(text="Disconnect", bg='#f56565')
            self.status_label.config(text="● Connected", fg='#48bb78')
            
            # Start reading thread
            self.reading_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.reading_thread.start()
            
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            
    def disconnect_serial(self):
        self.is_connected = False
        if self.serial_port:
            self.serial_port.close()
        self.connect_btn.config(text="Connect", bg='#48bb78')
        self.status_label.config(text="● Disconnected", fg='#fc8181')
        
    def read_serial(self):
        while self.is_connected:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    self.parse_data(line)
            except Exception as e:
                print(f"Read error: {e}")
                break
                
    def parse_data(self, line):
        # Parse format: Pitch=X.XX | Err=X.XX | P1=XXXX P2=XXXX | RPM1=X RPM2=X | I1=X.XXA I2=X.XXA
        pattern = r'Pitch=([-\d.]+)\s*\|\s*Err=([-\d.]+)\s*\|\s*P1=(\d+)\s+P2=(\d+)\s*\|\s*RPM1=(\d+)\s+RPM2=(\d+)\s*\|\s*I1=([-\d.]+)A\s+I2=([-\d.]+)A'
        match = re.search(pattern, line)
        
        if match:
            try:
                self.current_pitch = float(match.group(1))
                self.current_error = float(match.group(2))
                self.current_p1 = int(match.group(3))
                self.current_p2 = int(match.group(4))
                self.current_rpm1 = int(match.group(5))
                self.current_rpm2 = int(match.group(6))
                self.current_i1 = float(match.group(7))
                self.current_i2 = float(match.group(8))
                
                # Add to data queues
                self.time_counter += 1
                self.time_data.append(self.time_counter)
                self.pitch_data.append(self.current_pitch)
                self.error_data.append(self.current_error)
                self.rpm1_data.append(self.current_rpm1)
                self.rpm2_data.append(self.current_rpm2)
                self.i1_data.append(self.current_i1)
                self.i2_data.append(self.current_i2)
                
                # Update UI
                self.root.after(0, self.update_ui)
                
            except ValueError as e:
                print(f"Parse error: {e}")
                
    def update_ui(self):
        # Update status labels
        self.value_labels['pitch_val'].config(text=f"{self.current_pitch:.2f}°")
        self.value_labels['error_val'].config(text=f"{self.current_error:.2f}°")
        self.value_labels['p1_val'].config(text=f"{self.current_p1}")
        self.value_labels['p2_val'].config(text=f"{self.current_p2}")
        self.value_labels['rpm1_val'].config(text=f"{self.current_rpm1}")
        self.value_labels['rpm2_val'].config(text=f"{self.current_rpm2}")
        self.value_labels['i1_val'].config(text=f"{self.current_i1:.2f}A")
        self.value_labels['i2_val'].config(text=f"{self.current_i2:.2f}A")
        
        # Update roll indicator
        self.draw_artificial_horizon(self.current_pitch)
        
    def update_plots(self, frame):
        if len(self.time_data) > 0:
            # Update gyro plot (fixed scale)
            self.line_pitch.set_data(list(self.time_data), list(self.pitch_data))
            self.line_error.set_data(list(self.time_data), list(self.error_data))
            self.ax1.relim()
            self.ax1.autoscale_view(scalex=True, scaley=False)  # Only autoscale x-axis
            
            # Update RPM plot (fixed scale)
            self.line_rpm1.set_data(list(self.time_data), list(self.rpm1_data))
            self.line_rpm2.set_data(list(self.time_data), list(self.rpm2_data))
            self.ax2.relim()
            self.ax2.autoscale_view(scalex=True, scaley=False)  # Only autoscale x-axis
            
            # Update current plot (fixed scale)
            self.line_i1.set_data(list(self.time_data), list(self.i1_data))
            self.line_i2.set_data(list(self.time_data), list(self.i2_data))
            
        return self.line_pitch, self.line_error, self.line_rpm1, self.line_rpm2, self.line_i1, self.line_i2

def main():
    root = tk.Tk()
    app = BeamBalanceGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()