import serial
import time
import threading
import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# --- SERVO CONFIGURATION ---
PORT = 'COM7'
BAUD = 1000000
SERVO_ID = 1

class ServoDash(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("TASX Servo Control Center")
        self.geometry("1000x600")
        
        # Serial Setup
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.05)
        except Exception as e:
            print(f"Error: {e}")
            self.destroy()

        # Data Storage for Graphing
        self.history_x = list(range(50))
        self.history_pos = [0] * 50
        self.history_volt = [0] * 50
        self.history_speed = [0] * 50

        self.setup_ui()
        self.start_threads()

    def setup_ui(self):
        self.grid_columnconfigure(1, weight=1)
        
        # --- LEFT CONTROL PANEL ---
        self.ctrl_frame = ctk.CTkFrame(self, width=250)
        self.ctrl_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        ctk.CTkLabel(self.ctrl_frame, text="Servo Control", font=("Arial", 20, "bold")).pack(pady=10)

        # Position Slider
        ctk.CTkLabel(self.ctrl_frame, text="Target Position (1-4096)").pack()
        self.pos_slider = ctk.CTkSlider(self.ctrl_frame, from_=1, to=4096, command=self.update_pos)
        self.pos_slider.pack(pady=5)
        self.pos_label = ctk.CTkLabel(self.ctrl_frame, text="2048")
        self.pos_label.pack()

        # Torque Limit Slider
        ctk.CTkLabel(self.ctrl_frame, text="Torque Limit (0-1000)").pack(pady=(20,0))
        self.torque_slider = ctk.CTkSlider(self.ctrl_frame, from_=0, to=1000, command=self.update_torque)
        self.torque_slider.pack(pady=5)
        self.torque_val_label = ctk.CTkLabel(self.ctrl_frame, text="1000")
        self.torque_val_label.pack()

        # --- RIGHT GRAPH PANEL ---
        self.graph_frame = ctk.CTkFrame(self)
        self.graph_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.graph_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def update_pos(self, value):
        val = int(value)
        self.pos_label.configure(text=str(val))
        # Write to Target Position (0x2A)
        cmd = [SERVO_ID, 0x07, 0x03, 0x2A, val & 0xFF, (val >> 8) & 0xFF, 0, 0]
        self.send_cmd(cmd)

    def update_torque(self, value):
        val = int(value)
        self.torque_val_label.configure(text=str(val))
        # Torque Limit Address is 0x30
        cmd = [SERVO_ID, 0x04, 0x03, 0x30, val & 0xFF, (val >> 8) & 0xFF]
        self.send_cmd(cmd)

    def send_cmd(self, data):
        checksum = ~(sum(data) & 0xFF) & 0xFF
        full_cmd = bytearray([0xFF, 0xFF] + data + [checksum])
        self.ser.write(full_cmd)

    def start_threads(self):
        threading.Thread(target=self.data_loop, daemon=True).start()

    def data_loop(self):
        while True:
            # Read Position (0x38, 2 bytes), Speed (0x3A, 2 bytes), Voltage (0x3E, 1 byte)
            # We use a single read command to get a block of data for efficiency
            read_cmd = bytearray([0xFF, 0xFF, SERVO_ID, 0x04, 0x02, 0x38, 0x07, 0x00])
            read_cmd[-1] = ~(sum(read_cmd[2:-1]) & 0xFF) & 0xFF
            
            self.ser.write(read_cmd)
            time.sleep(0.04) # 25Hz Refresh Rate
            
            res = self.ser.read(self.ser.in_waiting)
            if len(res) >= 13: # Header(2) + ID(1) + Len(1) + Err(1) + Data(7) + Checksum(1)
                pos = res[5] | (res[6] << 8)
                speed = res[7] | (res[8] << 8)
                volt = res[11] / 10.0 # Voltage is at offset 6 from data start (0x3E - 0x38)
                
                # Update history
                self.history_pos.append(pos)
                self.history_volt.append(volt)
                self.history_speed.append(speed if speed < 32768 else speed - 65536)
                
                self.history_pos.pop(0)
                self.history_volt.pop(0)
                self.history_speed.pop(0)
                
                self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.plot(self.history_x, self.history_pos, label="Position", color="blue")
        # Secondary axis for voltage if needed, but keeping it simple:
        self.ax.legend(loc="upper left")
        self.ax.set_title("Live Servo Feedback")
        self.canvas.draw()

if __name__ == "__main__":
    app = ServoDash()
    app.mainloop()