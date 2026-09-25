import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import math
import csv


class Traction_Control_Sim:
    
    
    def __init__(self):

        self.running = False

        self.last_accel = 0
        
        self.root = tk.Tk()
        self.root.title("TCS Performance Dashboard")
        self.root.geometry("1200x800")
        self.root.configure(bg="#1e1e1e")

        # Performance Timer
        self.start_time = None
        self.accel_time_0_100 = 0.0
        self.timer_running = False

        # Distance & 75m Timer
        self.distance = 0.0
        self.accel_time_75m = 0.0
        self.timer_75m_active = False

        # Aerodynamic Parameters
        self.m = 230          # kg
        self.rho = 1.225      # kg/m^3
        self.Cd = 0.75        # Drag coefficient
        self.Cl = 3.0         # Lift coefficient
        self.A = 0.8          # m^2 (Frontal Area)
        self.Crr = 0.02       # Rolling resistance
        
        self.g = 9.81

        # Dimention parameters
        self.h_cg = 0.25    # Height of the weight center
        self.L = 1.53       # Legth of semi-axle

        # Pacejka Parameters (Tyre Force)
        self.B = 10.0
        self.C = 1.9
        self.E = 0.97

        # RPM Limits - Engine Parameters
        self.base_speed_rpm = 3500
        self.max_engine_rpm = 6000

        # PID Gains
        self.Kp = 5
        self.Ki = 0.5
        self.Kd = 0.1

        # PID Parameters
        self.target_slip = 0.22
        self.error_sum = 0
        self.last_error = 0

        # Max Engine Power
        self.Tmax = 110 # For an electrical-powered motor in student formula the values are 100Nm give or take
        self.G = 5 # This value has to do with the gear and the differential
        self.n = 0.90 # This value refers to the performance of the engine
        self.r = 0.23 # This is the radius of the wheel that can go from 0.20m to 0.25m in student formula
        
        self.Fmax = self.Tmax * self.G * self.n / self.r

        self.mu = 1.6 # This is the value of the coefficient of friction and in warmed-up and effective tires it varies from 1.4 to 1.6
        self.W_static = 230 * 0.60 * 9.81 # This is the weight of the driving_axle with 60% of the total weight dropped on his side
        

        # Data
        self.data_history = {
            "throttle": [0] * 100,
            "speed": [0] * 100,
            "wheel_speed": [0] *100,
            "slip": [0] * 100
        }

        self.data_to_save = {
            "throttle": [],
            "speed": [],
            "wheel_speed": [],
            "slip": []
        }

        # In_simulation variables
        self.time_scale = 1

        
        # Parameters for the upper layout

        # PID & Traction
        self.kp_var = tk.DoubleVar(value=5.0)
        self.ki_var = tk.DoubleVar(value=0.5)
        self.kd_var = tk.DoubleVar(value=0.1)
        self.target_slip_var = tk.DoubleVar(value=0.22)

        # Vehicle & Aero
        self.mu_var = tk.DoubleVar(value=1.6)
        self.mass_var = tk.DoubleVar(value=230.0)
        self.A_var = tk.DoubleVar(value=0.8)
        self.rho_var = tk.DoubleVar(value=1.225)
        self.Crr_var = tk.DoubleVar(value=0.02)
        self.cl_var = tk.DoubleVar(value=3.0)  # Lift Coefficient
        self.cd_var = tk.DoubleVar(value=0.75) # Drag Coefficient

        # Drivetrain
        self.gr_var = tk.DoubleVar(value=5.0)   # Gear Ratio
        self.tmax_var = tk.DoubleVar(value=110.0) # Max Torque (Nm)
        self.base_speed_rpm_var = tk.DoubleVar(value=3500)
        self.max_engine_rpm_var = tk.DoubleVar(value=+ 6000)

        # Pacejka Parameters
        self.p_b_var = tk.DoubleVar(value=10.0)
        self.p_c_var = tk.DoubleVar(value=1.9)
        self.p_e_var = tk.DoubleVar(value=0.97)

        # Geometry
        self.h_cg_var = tk.DoubleVar(value=0.25)
        self.l_axle_var = tk.DoubleVar(value=1.53)
        self.wheel_r_var = tk.DoubleVar(value=0.23)

        self.setup_ui()
        self.setup_plots()
        self.root.mainloop()

    
    def setup_ui(self):

         # Main upper frame
        self.top_container = tk.Frame(self.root, bg="#2d2d2d")
        self.top_container.pack(side=tk.TOP, fill="x", padx=5, pady=5)

        # Creation of Tabs (Notebook)
        self.notebook = ttk.Notebook(self.top_container)
        self.notebook.pack(side=tk.LEFT, fill="both", expand=True)

        tabs = {
            "PID": [("Kp", self.kp_var, 0, 20, 0.1), ("Ki", self.ki_var, 0, 5, 0.01), ("Kd", self.kd_var, 0, 2, 0.01), ("Target Slip", self.target_slip_var, 0, 0.5, 0.01)],
            "AERO": [("Mass", self.mass_var, 150, 400, 5), ("Cl", self.cl_var, 0, 5, 0.1), ("Cd", self.cd_var, 0, 1.5, 0.01), ("Rho", self.rho_var, 1.0, 1.3, 0.005)],
            "DRIVE": [("Max Nm", self.tmax_var, 50, 250, 5), ("Max RPM", self.max_engine_rpm_var, 2000, 10000, 500), ("MAX TORQUE RPM", self.base_speed_rpm_var, 500, 7000, 500), ("Gear Ratio", self.gr_var, 2, 10, 0.1), ("Crr", self.Crr_var, 0, 0.1, 0.005)],
            "TYRE": [("B (Stiff)", self.p_b_var, 1, 20, 0.5), ("C (Shape)", self.p_c_var, 1, 3, 0.1), ("E (Curv)", self.p_e_var, 0, 1.5, 0.05), ("Mu", self.mu_var, 0.5, 2.5, 0.05)],
            "GEO": [("CG Height", self.h_cg_var, 0.1, 0.5, 0.01), ("Wheelbase", self.l_axle_var, 1.0, 2.0, 0.05), ("Radius", self.wheel_r_var, 0.15, 0.35, 0.01)]
        }

        for name, params in tabs.items():
            frame = tk.Frame(self.notebook, bg="#2d2d2d")
            self.notebook.add(frame, text=f" {name} ")
            for label, var, f, t, r in params:
                f_box = tk.Frame(frame, bg="#2d2d2d")
                f_box.pack(side=tk.LEFT, padx=10, pady=5)
                tk.Label(f_box, text=label, bg="#2d2d2d", fg="white", font=("Arial", 7)).pack()
                tk.Scale(f_box, from_=f, to=t, variable=var, orient="horizontal", resolution=r, length=90, bg="#2d2d2d", fg="#00ffcc", highlightthickness=0, font=("Arial", 7)).pack()

        # Right Buttons
        btn_frame = tk.Frame(self.top_container, bg="#2d2d2d")
        btn_frame.pack(side=tk.RIGHT, padx=10)
        self.run_btn = tk.Button(btn_frame, text="RUN", command=self.toggle_simulation, 
                         bg="#28a745", fg="white", font=("Arial", 10, "bold"), width=12)
        self.run_btn.pack(pady=5, fill="x")
        tk.Button(btn_frame, text="EXPORT", command=self.save_results, bg="#00ffcc", font=("Arial", 8, "bold")).pack(pady=2, fill="x")
        tk.Button(btn_frame, text="RESET", command=self.reset_simulation, bg="#444", fg="white").pack(pady=2, fill="x")
            
        self.sidebar = tk.Frame(self.root, bg="#2d2d2d", width=250)
        self.sidebar.pack(side=tk.LEFT, fill="y", padx=5, pady=5)

        tk.Label(self.sidebar, text="CONTROLS", font=("Segoe UI", 14, "bold"), 
                 bg="#2d2d2d", fg="white").pack(pady=20)

        # Throttle Slider
        tk.Label(self.sidebar, text="Applied Throttle", bg="#2d2d2d", fg="lightgrey").pack()
        self.applied_throtle = tk.Scale(self.sidebar, from_=100, to=0, orient="vertical", 
                                        length=300, bg="#2d2d2d", fg="white", 
                                        highlightthickness=0, troughcolor="#404040")
        self.applied_throtle.pack(pady=10)

        # G-Force Display
        self.g_label_title = tk.Label(self.sidebar, text="ACCELERATION", bg="#2d2d2d", fg="gray")
        self.g_label_title.pack(pady=(20, 0))
    
        self.g_value_label = tk.Label(self.sidebar, text="0.00 G", font=("Segoe UI", 24, "bold"), 
                                  bg="#2d2d2d", fg="#00ffcc")
        self.g_value_label.pack()

        # TCS Display
        self.tcs_enabled = tk.BooleanVar(value=False)
        self.tcs_button = tk.Checkbutton(self.sidebar, text="Traction Control", 
                                        variable=self.tcs_enabled, 
                                        bg="#2d2d2d", fg="#00ffcc", 
                                        selectcolor="#1e1e1e", font=("Arial", 10))
        self.tcs_button.pack(pady=20)

        self.tcs_status_label = tk.Label(self.sidebar, text="TCS INACTIVE", bg="#2d2d2d", fg="gray")
        self.tcs_status_label.pack()

        # 0-100 Timer Display
        tk.Label(self.sidebar, text="0-100 km/h TIMER", bg="#2d2d2d", fg="gray").pack(pady=(20, 0))
        self.timer_label = tk.Label(self.sidebar, text="0.00 s", font=("Segoe UI", 24, "bold"), 
                                    bg="#2d2d2d", fg="#ffcc00")
        self.timer_label.pack()

        # 75m Timer Display
        tk.Label(self.sidebar, text="75m ACCEL TIME", bg="#2d2d2d", fg="gray").pack(pady=(20, 0))
        self.timer_75m_label = tk.Label(self.sidebar, text="0.00 s", font=("Segoe UI", 24, "bold"), 
                                        bg="#2d2d2d", fg="#00ffcc")
        self.timer_75m_label.pack()

        # The two previous displays are only visible in bigger screens

    def save_results(self):
        file_path = filedialog.asksaveasfilename(
            title="Save Full Simulation Results",
            defaultextension="",
            filetypes=[("All Files", "*.*")]
        )
        
        if not file_path:
            return

        try:
            # --- A. Save CSV Data ---
            csv_path = file_path + "_data.csv"
            with open(csv_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["PARAMETER LOG", f"Kp: {self.Kp}", f"Mu: {self.mu}", f"Slip: {self.target_slip}"])
                writer.writerow(["Index", "Throttle (%)", "Car Speed (km/h)", "Wheel Speed (km/h)"])
                
                # Using data_to_save for full run history
                for i in range(len(self.data_to_save["speed"])):
                    writer.writerow([i, self.data_to_save["throttle"][i], 
                                    self.data_to_save["speed"][i], 
                                    self.data_to_save["wheel_speed"][i]])

            # --- B. Create a Separate Figure for the Full Run Image ---
            # We create a new figure so it includes ALL data points from data_to_save
            fig_save, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), facecolor='#1a1a1a')
            plt.subplots_adjust(hspace=0.4)

            # Plot 1: Full Throttle History
            ax1.set_facecolor('#1a1a1a')
            ax1.plot(self.data_to_save["throttle"], color="#00ffcc", linewidth=1.5)
            ax1.set_title("FULL RUN THROTTLE INPUT", color="gray", fontsize=10, fontweight='bold')
            ax1.grid(True, color="#333333", alpha=0.5)

            # Plot 2: Full Speed Dynamics (Car vs Wheel)
            ax2.set_facecolor('#1a1a1a')
            ax2.plot(self.data_to_save["speed"], color="#ff2d55", linewidth=2, label="Car Speed")
            ax2.plot(self.data_to_save["wheel_speed"], color="#ffcc00", linewidth=1.5, alpha=0.7, label="Wheel Speed")
            ax2.set_title("FULL RUN VEHICLE DYNAMICS", color="gray", fontsize=10, fontweight='bold')
            ax2.legend(frameon=False, labelcolor="white")
            ax2.grid(True, color="#333333", alpha=0.5)

            # Add Watermark to the save-only figure
            watermark_text = "TCS SIMULATOR | Developed by Dimitrios Chantes"
            fig_save.text(0.95, 0.02, watermark_text, fontsize=8, color='gray', alpha=0.5, ha='right', style='italic')

            # Save the full-history plot
            img_path = file_path + "_plot.png"
            fig_save.savefig(img_path, dpi=300, bbox_inches='tight', facecolor='#1a1a1a')
            
            # Close the temporary figure to free up memory
            plt.close(fig_save)

            messagebox.showinfo("Success", f"Full run saved (Data & High-Res Plot)!")
            
        except Exception as e:
            messagebox.showerror("Error", f"Save failed: {str(e)}")


    def reset_simulation(self):

        # 1. Reset physical state variables
        self.distance = 0.0
        self.last_accel = 0.0
        self.error_sum = 0.0
        self.last_error = 0.0
        
        # 2. Reset performance timers and status flags
        self.accel_time_0_100 = 0.0
        self.accel_time_75m = 0.0
        self.timer_running = False
        self.timer_75m_active = False
        
        # 3. Clear data history arrays for real-time plotting
        for key in self.data_history:
            # Re-initialize with a list of zeros to 'flatten' the graphs
            self.data_history[key] = [0.0] * 100
            self.data_to_save[key] = []
            
        # 4. Update UI labels to reflect the reset state
        self.timer_label.config(text="0.00 s", fg="#ffcc00")
        self.timer_75m_label.config(text="0.00 s", fg="#00ffcc")
        self.g_value_label.config(text="0.00 G")
        self.tcs_status_label.config(text="TCS INACTIVE", fg="gray")
        
        # 5. Stop the simulation loop execution if a Run button exists
        self.running = False
        if hasattr(self, 'run_btn'):
            self.run_btn.config(text="RUN", bg="#28a745")
            
        # 6. Refresh the canvas to show the cleared data
        self.update_graphs()


    def toggle_simulation(self):

        self.running = not self.running
        if self.running:
            self.update_parameters()
            self.simulation_loop()
            self.run_btn.config(text="STOP", bg="#dc3545")
        else:
            self.run_btn.config(text="RUN", bg="#28a745")


    def update_parameters(self):
        # PID & Traction
        self.Kp = self.kp_var.get()
        self.Ki = self.ki_var.get()
        self.Kd = self.kd_var.get()
        self.target_slip = self.target_slip_var.get()

        # Vehicle & Aero
        self.mu = self.mu_var.get()
        self.m = self.mass_var.get()
        self.A = self.A_var.get()
        self.rho = self.rho_var.get()
        self.Crr = self.Crr_var.get()
        self.Cl = self.cl_var.get()
        self.Cd = self.cd_var.get()

        # Drivetrain
        self.G = self.gr_var.get()
        self.Tmax = self.tmax_var.get()
        self.max_engine_rpm = self.max_engine_rpm_var.get()
        self.base_speed_rpm = self.base_speed_rpm_var.get()
        self.Fmax = self.Tmax * self.G * self.n / self.r
        self.W_static = self.m * 0.60 * self.g

        # Pacejka Parameters
        self.B = self.p_b_var.get()
        self.C = self.p_c_var.get()
        self.E = self.p_e_var.get()

        # Geometry
        self.h_cg = self.h_cg_var.get()
        self.L = self.l_axle_var.get()
        self.r = self.wheel_r_var.get()

    
    def setup_plots(self):
        
        plt.style.use('dark_background')
        
        self.fig, (self.ax_throttle, self.ax_speed) = plt.subplots(2, 1, figsize=(8, 8))
        self.fig.tight_layout(pad=4.0)
        
        self.line_throttle, = self.ax_throttle.plot(self.data_history["throttle"], color="#00ffcc")
        self.line_speed, = self.ax_speed.plot(self.data_history["speed"], color="#ff4444")
        self.line_wheel, = self.ax_speed.plot(self.data_history["wheel_speed"], color="#ffcc00", label="Wheel")
        self.ax_speed.legend(loc="upper left", fontsize=8)

        
        for ax, title in zip([self.ax_throttle, self.ax_speed], ["Throttle (%)", "Vehicle Speed (km/h)"]):
            ax.set_title(title, fontsize=10, color="gray")
            ax.set_ylim(0, 200)
            ax.grid(True, alpha=0.2)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill="both", expand=True, padx=10, pady=10)

    
    def update_graphs(self):
        
        self.line_throttle.set_ydata(self.data_history["throttle"])
        self.line_speed.set_ydata(self.data_history["speed"])
        self.line_wheel.set_ydata(self.data_history["wheel_speed"])
        self.canvas.draw_idle()


    def compute_acceleration(self, v, slip):

        # Weight transfer
        weight_transfer = (self.m * self.last_accel * self.h_cg) / self.L

        # Downforce
        Downforce = 0.5 * self.rho * self.A * self.Cl * v**2

        total_vertical_weight = self.W_static + weight_transfer + Downforce
        D = total_vertical_weight * self.mu

        F_air = 0.5 * self.rho * self.A * self.Cd * (v**2)
        F_roll = self.Crr * self.m * self.g
        self.Fx = D * math.sin(self.C * math.atan(self.B * slip - self.E * (self.B * slip - math.atan(self.B * slip))))
        F_net = self.Fx - (F_air + F_roll)
        
        self.last_accel = F_net / self.m
        
        return self.last_accel

    
    def simulation_loop(self):
        
        if (not self.running): return
        
        # 1. Parameters and current speeds in m/s
        dt = 0.03 * self.time_scale
        step_dt = dt / 10
        Ktcs = 1.0
        v_car = self.data_history["speed"][-1] / 3.6
        v_wheel = self.data_history["wheel_speed"][-1] / 3.6
        
        for _ in range(10):

            Ktcs = 1.0 # reseting

            # 2. Slip
            if v_car > 0.5:
                slip = (v_wheel - v_car) / v_car
            else:
                slip = (v_wheel - v_car) / (v_car + 0.5) # Forcing the slip to not burst in really low speeds
                
            slip = max(0, min(slip, 1.0))

            # Traction Control System - TCS -- PID --
            if self.tcs_enabled.get() and v_car > 3.0: # Launch Logic
                self.tcs_status_label.config(text="TCS ACTIVE", fg="#00ffcc")

                # TCS LOGIC
                error = slip - self.target_slip

                self.error_sum += error * step_dt
                self.error_sum = max(-1, min(self.error_sum, 1))
                error_dif = (error - self.last_error) / step_dt

                pid_output = (self.Kp * error) + (self.Ki * self.error_sum) + (self.Kd * error_dif)
                Ktcs = max(0.40, min(1.0, 1.0 - pid_output)) # Ktcs >= 0.4 to avoid low accelaration in low speeds
                self.last_error = error
            
            else:
                Ktcs = 1
                self.error_sum = 0
                self.last_error = 0
                self.tcs_status_label.config(text="TCS INACTIVE", fg="gray")

            # 3. Computation of accelaration
            accel = Ktcs * self.compute_acceleration(v_car, slip)
            
            # 4. New car speed and distance
            v_car = max(0, v_car + (accel * step_dt))
            self.distance += v_car * step_dt

            # 5. RPM computation
            wheel_rpm = (v_wheel / (2 * math.pi * self.r)) * 60
            engine_rpm = wheel_rpm * self.G

            # RPM Checking
            # 6. Torque and new wheel speed
            throttle = self.applied_throtle.get()
            max_torque = (throttle / 100) * (self.Fmax * self.r)

            if engine_rpm < self.base_speed_rpm:
                # Constant Torque Area
                T_engine = max_torque * Ktcs
            elif engine_rpm < self.max_engine_rpm:
                # Constant Power Area (Torqye decreases as RPM increase)
                T_engine = (max_torque * (self.base_speed_rpm / engine_rpm)) * Ktcs
            else:
                # Hard Limiter
                T_engine = 0
            

            Jw = 0.3
            damping = 0.1 * v_wheel
            # dv_wheel = (Torque_net / Jw) * dt * radius
            dv_wheel = ((T_engine - (self.Fx * self.r) - damping) / Jw) * step_dt * self.r
            v_wheel= v_wheel + dv_wheel

            if v_wheel > v_car + 16.0:
                v_wheel = v_car + 16.0
            if v_wheel < v_car:
                v_wheel = v_car


        # 6. Data update
        self.data_history["throttle"].append(throttle)
        self.data_history["throttle"].pop(0)
        self.data_history["speed"].append(v_car * 3.6)
        self.data_history["speed"].pop(0)
        self.data_history["wheel_speed"].append(v_wheel * 3.6)
        self.data_history["wheel_speed"].pop(0)

        self.data_to_save["throttle"].append(throttle)
        self.data_to_save["speed"].append(v_car * 3.6)
        self.data_to_save["wheel_speed"].append(v_wheel * 3.6)
        
        # UI Updates 
        
        # 0-100km/h counter
        current_speed_kmh = v_car * 3.6
        
        if current_speed_kmh > 1.0 and current_speed_kmh < 100.0:
            self.accel_time_0_100 += dt 
            self.timer_label.config(text=f"{self.accel_time_0_100:.2f} s")

        if current_speed_kmh >= 100.0: # Stoping at the right speed
            self.timer_label.config(text=f"{self.accel_time_0_100:.2f} s", fg="#00ff00")

        if current_speed_kmh < 1.0: # Reseting
            self.accel_time_0_100 = 0.0

        # 75m distance counter
        if self.distance > 0.1 and self.distance < 75.0:
            self.accel_time_75m += dt
            self.timer_75m_label.config(text=f"{self.accel_time_75m:.2f} s")

        if self.distance >= 75.0:
            self.timer_75m_label.config(fg="#00ff00")

        if v_car < 0.2:
            self.distance = 0.0
            self.accel_time_75m = 0.0
            self.timer_75m_label.config(text="0.00 s", fg="#00ffcc")

        self.g_value_label.config(text=f"{accel / 9.81:.2f} G")
        self.update_graphs()
        self.root.after(30, self.simulation_loop)


if __name__ == "__main__":
    Traction_Control_Sim()