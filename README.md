# Formula Student TCS Performance Dashboard

An interactive, real-time **Traction Control System (TCS)** simulation dashboard designed for Formula Student racing dynamics. This tool allows engineers to simulate, tune, and analyze vehicle acceleration runs using high-fidelity mathematical models.

## 🚀 Features

- **Pacejka Magic Formula:** Simulates non-linear tyre force dynamics (B, C, E parameters).
- **Dynamic Weight Transfer & Aero:** Factors in longitudinal weight transfer under acceleration, aerodynamic downforce (\(C_l\)), and aerodynamic drag (\(C_d\)).
- **PID Control Tuning:** Real-time adjustable Proportional, Integral, and Derivative gains to optimize target wheel slip.
- **Drivetrain Modeling:** Includes gear ratios, torque-RPM curves (constant torque vs. constant power regions), and a hard RPM limiter.
- **Industry Standard Timers:** Built-in automatic timers for both **0-100 km/h** sprint and the official **75-meter Formula Student Acceleration Event**.
- **Data Export:** Telemetry logging that exports full-run data into a `.csv` file and generates a high-resolution `.png` analysis plot.

## 🛠️ Installation & Usage

1. **Clone the repository:**
   ```bash
   git clone https://github.com
   formula-student-tcs-simulator
   ```

2. **Install dependencies:**
   Make sure you have Python 3 installed, then run:
   ```bash
   pip install -r requirements.txt
   ```

3. **Run the simulator:**
   ```bash
   python tcs_simulator.py
   ```

## 📊 How It Works (Vehicle Physics)

The simulation updates every 30ms and calculates:
1. **Vertical Load ($F_z$):** 
   $$F_z = W_{static} + \Delta W_{transfer} + F_{downforce}$$
2. **Tractive Force ($F_x$):** Calculated using the Pacejka Tyre Model based on the current wheel slip ratio.
3. **Net Acceleration:** Derived from the net force after subtracting aerodynamic drag and rolling resistance ($C_{rr}$).

## 🖥️ UI Overview

* **Tabbed Parameters:** Fine-tune PID gains, Aerodynamics, Drivetrain constraints, Tyre properties, and Vehicle Geometry on the fly.
* **Live Telemetry Graphs:** Real-time Matplotlib plots tracking applied throttle, vehicle speed, and wheel speed.
* **Sidebar Monitors:** Quick view for instant G-force, TCS status, and event timers.

---
*Developed by Dimitrios Chantes.*
