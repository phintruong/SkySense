# 🚀 SkySense → Flight Software / GNC MASTER PLAN (ALL-IN-ONE)

---

# 🎯 OBJECTIVE

Transform project from:

LiDAR obstacle detection system  

→ into →

Full **Guidance, Navigation, and Control (GNC)** system with:
- physics simulation  
- state estimation  
- control system  
- sensor fusion  

---

# 🧠 FINAL SYSTEM (HOW EVERYTHING CONNECTS)

[SENSORS] → [STATE ESTIMATION] → [CONTROL] → [DRONE DYNAMICS] → [SENSORS]

Closed-loop system.

---

# 📁 FINAL PROJECT STRUCTURE

src/
  simulation/
  navigation/
  control/
  sensors/
  telemetry/
  config/

main.py

---

# 📅 PHASE 0 — CLEANUP (Day 1)

## Goal
Prepare repo for GNC

## Tasks

- delete duplicate Logic/rplidar_reader.py  
- delete nul file  

- create folders:
  - src/simulation
  - src/navigation
  - src/control
  - src/sensors
  - src/telemetry
  - src/config

- move:
  - core/logic.py → navigation/obstacle_detector.py
  - hardware/rplidar_reader.py → sensors/lidar.py
  - hardware/hc_sr04_distance.py → sensors/ultrasonic.py

---

# 📅 PHASE 1 — SIMULATION (Day 2–5)

## Goal
Build a fake drone that follows physics

---

## 1. Drone State

File: simulation/drone_model.py

Variables:

position = [x, y, z]  
velocity = [vx, vy, vz]  
orientation = [roll, pitch, yaw]  
angular_velocity = [wx, wy, wz]  

---

## 2. Physics Update

Every timestep:

- apply gravity  
- apply thrust  
- compute acceleration  
- update velocity  
- update position  

Formula:

acceleration = (thrust + gravity) / mass  
velocity += acceleration * dt  
position += velocity * dt  

---

## 3. Rotation

angular_velocity += torque * dt  
orientation += angular_velocity * dt  

---

## 4. Environment

File: simulation/environment.py

- define obstacles  
- function:

get_distance(position, direction)

---

## 5. Sensor Simulation

File: simulation/sensor_sim.py

### IMU:
- acceleration  
- angular velocity  

### LiDAR:
- raycast distances  

### Noise:

value = true_value + random_noise  

---

# 📅 PHASE 2 — STATE ESTIMATION (Day 6–9)

## Goal
Estimate real state from noisy sensors

---

## 1. IMU

File: sensors/imu.py

Outputs:
- acceleration  
- angular velocity  

---

## 2. Kalman Filter

File: navigation/ekf.py

State:

[x, y, z, vx, vy, vz, roll, pitch, yaw]

---

## 3. Prediction

x += v * dt  
v += a * dt  
orientation += gyro * dt  

---

## 4. Update

error = measurement - prediction  
state += K * error  

---

## 5. State Estimator

File: navigation/state_estimator.py

- combine sensors  
- output full state  

---

# 📅 PHASE 3 — CONTROL SYSTEM (Day 10–13)

## Goal
Stabilize drone

---

## 1. PID

File: control/pid.py

error = target - current  
integral += error * dt  
derivative = (error - prev_error) / dt  

output = Kp*error + Ki*integral + Kd*derivative  

---

## 2. Attitude Controller

File: control/attitude_controller.py

Controls:
- roll  
- pitch  
- yaw  

---

## 3. Position Controller

File: control/position_controller.py

Input:
- target position  

Output:
- desired angles + thrust  

---

# 📅 PHASE 4 — MOTOR MIXING (Day 14)

File: control/motor_mixer.py

Convert:
- thrust  
- roll torque  
- pitch torque  
- yaw torque  

→ into:

- 4 motor outputs  

---

# 📅 PHASE 5 — MAIN LOOP (Day 15–16)

File: main.py

Loop:

1. read sensors  
2. estimate state  
3. compute control  
4. update simulation  
5. log data  

Use fixed timestep:

dt = 0.01 (100 Hz)

---

# 📅 PHASE 6 — DISTURBANCES (Day 17)

Add:

- wind force  
- random push  
- sensor failure  

---

# 📅 PHASE 7 — LOGGING (Day 18–19)

File: telemetry/logger.py

Store:
- true state  
- estimated state  
- control output  

---

# 📅 PHASE 8 — VISUALIZATION (Day 18–19)

- trajectory plot  
- error plot  
- optional animation  

---

# 📅 PHASE 9 — DEMOS (Day 20–22)

## REQUIRED

- hover stability  
- move to target  
- disturbance recovery  
- sensor failure recovery  

---

# 🔥 FINAL CHECKLIST

- simulation loop  
- IMU + noise  
- Kalman filter  
- PID control  
- motor mixing  
- disturbance handling  
- trajectory plots  

---

# 🧠 RESUME LINE

Designed and implemented a real-time drone guidance, navigation, and control system with physics-based simulation, sensor fusion using an Extended Kalman Filter, and PID stabilization.

---

# ⚡ IF TIME IS SHORT

Focus only on:

1. simulation  
2. PID  
3. Kalman filter  
4. trajectory plot  
5. disturbance demo  

---

# 🧨 FINAL GOAL

Turn:

Obstacle detection project  

Into:

Full autonomous flight control system

claude --resume 1ac4ce39-d203-4c79-b2e4-02474518acce                                                        