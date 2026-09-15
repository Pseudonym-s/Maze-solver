# STM32 Micromouse — Maze Solving Robot

A modular, sensor-fused micromouse built around the **STM32F401 "Black Pill"** MCU, designed to autonomously map and solve a maze using a **Flood-Fill algorithm with A\*-style optimized path planning**, including diagonal shortcuts once the maze is known.

The robot is built as a **stack of custom PCB modules** — a black "core" stack (motor driver + sensor interconnect + MCU) that is fully finished and working, and a purple "add-on" stack (auxiliary sensor/IMU/power management boards) that is still a work in progress.

---
[![Maze-solver robot](images/img1.png)](images/img1.png)
[![Maze-solver robot](images/img2.png)](images/img2.png)
## 1. Hardware Overview

| Component | Role |
|---|---|
| **STM32F401CC "Black Pill"** | Main MCU — runs the flood-fill/A\* solver, motor control loop, and sensor fusion |
| **DRV8833 Dual H-Bridge Motor Driver (×2)** | Drives the 4× N20 gear motors independently for differential/diagonal steering |
| **N20 Micro Gear Motors w/ Encoders (×4)** | Drivetrain with quadrature encoders for precise distance/speed tracking |
| **MPU6050 IMU** | 6-axis gyro + accelerometer for heading correction and smooth, accurate turns |
| **VL53L0X ToF Sensors** | Time-of-flight distance sensors used for wall detection ahead/left/right |
| **2S LiPo Battery Packs (×2, 3.7V cells in series)** | Onboard power source, stacked in the center of the chassis |
| **Tactile Push Buttons** | On-board mode selection (e.g., explore vs. speed-run) and PID/threshold tuning |
| **Status LEDs (WS2812-style + discrete)** | Visual feedback for mode, battery, and debug states |

---

## 2. PCB Architecture — Modular Stack Design

The chassis is built as a **stack of interlocking PCB modules**, each handling one subsystem. This modular approach made debugging and assembly easier, since each board could be tested independently before being stacked and wired together.

### ✅ Black PCB Stack — Finished, Module-Based Core
[![Maze-solver](img/img3.png)](img/img3.png)
[![Maze-solver](img/img4.png)](img/img4.png)
[![Maze-solver robot](images/img5.png)](images/img5.png)
This is the working heart of the robot:
- **Top module** — STM32F401 "Black Pill" dev board, plugged into header rows, carrying USB, BOOT0/NRST buttons, and breaking out all GPIO used by the lower modules.
- **Mid module (motor driver board)** — Houses the two DRV8833 driver ICs (labeled `U13`), regulator/protection circuitry, indicator LED (`LED5`), and 4 motor headers (`MOTOR5`–`MOTOR8`). Connects to the sensor headers (`SENS5`–`SENS8`) on its outer wings.
- **Base/sensor interconnect layer** — Provides breakout headers for all 8 sensor ports (`SENS1`–`SENS8`) that plug into the ToF and IR wall-detection sensors mounted at the front/sides of the chassis.
- Battery packs are physically sandwiched between the motor driver board and the Black Pill layer, with the balance/output wiring routed through the middle.

This stack is fully assembled, soldered, wired, and functional — it's what actually drives the robot and reads the wall sensors today.

### 🚧 Purple PCB Stack — Auxiliary Boards, Not Fully Finished
[![Maze-solver robot](images/img6.png)](images/img6.png)
[![Maze-solver robot](images/img7.png)](images/img7.png)
[![Maze-solver robot](images/img8.png)](images/img8.png)
The purple boards are a secondary expansion stack meant to add more capability, but they are **only partially populated/assembled**:
- One purple board carries a QFP-package MCU/sensor IC (`U1`), crystal (`X1`), decoupling caps, and a duplicate set of motor/sensor headers (`MOTOR1`–`MOTOR4`, `SENSE1`–`SENSE4`) — intended as a secondary driver/sensor expansion board.
- A second purple board carries additional ICs (`U3`, `U9`), a trim potentiometer, and more tactile switches (`SW4`–`SW7`) for extra tuning/mode inputs.
- Several footprints on these boards are unpopulated (missing passives, unsoldered header pins, bare pads), and the interconnect header (`HM10`) is not yet wired into the main stack.
- **Status:** designed and fabricated, but incomplete — reserved for future upgrades (e.g., extra IMU/mag, additional switch bank, or a secondary compute/logic board).

---

## 3. How It Works

### Wall Detection
IR/ToF (VL53L0X) sensors mounted on the front and side sensor headers continuously measure distance to the nearest wall in each direction. Readings are thresholded to classify each of the three (or more) adjacent maze cells as **open** or **wall**.

### Movement & Odometry
Each N20 motor's quadrature encoder feeds pulse counts back to the STM32, letting the firmware track exact distance traveled and detect slippage or stalls. This gives cell-accurate positioning as the robot moves through the maze grid.

### Turning & Orientation — IMU + Kalman Filter
The MPU6050 IMU provides gyroscope and accelerometer data for heading. Because raw gyro data drifts and raw accelerometer data is noisy, a **Kalman filter** fuses the two together to produce a smooth, accurate heading estimate. This is what allows the robot to execute clean 90°/45° turns without over- or under-shooting, and to correct for small mechanical or sensor errors in real time.

### Maze Solving — Flood-Fill + A\*-style Optimization
1. **Exploration run:** The robot performs an initial run through the maze using a **Flood-Fill algorithm** — each cell is assigned a "distance to goal" value, updated as new walls are discovered, and the robot always moves toward the neighboring cell with the lowest flood value.
2. **Map building:** As walls are detected, the internal maze map is updated cell-by-cell so the flood values stay accurate.
3. **Path optimization:** Once the goal is reached (and optionally after multiple exploration passes), the firmware computes an optimized path using **A\*-style search** over the known maze graph, factoring in:
   - Shortest total cell count
   - **Diagonal movement** where two adjacent cells are simultaneously open, cutting corners instead of doing two 90° turns
4. **Speed run:** With the optimized route computed, the robot replays it directly and quickly, using the IMU + encoder feedback to keep the diagonal/turn execution smooth and precise.

### Mode Selection & Tuning
On-board tactile buttons let you switch between modes (e.g., exploration mode, speed-run mode, calibration mode) and adjust tuning parameters (like PID gains or sensor thresholds) without needing to reflash firmware — useful for on-the-fly adjustments between maze runs.

---

## 4. Key Features

- 🧠 **Flood-Fill maze-solving algorithm** for exploration, with **A\*-style optimized path planning** for the final solved run
- ↗️ **Diagonal movement support** for shorter, faster optimized paths
- 🎛️ **On-board tactile buttons** for mode selection and live parameter tuning
- 👁️ **IR/ToF (VL53L0X) sensors** for real-time wall detection
- 🔄 **Quadrature encoders** on all 4 N20 motors for precise distance/speed tracking
- 🧭 **MPU6050 IMU** for smooth, accurate, drift-corrected turning
- 📐 **Kalman filter sensor fusion** combining gyro + accelerometer data for reliable heading
- 🧩 **Modular PCB stack design** — motor driver, MCU, and sensor interconnect boards can be built/tested independently
- 🔋 **Dual 2S LiPo power** for extended runtime
- 💡 **Status LEDs** for mode/debug feedback

---

## 5. Build Status

| Stack | Status |
|---|---|
| Black PCB stack (MCU + motor driver + sensor interconnect) | ✅ Finished, assembled, fully functional |
| Purple PCB stack (auxiliary sensor/expansion boards) | 🚧 Fabricated but not fully populated/wired — work in progress |

---

## 6. Future Work

- Finish populating and wiring the purple expansion boards
- Integrate the secondary switch bank / auxiliary IC for extended tuning options
- Tune Kalman filter and PID gains further for faster diagonal speed-runs
- Add logging/telemetry over the STM32's USB for post-run maze map visualization
