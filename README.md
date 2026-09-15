# 🐁 Maze-Solver – STM32 Line Follower Robot (LFR)

[![Micromouse-style build](https://github.com/UtkrishtTripathi/MicroMouse/raw/main/render/img1.png)](https://github.com/UtkrishtTripathi/MicroMouse/blob/main/render/img1.png)

A **maze-solving line follower robot** built around an **STM32 microcontroller**, using the classic **wall-following (left/right-hand rule)** algorithm to navigate and escape mazes autonomously.

> This project is under active development — hardware bring-up and firmware for the `lfr` module are in progress.

---

## 🧠 Overview

Maze-Solver is a small autonomous robot that enters an unknown maze and finds its way to the exit by systematically "hugging" a wall as it moves. Unlike flood-fill-based micromice, which map the whole maze before optimizing a route, this robot follows a simpler, low-memory strategy:

- Keep one wall (left **or** right) in constant contact
- Follow every turn and junction that wall presents
- Guaranteed to reach the exit for simply-connected (non-looping) mazes

This makes it a lightweight, low-compute approach that's ideal for getting a maze-solving robot up and running quickly on STM32 hardware.

---

## ⚡ Key Features

- **STM32-based control board** for the sensing and motor-control loop
- **Wall-following navigation** using the left/right-hand rule
- **Line/wall sensing** to detect walls and junctions in real time
- Simple, deterministic logic — easy to debug and tune
- Designed as a foundation that can later be extended toward flood-fill or shortest-path solving

---

## 🔧 How Wall-Following Works

1. Pick a hand rule — say, **right-hand rule**.
2. At every step, the robot tries to turn right first.
3. If there's a wall to the right, it goes straight instead.
4. If there's a wall straight ahead too, it turns left.
5. If walls surround it on three sides, it turns around.

Followed consistently, this traces the boundary of the maze and is mathematically guaranteed to reach the exit — as long as the maze has no isolated loops disconnected from the outer wall.

---

## 🛠️ Hardware (Planned / In Progress)

| Component | Notes |
|---|---|
| MCU | STM32 (Arduino IDE / STM32Cube toolchain) |
| Sensors | IR / wall-detection sensors |
| Motors | Dual-motor differential drive |
| Chassis | `lfr` module — line-follower-robot base |

*(Update this table with your final BOM as the hardware design is finalized.)*

---

## 🖼️ Reference Hardware

The images below are from a related STM32-based maze robot build and are included here as a visual reference while this repo's own hardware photos and PCB renders are added.

**Example control board layout**
[![Reference PCB layout](https://github.com/UtkrishtTripathi/MicroMouse/raw/main/render/img2.jpeg)](https://github.com/UtkrishtTripathi/MicroMouse/blob/main/render/img2.jpeg)

**Example assembled robot**
[![Reference assembled robot](https://github.com/UtkrishtTripathi/MicroMouse/raw/main/render/img5.jpeg)](https://github.com/UtkrishtTripathi/MicroMouse/blob/main/render/img5.jpeg)

> 📌 Replace these with photos/renders of your own `lfr` build once available.

---

## 📁 Project Structure

```
Maze-solver/
└── lfr/        # Line follower robot firmware / hardware files
```

---

## 📊 Project Status

- ✔ Wall-following algorithm selected (left/right-hand rule)
- ✔ STM32 chosen as the control platform
- ▢ Sensor + motor integration in progress
- ▢ Firmware for `lfr` module in progress
- ▢ Hardware photos / PCB renders to be added

---

## 🙏 Acknowledgements

README structure inspired by [UtkrishtTripathi/MicroMouse](https://github.com/UtkrishtTripathi/MicroMouse), an STM32-based micromouse built for the WRC competition.

---

## ⚠️ Disclaimer

This project is for **educational and experimental purposes**. The author assumes no responsibility for damage, malfunction, or misuse. Use at your own risk.
