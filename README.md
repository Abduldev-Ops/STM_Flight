# STM32 FreeRTOS Flight Controller

A quadcopter flight controller built from scratch on an STM32F405 running FreeRTOS.
No off-the-shelf flight stack (no Betaflight or ArduPilot): every layer, from the
sensor drivers to the motor output, is written in C.

<!-- Add a photo of the drone here: ![Drone](docs/drone.jpg) -->

**Demo video:** [add link]
**Status:** [e.g. "Bench-tested with props on a tether; flight controller board being rebuilt after a power fault"]

> Originally developed on a Nucleo-L452RE (see tag [`l452-nucleo`](../../tree/l452-nucleo)),
> then ported to the STM32F405 for its 168 MHz timers and extra DMA headroom.

---

## What it does

- Reads the **MPU6050** IMU at a fixed **100 Hz** over I2C (400 kHz), with deadline-based
  scheduling and real measured dt
- Fuses gyro and accelerometer with an **Extended Kalman Filter** that estimates gyro bias,
  and gates the accelerometer when vibration makes it unreliable
- Runs a **cascaded PID** controller (angle loop → rate loop) with a desaturating motor mixer
- Drives four BLHeli_S ESCs with **OneShot125** from a dedicated 1 kHz output task
  (a DShot300 driver with DMA and CRC is also included; see [the DShot story](#the-dshot-story))
- Takes pilot input from a **PS5 DualSense** over Bluetooth via an ESP32 (bluepad32), using a
  checksummed binary UART protocol with a **500 ms RC failsafe**
- Streams a binary telemetry packet at **20 Hz** to the ESP32, which forwards it over WiFi UDP
  to a **Python ground station** (artificial horizon, altitude graph, motor bars, link status)
- Reads altitude from a **BMP280** and position from a **GY-NEO6** GPS (interrupt-driven NMEA parsing)
- **IWDG watchdog**, stack-overflow detection, a per-task stack monitor, and a crash handler
  that cuts the motors first

---

## System architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     STM32F405 (168 MHz)                       │
│                                                              │
│  task_sensor ──imu_queue──► task_filter ──state_queue──┐     │
│  (MPU6050, 100 Hz)          (EKF)                      │     │
│                                                        ▼     │
│  task_rc ─────rc_queue──────────────────────────► task_control
│  (USART3 ← ESP32)                                (PID + mixer)
│                                                        │     │
│                                         DSHOT_SetMotors()    │
│                                                        ▼     │
│                                     task_dshot (1 kHz, realtime)
│                                     TIM1 CH1–4 → OneShot125   │
│                                                              │
│  task_baro  (BMP280)  ─┐                                     │
│  task_gps   (USART2)  ─┼──► task_display (OLED, 5 Hz)        │
│                        └──► task_telemetry (20 Hz) ──USART3──► ESP32
│  task_loggr (5 Hz) ── all console output, so control never printf()s
│  task_watchdog (IWDG) · task_stackmon (stack high-water marks)│
└──────────────────────────────────────────────────────────────┘
                                                     │
                                         ESP32: Bluetooth (PS5) + WiFi UDP
                                                     │
                                          Python ground station
```

---

## Hardware

| Component | Interface | Pins |
|---|---|---|
| STM32F405RGT6 | — | — |
| MPU6050 IMU | I2C1, 400 kHz | PB6 / PB7 |
| BMP280 barometer | I2C1 | PB6 / PB7 |
| SSD1306 OLED | I2C1 | PB6 / PB7 |
| GY-NEO6 GPS | USART2, 9600 baud | PA2 / PA3 |
| ESP32 (RC in, telemetry out, console) | USART3, 115200 baud | PC10 / PC11 |
| 4× BLHeli_S 30A ESC | TIM1 CH1–4 | PA8 – PA11 |
| Power | Matek PDB, 3S LiPo (XT60) | — |
| Motors / props | 2300 KV, 5030 | — |

---

## FreeRTOS task map

| Task | Rate | Priority | Role |
|---|---|---|---|
| task_dshot | 1 kHz | Realtime | Writes the latest motor commands to TIM1 |
| task_watchdog | 2 Hz | High | Kicks the IWDG |
| task_sensor | 100 Hz | AboveNormal | MPU6050 read → imu_queue |
| task_filter | per sample | AboveNormal | EKF → state_queue |
| task_control | per state | AboveNormal | Arming, cascaded PID, mixer |
| task_rc | event | AboveNormal | RC packets from ESP32, failsafe |
| task_baro | ~10 Hz | Normal | BMP280 altitude |
| task_gps | event | Normal | NMEA parsing |
| task_display | 5 Hz | Normal | OLED |
| task_telemetry | 20 Hz | BelowNormal | Binary packet to ESP32 |
| task_loggr | 5 Hz | Low | Console output |
| task_stackmon | — | Low | Stack/heap margins |

---

## Results

Measured on the bench, from the firmware's own instrumentation:

| Metric | Result |
|---|---|
| Control-loop rate | 100 Hz, dt **9.92–10.08 ms** (±0.1 ms, ~1% jitter) with every task running |
| IMU reads | 100 per second, **0 failures** |
| Telemetry link | 20 Hz, **0 corrupted packets** across 1,500+ |
| DShot300 frames | **Bit-perfect** at the MCU pin (DWT cycle-count capture) |
| Motor output | All four motors controlled via OneShot125 |

dt is measured with DWT cycle-counter timestamps on each IMU sample, and the PID and EKF use
the measured value rather than assuming 10 ms.

---

## Design decisions

**Why FreeRTOS over a superloop?**
Each sensor and output has its own timing: the IMU at 100 Hz, the barometer at about 10 Hz,
GPS and RC whenever bytes arrive, motor output at 1 kHz. A superloop would have to
oversample the slow sensors or starve the fast ones. Priority-based tasks give each job the
CPU time it needs, and queues and mutexes make the data hand-offs explicit.

**Why an EKF over a complementary filter?**
A complementary filter trusts the gyro and accelerometer by a fixed ratio, so it gets fooled
when motor vibration corrupts the accelerometer. The EKF estimates gyro bias as part of its
state, so drift is corrected without manual calibration. On top of that, accelerometer
updates are gated: full trust when |a| is within 1 ± 0.10 g, a linear ramp down to 0.30 g,
and gyro-only beyond that.

**Why cascaded PID?**
A single loop from angle error to motor output reacts too slowly to fast disturbances. The
inner rate loop works on gyro rate and corrects rotation before it becomes an angle error;
the outer angle loop provides level-hold accuracy. Splitting them also makes tuning tractable:
tune the rate loop first, then add angle control on top.

**Why a separate 1 kHz motor-output task?**
task_control only stores the latest motor commands. A separate realtime-priority task writes
them to the timer at a fixed 1 kHz, so motor output timing never depends on how long the
control math or logging takes.

**Why a dedicated logger task?**
printf with floats is slow and stack-hungry. Moving all console output into a low-priority
task means task_control never blocks on the UART, which also freed about 400 bytes of
control-task stack.

**Why interrupt-driven UART for GPS and RC?**
Blocking receive holds the CPU for the full timeout. At 9600 baud that's about 1 ms per
byte, enough to disturb 100 Hz sensor timing. Interrupt-driven receive costs nothing between
bytes, and completed buffers are handed to the task whole, so it never reads a half-written one.

**Why checksums and failsafes on the RC link?**
The ESP32-to-STM32 link carries arming and throttle. Corrupted packets are dropped, the STM32
disarms if no valid packet arrives for 500 ms, and the ESP32 separately disarms if the
controller data it is forwarding goes stale.

---

## The DShot story

The original plan was DShot300: fully digital, no ESC calibration, CRC on every frame. I wrote
the driver (TIM1 + four DMA streams, packet generation, CRC) and verified with DWT cycle-count
capture that the frames leaving the pin were bit-perfect, at both DShot300 and DShot150.

The ESCs still rejected every non-zero frame (they accepted all-zero "stop" frames), while
analog PWM spun all four motors on the same pins. Without a logic analyzer I couldn't see what
arrived at the ESC's input pad, so I moved motor output to **OneShot125**: same timer, same
pins, no DMA, and no change to the flight code, since task_control still calls the same
`DSHOT_SetMotors()` interface. The DShot driver is kept in the code for when I can capture the
signal at the ESC end.

---

## Debugging log

Hardware problems I hit and how they were isolated:

- **Motors arming but not spinning.** Ruled out the motor windings and ESC output stages
  (BLHeli_S beep tones through the windings proved both worked), then found a Li-ion pack
  browning out under motor-start inrush, plus one ESC with Startup Power at half the default.
  Fixed with a LiPo and corrected ESC settings via BLHeliSuite.
- **DShot rejected by the ESCs.** See above.
- **Motors shaking and heating on the bench without props.** The PID was reacting to an error
  it had no thrust to correct; also found an uncorrected yaw gyro bias and a D-term filter
  cutoff too high for a 100 Hz loop.
- **Drone kept flying on frozen sticks during a props-on test.** The ESP32's Bluetooth link
  stalled under motor noise while still forwarding the last packet. Added an ESP32-side
  disarm on stale controller data within 250 ms.
- **Dead flight controller.** A PDB ground wire came loose while powered; measured 0.2 V on the
  3.3 V rail. The rebuild adds soldered grounds, series resistors on signal lines, decoupling
  capacitors and a pull-cord kill switch.

---

## Building and flashing

**Requirements:** STM32CubeIDE 1.19+, ST-Link.

1. Clone this repo.
2. STM32CubeIDE → File → Open Projects from File System → select the repo folder
   (the project appears as `drone_fin`).
3. Project → Build All, then Run → Debug to flash.

**ESP32 (RC + telemetry bridge):**
1. Install the Arduino IDE with bluepad32 board support.
2. Copy `esp32/secrets_example.h` to `esp32/secrets.h` and fill in your WiFi details.
3. Open `esp32/ps_rc_to_stm.ino` and flash it to the ESP32.

**Ground station:**
```bash
pip install matplotlib
python esp32/ground_station/ground_gui.py
```

---

## Repository structure

```
Core/
  Src/
    task_sensor.c     MPU6050 read, 100 Hz
    task_filter.c     EKF + accelerometer gating
    task_control.c    arming, cascaded PID, mixer
    task_dshot.c      1 kHz motor output
    task_rc.c         RC packets + failsafe
    task_telemetry.c  20 Hz binary telemetry
    task_baro.c       BMP280
    task_gps.c        GPS
    task_display.c    OLED
    task_loggr.c      console output
    task_watchdog.c   IWDG
    flight/  ekf.c, pid.c, dshot.c (DShot300 + OneShot125)
    drivers/ mpu6050.c, bmp280.c, gps.c
esp32/
  ps_rc_to_stm.ino    bluepad32 RC + WiFi UDP bridge
  secrets_example.h
  ground_station/     ground_gui.py, ground_station.py
```

---

## What's next

- Capture the DShot signal at the ESC with a logic analyzer and switch back to DShot300
- Finish the board rebuild and first untethered hover
- PMW3901 optical flow for GPS-denied position hold