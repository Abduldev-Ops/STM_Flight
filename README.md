# STM32 FreeRTOS Flight Controller

A custom drone flight controller built from scratch on STM32 with FreeRTOS.
No off-the-shelf flight controller software — every layer from sensor driver
to motor protocol is written in C.

**Demo video:** [link coming soon]

---

## What it does

- Reads IMU data from MPU6050 at 100Hz over I2C
- Fuses accelerometer and gyroscope using an **Extended Kalman Filter**
  with automatic gyro bias correction
- Runs a **cascaded PID controller** (angle loop → rate loop) to compute
  stabilization commands
- Encodes motor commands as **DSHOT300** packets with CRC, transferred
  via TIM + DMA — no analog PWM
- Receives pilot input from a **PS5 DualSense** over Bluetooth via ESP32
  (bluepad32) with XOR-checksummed binary UART protocol and 500ms failsafe
- Broadcasts live flight state over **WiFi UDP** to a Python ground station
- Displays attitude, altitude, and GPS status on an **SSD1306 OLED**
- Reads barometric altitude from **BMP280** and GPS position from
  **GY-NEO6MV2** (NMEA parsing)
- Arms/disarms via PS5 cross button or B1 hold with throttle-zero safety check
- **IWDG watchdog** resets the system if any task deadlocks

---

## System architecture

```
┌─────────────────────────────────────────────┐
│           STM32 Nucleo-L452RE                │
│                                             │
│  task_sensor  ──imu_queue──►  task_filter   │
│  (MPU6050)                    (EKF)         │
│                                    │        │
│                             state_queue     │
│                                    │        │
│  task_rc  ──rc_queue──────►  task_control  │
│  (LPUART1←ESP32)              (PID + mix)  │
│                                    │        │
│                              DSHOT DMA      │
│                             (TIM2 CH1-4)    │
│                                             │
│  task_baro  ──baro_queue──►  task_display  │
│  (BMP280)                    (OLED + telem) │
│                                             │
│  task_gps   ──gps_queue──►  task_display  │
│  (USART1)                                   │
│                                             │
│  task_telemetry  ──LPUART1──►  ESP32       │
│  task_watchdog   (IWDG kick)                │
│  task_log        (button + arming)          │
└─────────────────────────────────────────────┘
         │                        │
    DSHOT300                 WiFi UDP
    4x ESC                       │
                         Python ground station
                         (artificial horizon,
                          altitude graph,
                          motor bar chart)
```

---

## Hardware

| Component | Interface | Address/Pin |
|-----------|-----------|-------------|
| STM32 Nucleo-L452RE | — | — |
| MPU6050 IMU | I2C1 | 0x68, PB8/PB9 |
| BMP280 Barometer | I2C1 | 0x76, PB8/PB9 |
| SSD1306 OLED | I2C1 | 0x3C, PB8/PB9 |
| GY-NEO6MV2 GPS | USART1 9600 | PA10 RX |
| ESP32 (RC + telemetry) | LPUART1 115200 | PC0 RX, PC1 TX |
| DSHOT motors (×4) | TIM2 DMA | PA0, PA1, PB10, PB11 |
| B1 button | EXTI PC13 | falling edge |

---

## FreeRTOS task map

| Task | Rate | Priority | Communicates via |
|------|------|----------|-----------------|
| task_sensor | 100Hz | AboveNormal | → imu_queue |
| task_filter | 100Hz | AboveNormal | imu_queue → state_queue |
| task_control | 100Hz | AboveNormal | state_queue + rc_queue → DSHOT + telemetry_queue |
| task_rc | event | AboveNormal | LPUART1 → rc_queue |
| task_telemetry | 10Hz | Normal | telemetry_queue → LPUART1 |
| task_baro | 10Hz | Normal | → baro_queue |
| task_gps | event | Normal | USART1 → gps_queue |
| task_display | 100Hz | Normal | state_queue + baro_queue + gps_queue → OLED |
| task_log | event | Normal | button_sem → armed/logging |
| task_watchdog | 2Hz | High | IWDG kick |

---

## Design decisions

**Why FreeRTOS over a superloop?**
Each sensor and output has independent timing requirements — IMU at 100Hz,
barometer at 10Hz, GPS event-driven. A superloop can't satisfy all of these
without either oversampling slow sensors or undersampling fast ones. FreeRTOS
tasks with priority-based scheduling give each concern exactly the CPU time
it needs, with the scheduler handling preemption transparently.

**Why EKF over a complementary filter?**
A complementary filter uses a fixed trust weight (alpha) between gyro and
accelerometer. This doesn't adapt when the accelerometer is corrupted by
motor vibration during flight. The EKF estimates gyro bias as part of its
state vector — automatically correcting for drift without manual calibration.
In practice this means the attitude estimate stays accurate during aggressive
maneuvers where a complementary filter would drift.

**Why cascaded PID (angle + rate loops)?**
A single PID loop from angle error to motor output lacks the bandwidth to
reject fast disturbances. The rate loop (inner) runs at 1000Hz on raw gyro
data and rejects rapid angular disturbances before they become large angle
errors. The angle loop (outer) runs at 500Hz and provides the steady-state
accuracy needed for stable hover. The separation also makes tuning tractable
— you can tune the rate loop independently before introducing angle control.

**Why DSHOT over analog PWM?**
Analog PWM requires per-ESC calibration, is susceptible to noise, and has
~1000 steps of resolution. DSHOT is fully digital — no calibration, 2048
throttle steps, built-in CRC for error detection. Implementation uses TIM2
PWM + DMA burst so the CPU is not involved during transmission.

**Why interrupt-driven UART for GPS and RC?**
HAL_UART_Receive in blocking mode holds the CPU for the full timeout on every
byte. At 9600 baud that's ~1ms per byte — enough to corrupt 100Hz sensor
timing. Interrupt-driven receive fires an ISR per byte and re-arms
automatically, consuming zero CPU between bytes. The task only wakes when a
complete line or packet is ready.

**Why XOR checksum on the RC link?**
The ESP32-to-STM32 UART link carries safety-critical arming and throttle
data. Bit errors on this link could cause unintended arming or incorrect
motor commands. XOR checksum over the full payload detects any single-bit
error and most multi-bit errors — sufficient for a 115200 baud UART link
at flight distances. Corrupted packets are silently dropped and the 500ms
failsafe disarms the system if valid packets stop arriving.

---

## Building and flashing

**Requirements:**
- STM32CubeIDE 1.19.0
- arm-none-eabi-gcc 13.3.1 (bundled with CubeIDE)
- ST-Link (built into Nucleo board)

**Steps:**
1. Clone this repo
2. Open STM32CubeIDE → File → Open Projects from File System → select repo folder
3. Build: Project → Build All
4. Flash: Run → Debug (or Run → Run)

**ESP32 (RC + telemetry):**
1. Install Arduino IDE with bluepad32 board support
2. Open `esp32/ps_rc_to_stm/ps_rc_to_stm.ino`
3. Fill in your WiFi credentials
4. Flash to ESP32

**Ground station:**
```bash
pip install matplotlib
python ground_station/ground_station_gui.py --port 5005
```

---

## Repository structure

```
Core/
  Src/
    task_sensor.c      — MPU6050 I2C driver + task
    task_filter.c      — Extended Kalman Filter task
    task_control.c     — Cascaded PID + motor mixing + DSHOT
    task_rc.c          — PS5 controller UART receive + failsafe
    task_telemetry.c   — Binary telemetry packet to ESP32
    task_baro.c        — BMP280 barometer task
    task_gps.c         — GPS NMEA parsing task
    task_display.c     — OLED display task
    task_log.c         — Arming state machine + logging
    task_watchdog.c    — IWDG watchdog task
    flight/
      ekf.c            — EKF predict + update (pure math)
      pid.c            — PID controller (pure math)
      dshot.c          — DSHOT300 packet gen + DMA buffer
    drivers/
      mpu6050.c        — MPU6050 I2C register reads
      bmp280.c         — BMP280 compensation math
      gps.c            — NMEA GGA parser
esp32/
  ps_rc_to_stm/        — Arduino sketch (bluepad32 + WiFi UDP)
ground_station/
  ground_station_gui.py — Python GUI (matplotlib)
```

---

## What's next

- Port to STM32F405 Blackpill (168MHz, proper DSHOT timer peripherals)
- PMW3901 optical flow for GPS-denied indoor navigation
- Precision landing with AprilTag marker detection