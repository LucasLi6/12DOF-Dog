# 12DOF Robot Dog

Firmware for a 12-degree-of-freedom quadruped robot dog, driven by an
STM32F103C8T6 and controlled wirelessly from a phone. The robot uses 12 MG996R
servos (3 per leg), an HC-05 Bluetooth module for the remote link, a 128×64 OLED
for facial expressions, and an MPU6050 IMU for attitude sensing.

> **Language note:** this codebase contains Chinese comments and inline notes.
> The README below documents the hardware and the feature set.

---

## Features

- **Inverse-kinematics walking gaits** — forward/backward, turning in place,
  left/right strafe, four diagonal gaits, and marching in place.
- **Posture control** — stand, sit, lie down, and neutral standing pose.
- **Special actions** — dance, swing, test, and a "pee" pose.
- **Live height adjustment** — raise/lower the body from the phone.
- **Facial expressions** — the OLED shows different faces per action mode.
- **Wireless control** — phone control through the **WeXCube** protocol over an
  HC-05 Bluetooth link (USART3).
- **IMU attitude sensing** — MPU6050 driver with pitch/roll computation.
- **Safety handling** — servo-angle limits and an error handler that parks all
  12 servos at their neutral position.

### Planned / in progress

- **Standing self-balancing** — PID gains (`P` / `I` / `D`) are already received
  from the phone mini-program (control IDs 22–24) but are not yet wired into a
  closed-loop balance controller. The plan is documented in
  [`main自平衡计划.txt`](main自平衡计划.txt). The MPU6050 pitch/roll convention is:
  `pitch > 0` when the head tips down, `roll > 0` when the body leans right.

---

## Hardware

| Component               | Qty | Purpose                       |
| ----------------------- | --- | ----------------------------- |
| STM32F103C8T6 (Blue Pill)| 1  | Main controller               |
| MG996R servo            | 12  | Leg joints (3 per leg)        |
| HC-05 Bluetooth module  | 1   | Wireless communication        |
| OLED 128×64 (I2C)       | 1   | Facial expressions & status   |
| MPU6050                 | 1   | IMU — pitch/roll              |
| 7.4 V Li-ion battery    | 1   | Main power                    |
| Servo driver board      | 1   | Servo power distribution      |

## Pinout

| STM32 pin | Function      | Connected to               |
| --------- | ------------- | -------------------------- |
| PA0–PA7   | PWM output    | Servos 1–8                 |
| PB0–PB3   | PWM output    | Servos 9–12                |
| PA9       | USART1_TX     | Voice module (optional)    |
| PA10      | USART1_RX     | Voice module (optional)    |
| PB10      | USART3_TX     | HC-05 RX                   |
| PB11      | USART3_RX     | HC-05 TX                   |
| PB6       | I2C1_SCL      | OLED SCL                   |
| PB7       | I2C1_SDA      | OLED SDA                   |
| VCC (3.3 V) | Power       | Bluetooth, OLED            |
| GND       | Ground        | All modules                |

### Servo numbering

| Servo | Leg     | Joint |
| ----- | ------- | ----- |
| 1–3   | Front-left  | hip / knee / ankle |
| 4–6   | Rear-left   | hip / knee / ankle |
| 7–9   | Front-right | hip / knee / ankle |
| 10–12 | Rear-right  | hip / knee / ankle |

---

## Repository layout

```
├── User/          # Application: main.c, interrupt handlers, config
├── HardWare/      # Peripheral drivers: Servo, PWM, OLED, MPU6050,
│                  # Bluetooth, Face, PetAction, RobotState, error handler
├── Library/       # STM32F10x Standard Peripheral Library
├── Start/         # Startup files, system_stm32f10x, core_cm3
├── wexcube_sdk/   # WeXCube (WeChat mini-program) communication protocol
├── Project.uvprojx  # Keil MDK project file
└── DebugConfig/, Listings/, Objects/  # Build output (git-ignored)
```

## Building

1. Open `Project.uvprojx` in **Keil MDK (uVision 5)**.
2. Select the `STM32F103C8T6` target and compile.
3. Flash via ST-Link or serial (USART1).

The project uses the **STM32F10x Standard Peripheral Library** (no HAL/CubeMX).

---

## Control (WeXCube mini-program)

The robot is controlled from the WeXCube WeChat mini-program. Commands arrive as
control events/values over the HC-05 link and are mapped to actions in
[`User/main.c`](User/main.c).

### Action mapping

| Ctrl ID | Action                | Ctrl ID | Action            |
| ------- | --------------------- | ------- | ----------------- |
| 1       | Lie down              | 14      | Strafe right      |
| 2       | Stop                  | 15      | Strafe left       |
| 3       | Stand                 | 16      | Test              |
| 4       | Sit                   | 17      | Dance             |
| 5       | Walk forward          | 18      | Height +          |
| 6       | Walk backward         | 19      | Height −          |
| 7       | Turn left             | 20      | March in place    |
| 8       | Turn right            | 21      | Pee pose          |
| 9       | Swing                 |         |                   |
| 10      | Left-forward walk     |         |                   |
| 11      | Left-backward walk    |         |                   |
| 12      | Right-forward walk    |         |                   |
| 13      | Right-backward walk   |         |                   |

### Trim / tuning values

| Ctrl ID | Parameter                     |
| ------- | ----------------------------- |
| 22      | P gain (self-balancing)       |
| 23      | I gain (self-balancing)       |
| 24      | D gain (self-balancing)       |
| 25      | Front/back center of gravity  |
| 26      | Left/right center of gravity  |
| 27      | Up/down center of gravity (step height) |

---

## Configuration

Key parameters live in [`HardWare/robot_config.h`](HardWare/robot_config.h):

- `ROBOT_TYPE` — large dog vs. small dog (leg length, height range, initial Y).
- `LEG_LENGTH_MM`, `MIN/MAX_HEIGHT_MM`, `INITIAL_Y_POS` — geometry constants.
- `ACTION_INTERVAL_MS`, `ACTION_TIMEOUT_MS`, `DANCE_STEP_INTERVAL_MS` — timing.
- `SERVO_MIN/MAX/MID_ANGLE` — servo angle limits.

Leg-height safety limits are also enforced in `main.c`
(`MIN_LEG_HEIGHT` / `MAX_LEG_HEIGHT`, currently 140–180).

---

## Safety notes

- Do not exceed **8.4 V** supply voltage.
- Avoid stalling servos for long periods; keep joints clear of obstacles.
- Bluetooth range is about **10 m** line-of-sight.
- Back up the firmware before updating.
