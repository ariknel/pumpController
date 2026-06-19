# Adaptive Cooling Pump Controller

A temperature-controlled cooling pump controller for 2-stroke and 4-stroke scooter/moped tuning. Automatically adjusts pump speed based on engine temperature, with manual override and fail-safe fallback built in.

Built by **Arik Nel**.

![3D Render](3d-render.PNG)
![3D Render](3d-render-underside.PNG)
<!-- ![Assembled Board](assembled-photo.png)n -->


## Features

- **Wide input range (6–24V)** — runs directly off the scooter/moped battery, with protection against overvoltage above 12V
- **ESD protection** against electrostatic discharge
- **Automotive-grade fuse** (5A minimum) protecting the full wiring harness from a board-level short
- **Temperature sensor input**, with control logic handled entirely in firmware
- **Fully programmable microcontroller** — adjust the temperature curve, thresholds, and behavior in software
- **Backup trimpot** — if the temperature sensor is disconnected, the controller falls back to a fixed speed set on the onboard potentiometer
- **External potentiometer input** — connect a potentiometer mounted on the dashboard for manual speed control/override; takes priority over the onboard trimpot, and if disconnected, the controller falls back to the safe onboard trimpot value
- **Status LED** — blinks at a rate proportional to the current PWM duty cycle, giving an at-a-glance read of pump speed

## Hardware

- **MCU:** ATmega328P — fully programmable, ISP-flashable
- **Buck converter:** AP63200QWU-7, automotive-rated input range
- **Motor driver:** AON6354 N-channel MOSFET, low-side PWM switching
- **Input protection:** TVS diode, reverse-polarity diode, automotive fuse

## Hardware

**Pinout ATmega328P-AU:**
- Pulse LED (PD7
- NTC (PTC0)
- Trimpot (PC1)
- Ext. Pot (PC2)
