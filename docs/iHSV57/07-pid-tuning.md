# PID Tuning

## Three Control Loops

The iHSV57 has three nested control loops, from innermost to outermost:

1. **Current loop** (2 kHz bandwidth) - Controls motor current/torque
2. **Speed loop** (500 Hz bandwidth) - Controls motor velocity
3. **Position loop** (200 Hz bandwidth) - Controls motor position

Each loop has its own PID gains. The inner loops must be tuned before the outer ones.

## Auto-Tuning (V6 Firmware)

The V6 firmware provides automatic gain adjustment via P01-02 and P01-03.

### P01-02: Auto Adjustment Mode

| Value | Mode | Description |
|-------|------|-------------|
| 0 | Manual | All gains set manually |
| 1 | Standard auto | Auto-adjusts: P02-00, P02-01, P02-10, P02-11, P02-13, P02-14, P08-20. User sets P02-03 (speed FF) and P02-04 (FF smoothing). |
| 2 | Positioning auto | Auto-adjusts same as mode 1, plus P08-20. Fixes P02-03=30% and P02-04=0.5ms. |

### P01-03: Rigidity Setting (0-31)

When P01-02 = 1 or 2, P01-03 selects from 32 built-in gain presets:
- **Low values (0-10)**: Low stiffness, soft feel, may have overshoot
- **Mid values (11-20)**: Moderate stiffness, good for most applications
- **High values (21-31)**: High stiffness, fast response, may vibrate

**Default**: 13

**OSSM tip from MikesMachines**: Rigidity has a huge impact on machine feel:
- Low rigidity creates a springy, compliant feel
- Too low and the motor can't maintain the stroke
- High rigidity is stiff and precise
- Too high causes vibration and oscillation
- **Can be changed while running** - potential for dynamic adjustment

### P01-04: Moment of Inertia Ratio (0-100)

Set to: load_inertia / motor_inertia
Can use the value from AF-J-L (automatic inertia identification).
Default: 1 (equal inertia)

## Manual Gain Parameters

### Position Loop (P02-xx)

| Parameter | Name | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| P02-00 | Position gain 1 (Kp) | 0-3000 | 48.0 1/S | Steady-state response. Higher = stiffer, but can vibrate. |
| P02-01 | Position gain 2 (Kp) | 0-3000 | 57.0 1/S | Dynamic response. Used with gain switching. |
| P02-03 | Speed feedforward | 0-100 | 30.0 % | Reduces position tracking error at speed. Too high = overshoot. |
| P02-04 | Speed FF smoothing | 0-64.00 | 0.5 ms | Smooths feedforward. Higher = more filtering but adds phase lag. |

### Speed Loop (P02-xx)

| Parameter | Name | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| P02-10 | Speed gain 1 (Kp) | 1-2000 | 27.0 Hz | Proportional gain for static response. Increase until vibration, then back off. |
| P02-11 | Speed integral 1 (Ti) | 0.1-1000 | 10.0 ms | Integral time constant. Smaller = faster, stiffer. Minimize without oscillation. |
| P02-12 | Pseudo diff FF 1 | 0-100 | 100.0 % | At 100% = pure PI. At 0% = integral only (slow, good for filtering). |
| P02-13 | Speed gain 2 (Kp) | 1-2000 | 27.0 Hz | Dynamic response gain (with gain switching). |
| P02-14 | Speed integral 2 (Ti) | 0.1-1000 | 1000.0 ms | Dynamic integral constant. |
| P02-15 | Pseudo diff FF 2 | 0-100 | 100.0 % | Dynamic differential feedforward. |
| P02-19 | Torque feedforward | 0-30000 | 0 % | Adds current proportional to speed derivative. |
| P02-20 | Torque FF smoothing | 0-64.00 | 0.8 ms | Torque feedforward filter time. |

### Current Loop (Advanced - P02-xx via V5 map)

| Parameter | Name | Description |
|-----------|------|-------------|
| Cp | Current proportional | Higher = faster current response |
| Ci | Current integral | Higher = better steady-state current accuracy |

Typically left at factory defaults unless specifically tuning for unusual loads.

## Gain Switching (P02-30)

The servo can automatically switch between two gain sets (gain 1 and gain 2)
based on conditions. This is useful for:
- Different gains for acceleration vs steady state
- Different gains for loaded vs unloaded
- Changing feel based on position or torque

### P02-30: Gain Switching Mode

| Value | Condition | Description |
|-------|-----------|-------------|
| 0 | Fixed first gain | Always use P02-00/10/11/12 |
| 1 | Fixed second gain | Always use P02-01/13/14/15 |
| 2 | DI input switching | Switch via digital input |
| 3 | Torque command big | Switch to second when torque > threshold |
| 4 | Speed command big | Switch to second when speed change > threshold |
| 5 | Speed command large | Switch to second when speed > threshold |
| 6 | Large position deviation | Switch to second when position error > threshold |
| 7 | Have location command | Switch to second during position command, first after completion |
| 8 | Location incomplete | Switch to second while positioning, first when complete |
| 9 | Real speed is big | Switch based on actual speed vs threshold |
| 10 | Location command + actual speed | Switch based on position command and speed |

Threshold set by P02-31, hysteresis by P02-32, switching delay by P02-33.

## Advanced Parameters (P08-xx)

| Parameter | Name | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| P08-19 | Feedback speed low-pass filter | 0-25.00 ms | 0.8 ms | Reduces noise when motor sounds rough |
| P08-20 | Torque command filter | 0-25.00 ms | 0.84 ms | Reduces squeal during running |
| P08-25 | Disturbance torque compensation | 0-100 % | 0 | Higher = better disturbance rejection, but more noise |
| P08-26 | Disturbance torque filter | 0-25.00 ms | 0.8 ms | Filters disturbance compensation |

## Tuning Procedure (Manual)

1. **Start with auto-tune**: Set P01-02=1, P01-03=13. Run the motor under typical load.
2. **Adjust rigidity**: Increase P01-03 until vibration, then back off 2-3 steps.
3. **If auto-tune isn't enough**, switch to P01-02=0 (manual) and fine-tune:
   a. **Speed loop first**: Increase P02-10 until oscillation, back off 20%. Decrease P02-11 until oscillation, increase 20%.
   b. **Position loop second**: Increase P02-00 until oscillation, back off 20%.
   c. **Feedforward**: Increase P02-03 to reduce tracking error during motion.
4. **Use iHSV-Servo-Tool** to watch real-time position/velocity/torque while tuning.

## Real-World Values (from go-servoc configs)

Sebastian's config (V5 firmware):
```yaml
pos_loop:
  pp: 20000
  pd: 0
  pf: 1400      # feedforward 14%
  pos_filter: 7
  pos_error: 30000

vel_loop:
  vp: 8500
  vi: 3000
  vd: 0
  vel_limit: 18000
  acc: 1
  dec: 500
  vel_filter: 7

current_loop:
  cp: 3000
  ci: 500
  continous_current: 6000
  limit_current: 15000
```

Note: These are V5 register values and may not directly translate to V6 parameter ranges.
