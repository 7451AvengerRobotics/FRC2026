# Mechanism Tuning Guide

This guide covers tuning all position- and velocity-controlled mechanisms on the robot so they respond correctly, avoid oscillation or “screaming,” and meet performance goals. It applies to **Hood**, **Turret**, **Shooter**, **Intake Pivot**, and similar subsystems using CTRE Phoenix 6 (TalonFX / TalonFXS) or Spark Flex.

---

## Table of contents

1. [Concepts](#1-concepts)
2. [Position control (Hood, Turret, Pivots)](#2-position-control-hood-turret-pivots)
3. [Velocity control (Shooter)](#3-velocity-control-shooter)
4. [Motion Magic](#4-motion-magic)
5. [Step-by-step tuning procedures](#5-step-by-step-tuning-procedures)
6. [Troubleshooting](#6-troubleshooting)
7. [Mechanism-specific notes](#7-mechanism-specific-notes)

---

## 1. Concepts

### PID + Feedforward

- **P (Proportional)**
  Output proportional to position (or velocity) error. Higher kP = faster correction but more overshoot and risk of oscillation. Primary gain for “getting there.”

- **I (Integral)**
  Output proportional to accumulated error over time. Removes steady-state error but can cause overshoot and **windup** if the mechanism can’t reach setpoint (e.g. binding or hitting a stop). Often left at 0 until P and feedforward are good.

- **D (Derivative)**
  Output proportional to rate of change of error. Dampens overshoot and can reduce oscillation. Sensitive to **noise** (encoder jitter); too high kD can cause jitter or screaming.

- **kS (Static friction)**
  Constant voltage to overcome static friction so the mechanism starts moving. Helps at very low speeds.

- **kV (Velocity feedforward)**
  Voltage per unit velocity. Lets the controller “drive” at the right speed so P doesn’t have to do all the work. Reduces overshoot and oscillation and makes motion smoother.

- **kA (Acceleration feedforward)**
  Voltage per unit acceleration. Used with Motion Magic / motion profiling to track aggressive trajectories. Optional for many mechanisms.

### Open-loop vs closed-loop

- **Open-loop:** You command percent output or voltage; no feedback. Good for quick smoke tests only.
- **Closed-loop:** Controller uses encoder (or other sensor) to hold a setpoint. All serious tuning is closed-loop (position or velocity).

### Encoder direction and scaling

- **Direction:** If “positive” command makes the mechanism move in the **negative** direction in the real world, the controller will fight itself and often **oscillate or scream**. Fix by inverting the motor or the sensor in config.
- **Scaling:** Position setpoints and readings must be in consistent units (e.g. rotations or radians). Check `SensorToMechanismRatio` / `RotorToSensorRatio` (Phoenix) or equivalent so one revolution of the mechanism equals the expected units.

---

## 2. Position control (Hood, Turret, Pivots)

These mechanisms hold an **angle or position**. Setpoint is in rotations (Phoenix) or radians (converted in code).

### Goals

- Reach setpoint with acceptable **rise time** and **overshoot**.
- No sustained **oscillation** or **screaming** at rest or during motion.
- **Settling** within a small tolerance (e.g. ±1° or ±0.01 rad) for long enough to consider “at setpoint.”

### Gains that matter most

| Gain | Effect | Tuning tip |
|------|--------|------------|
| **kP** | Pulls toward setpoint; main “strength” of correction | Too high → overshoot, oscillation, screaming. Start low (e.g. 0.5–2) and increase until it reaches setpoint without overshooting. |
| **kD** | Dampens rate of change; reduces overshoot and ringing | Add after kP. Too high → jitter/screaming from encoder noise. Use 0 or small values (e.g. 0.01–0.1) unless needed. |
| **kI** | Removes steady-state error | Use 0 initially. If there’s a small persistent error, add a **small** kI (e.g. 0.001) and watch for windup. |
| **kS** | Overcomes static friction | Set so mechanism just starts moving from rest; avoid making it too large. |
| **kV** | Feeds forward for motion; reduces load on P | Tune so that at constant speed, P doesn’t have to work hard. Reduces overshoot and oscillation. |
| **kA** | Used with Motion Magic for acceleration | Optional; tune after cruise/accel and P/FF are good. |

### Order of tuning (position)

1. Set **kI = 0**, **kD = 0**.
2. Set **kS** so the mechanism barely moves from rest when you command a small step.
3. Increase **kP** until it reaches the setpoint; if it **overshoots or oscillates**, reduce kP.
4. Add **kV** so motion is smooth and P doesn’t have to “push” the whole way; re-check for overshoot.
5. If you need less overshoot, add a **small kD**; if the mechanism starts **screaming or jittering**, reduce kD or set to 0.
6. Only then consider a **small kI** if there’s steady-state error and no windup (e.g. no hitting a stop).

---

## 3. Velocity control (Shooter)

The shooter holds a **flywheel speed** (RPM or rad/s). Setpoint is velocity; feedback is from the motor encoder.

### Goals

- Reach target RPM quickly.
- Minimal overshoot and no sustained oscillation.
- Hold speed steady under load (e.g. when a ball is fed).

### Gains

- **kV:** Main feedforward: voltage per unit speed. Dominates at steady state.
- **kS:** Overcome static friction so the wheel starts.
- **kP / kD:** Small closed-loop correction. High kP can cause oscillation; keep modest.

### Tuning order (velocity)

1. **kS:** From rest, find the voltage that barely spins the flywheel.
2. **kV:** At a few steady speeds, measure voltage and speed; kV ≈ voltage / speed (in consistent units). Tune so steady-state error is small.
3. **kP:** Add a small value to correct error; increase until response is fast enough but not oscillatory.
4. **kD:** Optional; can help with overshoot but is often 0 for velocity loops.

---

## 4. Motion Magic

Motion Magic (Phoenix) or similar **motion profiling** limits **velocity** and **acceleration** (and optionally jerk) so the mechanism follows a smooth path instead of stepping directly to the setpoint.

### Parameters

- **Cruise velocity:** Max speed (e.g. rotations per second) during the move.
- **Acceleration:** How quickly it ramps to cruise and back to zero.
- **Jerk (if used):** Rate of change of acceleration; smoother starts/stops.

### When to tune Motion Magic

- **Too aggressive (high cruise/accel):** Mechanism may **slam**, **vibrate**, or **scream**; current spikes; mechanical wear.
- **Too conservative (low cruise/accel):** Mechanism is slow to reach setpoint; may feel “sluggish.”

### Tuning order

1. Set **PID + FF** first with a **step** setpoint (no profiling or gentle profile) so the mechanism is stable and doesn’t oscillate.
2. Enable Motion Magic and set **cruise** and **accel** to **low** values.
3. Increase **cruise** until the mechanism reaches setpoint in acceptable time.
4. Increase **acceleration** until the move is quick but still smooth and quiet.
5. If the mechanism **screams or shakes** during the move, reduce cruise and/or acceleration.

---

## 5. Step-by-step tuning procedures

### A. Position mechanism (e.g. Hood or Turret) from scratch

1. **Verify hardware**
   - Motor and encoder wired; no binding; mechanism moves freely in the allowed range.
   - Confirm **encoder direction**: positive command → positive change in position (or invert motor/sensor to make it so).

2. **Zero gains**
   - Set kP, kI, kD, kS, kV, kA to 0. Command a small setpoint change; nothing should happen (or only drift). Confirms closed-loop is in use.

3. **kS**
   - Increase kS until the mechanism **just** starts moving from rest when you command a small step. Don’t make kS larger than needed.

4. **kP**
   - Set kI = 0, kD = 0. Increase kP until the mechanism moves to the setpoint.
   - If it **overshoots** or **oscillates**, reduce kP until it settles without overshoot (or with acceptable overshoot).

5. **kV**
   - Add kV so that during motion the controller “feeds forward” the right amount. This reduces the work P has to do and often **reduces overshoot and screaming**. Tune by eye or by increasing kV until motion is smooth; back off if it overshoots.

6. **kD (optional)**
   - Add a **small** kD to dampen overshoot. If you get **jitter or screaming**, reduce kD or set to 0 (encoder noise).

7. **kI (optional)**
   - Only if there’s a **steady-state error** (e.g. always 1° short). Add a **very small** kI. Ensure the mechanism can actually reach setpoint; otherwise I will wind up and cause big overshoot and oscillation.

8. **Motion Magic (if used)**
   - Start with low cruise and low acceleration. Increase until the move is fast enough but still smooth and quiet.

### B. Velocity mechanism (e.g. Shooter)

1. **kS:** Voltage that barely spins the wheel from rest.
2. **kV:** From a few steady-state (voltage, speed) points, fit kV ≈ V / ω (or V / RPM in consistent units).
3. **kP:** Small value; increase until speed error is corrected without oscillation.
4. **kD:** Usually 0 unless you need extra damping.

### C. “Mechanism screams” (already tuned, now debugging)

1. **Lower kP** — most common fix for oscillation/screaming.
2. **Lower kD** — D amplifies encoder noise; try 0 or half.
3. **Check direction** — wrong sign → fighting itself → oscillation.
4. **Check for binding or hard stops** — mechanism can’t reach setpoint → integrator windup or continuous high output → screaming.
5. **Add or increase kV** — so P doesn’t have to do all the work; often quiets the system.
6. **Reduce Motion Magic cruise/accel** — less aggressive move can reduce vibration and noise.

---

## 6. Troubleshooting

Use this as a quick reference: **symptom → likely cause → action**.

| Symptom | Likely cause | Action |
|--------|----------------|--------|
| **Screaming / loud whine / vibration at or near setpoint** | kP too high; oscillation | **Lower kP.** Add or increase kV so P doesn’t dominate. |
| **Screaming / jitter** | kD too high; amplifying encoder noise | **Lower kD** or set to 0. |
| **Motor doesn’t move** | Gains all zero; or wrong sensor | Set at least **kP** (and **kS** if friction is high). Check encoder source and scaling. |
| **Moves wrong direction then corrects; oscillates** | Wrong encoder or motor direction | **Invert** motor or sensor so positive command → positive position change. |
| **Overshoots then settles** | kP too high; not enough damping | **Lower kP.** Optionally add **small kD** or increase **kV**. |
| **Never reaches setpoint; drifts** | No integral or feedforward | Add **kV** (and **kS** if needed). Optionally small **kI** if steady-state error remains. |
| **Reaches setpoint then slowly creeps away** | Need small kI or disturbance | Add a **very small kI** or re-check mechanism (friction, gravity). |
| **Huge overshoot when it finally moves** | Integral windup (can’t reach setpoint) | **Set kI = 0** until mechanism can reach setpoint. Fix binding/limits; then re-add small kI if needed. |
| **Slams into mechanical stop; screams** | Setpoint beyond soft/hard limit | Add **soft limits** in code; don’t command past mechanical range. |
| **Slow to reach setpoint** | Low kP; or Motion Magic too conservative | Increase **kP** (without overshoot) and/or **cruise velocity** and **acceleration**. |
| **Shooter speed oscillates** | kP too high in velocity loop | **Lower kP**; rely more on **kV** for steady state. |

---

## 7. Mechanism-specific notes

### Hood (TalonFXS + Minion, encoder from controller)

- **Constants:** `HoodConstants` — kP, kI, kD, kS, kV, kA; Motion Magic cruise/accel; min/max angle; `kEncoderOffsetRotations`, `kEncoderToHoodRadiansPerRotation`, `kHoodGearRatio`.
- **Feedback:** Rotor sensor (no CANcoder). `SensorToMechanismRatio` = motor rotations per hood rotation.
- **Tuning:** Position control + Motion Magic. Follow [Section 5A](#a-position-mechanism-eg-hood-or-turret-from-scratch) and [Section 4](#4-motion-magic). If it screams, lower kP and/or Motion Magic aggressiveness.

### Turret (TalonFXS + Minion)

- **Constants:** `TurretConstants` — kP, kI, kD, kS, kV, kA. Encoder range [0, 10] = one revolution in code.
- **Tuning:** Position control. Same as Hood: start with kP and kV; add kD only if needed and keep small to avoid noise.

### Shooter (Spark Flex)

- **Constants:** `ShooterConstants` — kP, kD, kS, kV (and fixed RPM / launch velocity for ballistics).
- **Tuning:** Velocity control. kS and kV first; then small kP. See [Section 3](#3-velocity-control-shooter) and [Section 5B](#b-velocity-mechanism-eg-shooter).

### Intake Pivot (TalonFX)

- **Constants:** `IntakePivotConstants` — kP, kI, kD, kS, kV, kA, gear ratio.
- **Tuning:** Position control + Motion Magic. Same procedure as Hood; respect mechanical limits (stow/deployed).

### General

- **Current limits:** Set in config to protect motors and mechanism; don’t rely on tuning to limit current.
- **Neutral mode:** Brake for position mechanisms (hood, pivot) so they hold when disabled; coast for turret/shooter if desired.
- **Logging:** Use AdvantageScope or similar to log setpoint, position/velocity, and output. Essential for diagnosing oscillation and windup.

---

## Summary

- **Position:** Tune kS → kP → kV → (optional) kD → (optional) kI; then Motion Magic if used. **Lower kP** is the first fix for screaming or oscillation.
- **Velocity:** kS → kV → kP; kD usually 0.
- **Always:** Check encoder direction and scaling; ensure no binding or commanding past limits; use logging to verify behavior.

This guide should cover the tuning situations you’ll run into for Hood, Turret, Shooter, and other mechanisms on the robot. For ballistic and aim logic (turret yaw, hood pitch), see `shooting_ballistics.md` and `robot_architecture.md`.
