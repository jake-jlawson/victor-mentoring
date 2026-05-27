# Robotic Arm Simulation - Operation Guide

## Run Setup

Run from the `robotic-arm` folder:

```powershell
cd "c:\Users\jakej\Files\Tutoring\OA Mentoring\Victor\victor-mentoring\robotic-arm"
.\.venv\Scripts\python.exe main.py
```

If your virtual environment is already activated, you can use:

```powershell
python main.py
```

## Command Template

```powershell
.\.venv\Scripts\python.exe main.py `
  --controller <open_loop|pid|pid_gravity|feedforward> `
  --links <int>=1 `
  --reference-mode <static|point_to_point> `
  --target-angle <rad> `
  --goal-angle <rad> `
  --move-start <s> `
  --move-duration <s> `
  --noise-angle-std <rad> `
  --noise-velocity-std <rad/s> `
  --duration <s>
```

### 1) Baseline static setpoint (default)

```powershell
.\.venv\Scripts\python.exe main.py
```

### 2) Static setpoint at a custom angle

```powershell
.\.venv\Scripts\python.exe main.py --controller pid --reference-mode static --target-angle 0.50
```

### 3) Smooth point-to-point move (realistic non-oscillatory reference)

```powershell
.\.venv\Scripts\python.exe main.py --controller pid --reference-mode point_to_point --target-angle 0.20 --goal-angle 0.90 --move-start 0.5 --move-duration 2.0
```

### 4) Slower, gentler robot move

```powershell
.\.venv\Scripts\python.exe main.py --controller pid --reference-mode point_to_point --target-angle 0.10 --goal-angle 0.70 --move-start 1.0 --move-duration 4.0
```

### 5) Open-loop torque-only behavior (no feedback correction)

```powershell
.\.venv\Scripts\python.exe main.py --controller open_loop --reference-mode static --duration 12
```

Note: `open_loop` ignores reference tracking and applies its own torque profile.

### 6) PID + gravity compensation

```powershell
.\.venv\Scripts\python.exe main.py --controller pid_gravity --reference-mode point_to_point --target-angle 0.20 --goal-angle 1.00 --move-duration 2.0
```

### 7) Feedforward-dominant control

```powershell
.\.venv\Scripts\python.exe main.py --controller feedforward --reference-mode point_to_point --target-angle 0.20 --goal-angle 0.90
```

### 8) Sensor-noise robustness test (low noise)

```powershell
.\.venv\Scripts\python.exe main.py --controller pid --reference-mode static --target-angle 0.45 --noise-angle-std 0.01 --noise-velocity-std 0.02
```

### 9) Sensor-noise robustness test (higher noise)

```powershell
.\.venv\Scripts\python.exe main.py --controller pid --reference-mode static --target-angle 0.45 --noise-angle-std 0.03 --noise-velocity-std 0.08
```

### 10) Multi-link arm (3 links) with smooth move

```powershell
.\.venv\Scripts\python.exe main.py --links 3 --controller pid_gravity --reference-mode point_to_point --target-angle 0.20 --goal-angle 0.80 --move-duration 2.5
```

### 11) Fixed-length run for comparison videos

```powershell
.\.venv\Scripts\python.exe main.py --controller pid --reference-mode point_to_point --target-angle 0.20 --goal-angle 0.90 --duration 15
```

## Practical Comparison Sequence (Classroom)

1. Start with static PID setpoint (clear target regulation).
2. Switch to open-loop (show no error correction).
3. Return to PID and add sensor noise.
4. Compare PID vs PID+gravity on point-to-point moves.
5. Increase links from 1 to 2 or 3.

## Option Meanings

- `--controller`: control strategy.
- `--links`: number of arm links/joints.
- `--reference-mode static`: hold one target angle.
- `--reference-mode point_to_point`: one smooth move from start to goal.
- `--target-angle`: static target, and point-to-point start angle.
- `--goal-angle`: final angle in point-to-point mode.
- `--move-start`: when the move begins.
- `--move-duration`: move time; larger means slower motion.
- `--noise-angle-std`: measurement noise on angle.
- `--noise-velocity-std`: measurement noise on angular velocity.
- `--duration`: optional simulation stop time.

## Troubleshooting

- If the animation lags, reduce `--links`, use a simpler controller, or close other heavy apps.
- If response is too aggressive, reduce target angles or use longer `--move-duration`.
- If tracking is noisy, reduce noise levels or use `pid_gravity`.
