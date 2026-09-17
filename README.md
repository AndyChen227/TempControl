# Temperature Control

This is my solution to the SARP-UW Temperature Control (Hard) software problem.
The goal is to keep the plant at 40 C using only its measured temperature and a
control signal that can heat or cool the box.

## Files

- `Controller.py` - my PID controller
- `Plant_Box.py` - the provided simulation model, unchanged
- `Temperature_Control___Hard.pdf` - the original problem statement

## Controller

```python
integral = 0.0
last_error = None


def controller(curr_temp: float, set_point: float) -> float:
    global integral, last_error

    Kp = 1.0
    Ki = 0.05
    Kd = 5.0
    I_MAX = 130.0

    error = set_point - curr_temp

    integral += error
    integral = max(-I_MAX, min(I_MAX, integral))

    if last_error is None:
        derivative = 0.0
    else:
        derivative = error - last_error
    last_error = error

    return Kp * error + Ki * integral + Kd * derivative
```

The provided loop runs once per second, so the controller uses a one-second
time step. The three terms have different jobs:

- **P** reacts to the current error.
- **I** remembers past error and supplies the continuing power needed to
  replace heat lost to the room.
- **D** reacts to how quickly the error changes and reduces power when the
  temperature approaches the set point quickly.

## Why these gains?

The gains were chosen one at a time in simulation, rather than guessed all at
once.

### `Kp = 1.0`

With an initial error of 20 C, `Kp = 1` produces a proportional output of 20.
It heats the plant at a reasonable rate without the large overshoot produced by
larger proportional gains. P control alone cannot reach 40 C because its output
approaches zero while the box is still losing heat.

### `Ki = 0.05`

Increasing `Ki` removes the steady-state error faster, but also increases
integral windup and overshoot. With `Kp = 1` and no D term, representative
tests gave:

| Ki | Overshoot | Settling time |
|---:|---:|---:|
| 0.01 | 0.09 C | 319 s |
| 0.02 | 0.77 C | 123 s |
| 0.05 | 4.91 C | 62 s |
| 0.10 | 10.12 C | 118 s |

`Ki = 0.05` builds the required steady control effort quickly. Its windup is
then controlled with the integral limit.

### Why the steady control signal is `6.25`

At steady state, both the plant temperature and heater temperature stop
changing. Let `H` be the heater temperature and `u` be the control signal.
The room temperature is 20 C and the target plant temperature is 40 C.

From the plant equation:

```text
0 = 0.08(H - 40) + 0.02(20 - 40)
H = 45 C
```

The heater must therefore remain at 45 C. From the heater equation:

```text
0 = 0.20u - 0.05(45 - 20)
u = 6.25
```

Therefore the controller needs an average output of about `6.25` to hold the
plant at 40 C. This is a control-signal value, not a value in watts.

Near steady state, P and D are approximately zero, so the I term supplies this
output:

```text
Ki * integral = 6.25
0.05 * integral = 6.25
integral = 125
```

The integral is not manually set to 125. It starts at zero and naturally moves
toward this value as errors accumulate. `I_MAX = 130` is slightly above 125,
so it allows enough steady output while preventing excessive windup.

### `Kd = 5.0`

After adding the integral limit, different D gains gave:

| Kd | Overshoot | Settling time |
|---:|---:|---:|
| 0 | 3.03 C | 62 s |
| 2 | 1.59 C | 62 s |
| 5 | 0.41 C | 24 s |
| 10 | 0.24 C | 41 s |

`Kd = 5` gave the best balance: less than 0.5 C overshoot and the shortest
settling time. A larger D gain reduced overshoot slightly, but slowed settling
and increased sensitivity to sensor noise.

## Result

For the fixed test seed used during tuning, the final controller reached the
40 C target with about 0.41 C overshoot, settled within +/-0.5 C in about
25 seconds, and had about 0.02 C steady-state error.

## Run

```bash
pip install matplotlib
python Controller.py
```
