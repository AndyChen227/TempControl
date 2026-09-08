"""
Offline simulation harness for the Plant_Box temperature control problem.

The provided main loop in Controller.py runs in real time (time.sleep(1) per
step), which makes tuning a controller impractically slow.  This harness drives
the exact same Plant_Box model as fast as the CPU allows, with live plotting
disabled, and records the full state history so runs can be compared offline.

Nothing here changes the plant model -- it is the same Plant_Box class.
"""

import random

import matplotlib
matplotlib.use("Agg")  # headless backend: save figures to disk, no GUI window
import matplotlib.pyplot as plt

from Plant_Box import Plant_Box


def run(controller_fn, set_point=40.0, steps=300, seed=0):
    """Run a closed-loop simulation and return the recorded history.

    Args:
        controller_fn: callable (curr_temp, set_point) -> control power.
                       Same signature as controller() in Controller.py.
        set_point:     target plant temperature in Celsius.
        steps:         number of 1-second control steps to simulate.
        seed:          RNG seed.  Plant_Box uses the global `random` module for
                       its noise, so seeding here makes runs reproducible --
                       essential for comparing two controllers fairly.

    Returns:
        dict of lists: time, temp (plant), heater (internal heater temp),
        control (controller output), all of length `steps`.
    """
    random.seed(seed)

    box = Plant_Box()
    box.plot_temp(False)  # disable the live plot; we plot afterwards instead

    history = {"time": [], "temp": [], "heater": [], "control": []}

    for k in range(steps):
        temp = box.get_temp()
        u = controller_fn(temp, set_point)

        box.set_control_power(u)
        box.update_temp()

        # Record the state the controller acted on, plus its output.
        history["time"].append(k)
        history["temp"].append(temp)
        history["heater"].append(box.heater_temp)
        history["control"].append(u)

    return history


def metrics(history, set_point=40.0, settle_band=0.5, tail=50):
    """Reduce a run to the numbers we actually judge a controller by."""
    temps = history["temp"]

    peak = max(temps)
    overshoot = max(0.0, peak - set_point)

    # Settling time: first step after which the plant never again leaves the
    # +/- settle_band window around the set point.
    settle_time = None
    for k in range(len(temps)):
        if all(abs(t - set_point) <= settle_band for t in temps[k:]):
            settle_time = k
            break

    # Steady-state error: mean offset over the last `tail` samples.
    tail_temps = temps[-tail:]
    steady_error = sum(tail_temps) / len(tail_temps) - set_point

    return {
        "peak": peak,
        "overshoot": overshoot,
        "settle_time": settle_time,
        "steady_error": steady_error,
        "final_control": history["control"][-1],
    }


def plot(histories, set_point=40.0, title="Temperature vs Time",
         filename="run.png", show_control=True):
    """Plot one or more runs.

    Args:
        histories: dict mapping a label -> history dict from run().
    """
    n_axes = 2 if show_control else 1
    fig, axes = plt.subplots(n_axes, 1, figsize=(10, 7 if show_control else 4),
                             sharex=True)
    if n_axes == 1:
        axes = [axes]

    ax = axes[0]
    for label, h in histories.items():
        ax.plot(h["time"], h["temp"], label=label)
    ax.axhline(set_point, color="k", linestyle="--", linewidth=1,
               label=f"set point ({set_point}C)")
    ax.set_ylabel("Plant temperature (C)")
    ax.set_title(title)
    ax.legend(loc="best")
    ax.grid(alpha=0.3)

    if show_control:
        ax = axes[1]
        for label, h in histories.items():
            ax.plot(h["time"], h["control"], label=label)
        ax.axhline(0, color="k", linewidth=0.8)
        ax.set_ylabel("Control signal u")
        ax.set_xlabel("Time (s)")
        ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(filename, dpi=120)
    plt.close(fig)
    print(f"saved {filename}")


def report(label, history, set_point=40.0):
    """Print the metrics for one run in a readable one-liner."""
    m = metrics(history, set_point)
    settle = "never" if m["settle_time"] is None else f"{m['settle_time']}s"
    print(
        f"{label:<24} "
        f"peak={m['peak']:6.2f}C  "
        f"overshoot={m['overshoot']:5.2f}C  "
        f"settle(+/-0.5C)={settle:>6}  "
        f"steady_err={m['steady_error']:+6.2f}C  "
        f"final_u={m['final_control']:7.3f}"
    )


if __name__ == "__main__":
    # Step 0 baseline: the starter controller, which always returns 0.
    # With no control power the plant simply sits at ambient temperature.
    def no_control(curr_temp, set_point):
        return 0.0

    h = run(no_control, steps=300)
    report("no control (u=0)", h)
    plot({"no control (u=0)": h},
         title="Step 0 baseline: controller returns 0",
         filename="figures/step0_baseline.png")
