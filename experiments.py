"""
Reproduces every experiment and figure referenced in README.md.

Run with:   python experiments.py

Each controller below is one step in the development history.  They are kept
in the repository (rather than deleted as they were superseded) because the
reasoning that led from one to the next is the actual content of this project.

All runs share a fixed RNG seed, so two controllers face the identical noise
sequence and any difference in the result is attributable to the controller.
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import sim

SET_POINT = 40.0
FIGDIR = "figures"


# ---------------------------------------------------------------------------
# Controllers, in the order they were developed
# ---------------------------------------------------------------------------

def no_control():
    """v0 -- the starter code.  Never touches the heat pump."""
    return lambda temp, sp: 0.0


def full_power(u=100.0):
    """v1 -- always on.  Reproduces the 'it is like an oven' failure from the
    problem statement.  Ignores both of its inputs."""
    return lambda temp, sp: u


def bang_bang(u=100.0):
    """v2 -- first controller that actually reads the sensor, but it only ever
    asks 'too hot or too cold?', never 'by how much?'."""
    return lambda temp, sp: u if temp < sp else -u


def p_control(kp):
    """v3 -- output proportional to the error.  Eases off near the set point,
    but cannot hold a non-zero output at zero error, so it always settles
    short of the target."""
    return lambda temp, sp: kp * (sp - temp)


def pi_control(kp, ki):
    """v4 -- the integral term accumulates error over time, so it can hold a
    non-zero output once the error reaches zero.  Kills steady-state error,
    but winds up during the long climb and overshoots."""
    state = {"i": 0.0}

    def ctrl(temp, sp):
        error = sp - temp
        state["i"] += error
        return kp * error + ki * state["i"]

    return ctrl


def pid_control(kp, ki, kd, i_max=None):
    """v5 (final) -- adds a derivative term for early braking and clamps the
    integrator to stop windup.  i_max=None disables the clamp, which is how
    the 'D only' comparison in the README is produced."""
    state = {"i": 0.0, "e": None}

    def ctrl(temp, sp):
        error = sp - temp

        state["i"] += error
        if i_max is not None:
            state["i"] = max(-i_max, min(i_max, state["i"]))

        derivative = 0.0 if state["e"] is None else error - state["e"]
        state["e"] = error

        return kp * error + ki * state["i"] + kd * derivative

    return ctrl


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def fig_evolution():
    """Every version on one axis: the whole story in a single picture."""
    versions = [
        ("v0  no control",          no_control(),                 "tab:gray"),
        ("v2  bang-bang +/-100",    bang_bang(),                  "tab:red"),
        ("v3  P  (Kp=1)",           p_control(1.0),               "tab:orange"),
        ("v3  P  (Kp=5)",           p_control(5.0),               "gold"),
        ("v4  PI (Kp=1, Ki=0.05)",  pi_control(1.0, 0.05),        "tab:purple"),
        ("v5  PID + clamp",         pid_control(1.0, 0.05, 5.0, 130.0), "tab:green"),
    ]

    fig, ax = plt.subplots(figsize=(11, 6))
    for label, ctrl, color in versions:
        h = sim.run(ctrl, SET_POINT, steps=200)
        lw = 2.6 if label.startswith("v5") else 1.6
        ax.plot(h["time"], h["temp"], label=label, color=color, lw=lw)

    ax.axhline(SET_POINT, color="k", ls="--", lw=1.2, label="set point (40C)")
    ax.axhspan(SET_POINT, 56, color="red", alpha=0.05)
    ax.text(120, 52, "Pickles cooks up here", color="crimson", fontsize=10)
    ax.set_ylim(15, 56)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Plant temperature (C)")
    ax.set_title("Every version, same noise seed")
    ax.legend(loc="lower right", fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f"{FIGDIR}/evolution.png", dpi=120)
    plt.close(fig)


def fig_final():
    """The final controller on its own, with the tolerance band drawn in."""
    h = sim.run(pid_control(1.0, 0.05, 5.0, 130.0), SET_POINT, steps=400)
    m = sim.metrics(h, SET_POINT)

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True,
                                 gridspec_kw={"height_ratios": [2, 1]})
    a1.plot(h["time"], h["temp"], color="tab:green", lw=2.2)
    a1.axhline(SET_POINT, color="k", ls="--", lw=1.2)
    a1.axhspan(SET_POINT - 0.5, SET_POINT + 0.5, color="tab:green", alpha=0.12,
               label="+/-0.5C band")
    a1.set_ylabel("Plant temperature (C)")
    a1.set_title(
        f"Final PID: overshoot {m['overshoot']:.2f}C, "
        f"settles in {m['settle_time']}s, steady-state error {m['steady_error']:+.2f}C"
    )
    a1.legend(loc="lower right")
    a1.grid(alpha=0.3)

    a2.plot(h["time"], h["control"], color="darkgreen", lw=1.4)
    a2.axhline(6.25, color="gray", ls="--", lw=1)
    a2.text(250, 7.2, "6.25 = control effort needed to hold 40C", color="gray", fontsize=9)
    a2.set_ylabel("Control signal u")
    a2.set_xlabel("Time (s)")
    a2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(f"{FIGDIR}/final_pid.png", dpi=120)
    plt.close(fig)


def fig_terms():
    """Who does the work, and when."""
    kp, ki, kd, i_max = 1.0, 0.05, 5.0, 130.0
    state = {"i": 0.0, "e": None}
    terms = []

    def ctrl(temp, sp):
        error = sp - temp
        state["i"] = max(-i_max, min(i_max, state["i"] + error))
        d = 0.0 if state["e"] is None else error - state["e"]
        state["e"] = error
        p_t, i_t, d_t = kp * error, ki * state["i"], kd * d
        terms.append((p_t, i_t, d_t))
        return p_t + i_t + d_t

    h = sim.run(ctrl, SET_POINT, steps=140)
    t = h["time"]

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 8), sharex=True,
                                 gridspec_kw={"height_ratios": [1, 1.2]})
    a1.plot(t, h["temp"], lw=2.2, color="tab:blue")
    a1.axhline(SET_POINT, color="k", ls="--", lw=1.2)
    a1.set_ylabel("Temperature (C)")
    a1.set_title("Which term is doing the work, and when")
    a1.grid(alpha=0.3)

    a2.plot(t, [x[0] for x in terms], lw=2, color="tab:orange",
            label="P  -- how far off right now")
    a2.plot(t, [x[1] for x in terms], lw=2, color="tab:green",
            label="I  -- how long it has been off")
    a2.plot(t, [x[2] for x in terms], lw=2, color="tab:red",
            label="D  -- how fast it is changing")
    a2.plot(t, [sum(x) for x in terms], lw=1.2, color="k", ls=":", label="total u")
    a2.axhline(6.25, color="gray", ls="--", lw=1)
    a2.axhline(0, color="k", lw=0.8)
    a2.set_xlabel("Time (s)")
    a2.set_ylabel("Contribution to u")
    a2.legend(loc="upper right", fontsize=9)
    a2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(f"{FIGDIR}/pid_terms.png", dpi=120)
    plt.close(fig)


def fig_windup():
    """The integral term with and without the clamp -- the root-cause figure."""
    def traced(i_max):
        state = {"i": 0.0, "e": None}
        rec = []

        def ctrl(temp, sp):
            error = sp - temp
            state["i"] += error
            if i_max is not None:
                state["i"] = max(-i_max, min(i_max, state["i"]))
            d = 0.0 if state["e"] is None else error - state["e"]
            state["e"] = error
            rec.append(0.05 * state["i"])
            return 1.0 * error + 0.05 * state["i"] + 5.0 * d

        return ctrl, rec

    c_no, i_no = traced(None)
    h_no = sim.run(c_no, SET_POINT, steps=120)
    c_cl, i_cl = traced(130.0)
    h_cl = sim.run(c_cl, SET_POINT, steps=120)

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    a1.plot(h_no["time"], h_no["temp"], color="tab:red", lw=2, label="no clamp")
    a1.plot(h_cl["time"], h_cl["temp"], color="tab:green", lw=2, label="clamped at 130")
    a1.axhline(SET_POINT, color="k", ls="--", lw=1.2)
    a1.set_ylabel("Temperature (C)")
    a1.set_title("Integral windup is the root cause of the overshoot")
    a1.legend()
    a1.grid(alpha=0.3)

    a2.plot(h_no["time"], i_no, color="tab:red", lw=2, label="I term, no clamp")
    a2.plot(h_cl["time"], i_cl, color="tab:green", lw=2, label="I term, clamped")
    a2.axhline(6.25, color="gray", ls="--", lw=1)
    a2.text(70, 6.6, "6.25 = what is actually needed", color="gray", fontsize=9)
    a2.annotate("wound up to 9.24 --\nthe excess can only be\nunwound by going ABOVE 40C",
                xy=(16, 9.24), xytext=(34, 8.6), color="crimson", fontsize=9,
                arrowprops=dict(arrowstyle="->", color="crimson"))
    a2.set_ylabel("I term contribution")
    a2.set_xlabel("Time (s)")
    a2.legend(loc="lower right")
    a2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(f"{FIGDIR}/windup.png", dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
# The results table quoted in the README
# ---------------------------------------------------------------------------

def results_table():
    rows = [
        ("v0  no control",              no_control()),
        ("v1  full power (u=100)",      full_power()),
        ("v2  bang-bang +/-100",        bang_bang()),
        ("v3  P   Kp=1",                p_control(1.0)),
        ("v3  P   Kp=5",                p_control(5.0)),
        ("v4  PI  Kp=1 Ki=0.05",        pi_control(1.0, 0.05)),
        ("v4b PI  + clamp only",        pid_control(1.0, 0.05, 0.0, 130.0)),
        ("v4c PID + D only, no clamp",  pid_control(1.0, 0.05, 5.0, None)),
        ("v5  PID + clamp  (final)",    pid_control(1.0, 0.05, 5.0, 130.0)),
    ]

    header = f"{'version':<30}{'peak':>8}{'overshoot':>11}{'settle':>9}{'steady err':>12}"
    print(header)
    print("-" * len(header))
    for label, ctrl in rows:
        h = sim.run(ctrl, SET_POINT, steps=500)
        m = sim.metrics(h, SET_POINT)
        settle = "never" if m["settle_time"] is None else f"{m['settle_time']}s"
        print(f"{label:<30}{m['peak']:>8.2f}{m['overshoot']:>11.2f}"
              f"{settle:>9}{m['steady_error']:>+12.2f}")


if __name__ == "__main__":
    results_table()
    print()
    for name, fn in [("evolution", fig_evolution), ("final_pid", fig_final),
                     ("pid_terms", fig_terms), ("windup", fig_windup)]:
        fn()
        print(f"saved {FIGDIR}/{name}.png")
