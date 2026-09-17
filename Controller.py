from Plant_Box import Plant_Box
import time

integral = 0.0
last_error = None


def controller(curr_temp: float, set_point: float) -> float:
    """Return the PID control signal for the heat pump."""
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


def main():
    box = Plant_Box()
    set_point = 40  # C

    # Main loop, do not change this
    while True:
        box.set_control_power(controller(box.get_temp(), set_point))
        box.update_temp()
        time.sleep(1)

if __name__ == "__main__":
    main()
