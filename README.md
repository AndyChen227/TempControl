# Temperature Control

## English

### Project goal

This is my solution to the SARP-UW **Temperature Control - Hard** software
problem. The goal is to keep a plant at 40 C using its measured temperature
and a control signal that can heat or cool the box.

### Repository files

| File | Purpose |
|---|---|
| `.gitignore` | Prevents local Python cache and editor files from being uploaded. It is not part of the controller logic. |
| `Controller.py` | My solution. It contains the PID controller and the provided main loop. |
| `Plant_Box.py` | The simulation model supplied by SARP-UW. Its simulation behavior has not been changed. |
| `README.md` | Explains the controller, gain selection, and steady-state calculation in English and Chinese. |
| `Temperature_Control_Hard_Problem.pdf` | The original SARP-UW problem statement. |

### Controller code

This is the same basic PID version I developed originally. Only formatting and
comments were cleaned up; the control logic remains the same.

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

The provided loop runs once per second, so the controller assumes a one-second
time step.

- **P** responds to the current error.
- **I** accumulates past error and supplies the continuing power needed to
  replace heat lost to the room.
- **D** responds to how quickly the error changes and reduces power when the
  temperature approaches the target quickly.

### How the gains were selected

The gains were tested one at a time in simulation instead of being guessed all
at once.

#### `Kp = 1.0`

The initial error is `40 - 20 = 20 C`, so `Kp = 1` initially produces a
proportional output of 20. It gives a reasonable heating rate without the large
overshoot caused by higher proportional gains. P control alone cannot reach
40 C because its output becomes smaller near the target while the box is still
losing heat.

#### `Ki = 0.05`

A larger `Ki` removes the steady-state error faster, but it also creates more
integral windup and overshoot. With `Kp = 1` and no D term, representative
tests gave:

| Ki | Overshoot | Settling time |
|---:|---:|---:|
| 0.01 | 0.09 C | 319 s |
| 0.02 | 0.77 C | 123 s |
| 0.05 | 4.91 C | 62 s |
| 0.10 | 10.12 C | 118 s |

`Ki = 0.05` builds the required steady control effort reasonably quickly. The
integral limit is then used to control windup.

#### Why the steady control signal is `6.25`

At steady state, the plant and heater temperatures stop changing. Let `H` be
the heater temperature and `u` be the control signal. The room is at 20 C and
the target plant temperature is 40 C.

The plant equation gives:

```text
0 = 0.08(H - 40) + 0.02(20 - 40)
H = 45 C
```

The heater must remain at 45 C. The heater equation then gives:

```text
0 = 0.20u - 0.05(45 - 20)
u = 6.25
```

Therefore, the controller needs an average control signal of approximately
`6.25` to hold the plant at 40 C. This is a model control-signal value, not a
power value in watts.

Near steady state, P is approximately zero because the temperature error is
small. D is approximately zero because the error is no longer changing. The I
term therefore supplies the continuing output:

```text
Ki * integral = 6.25
0.05 * integral = 6.25
integral = 125
```

The integral is not manually set to 125. It starts at zero and naturally moves
toward this value as errors accumulate. `I_MAX = 130` is slightly above 125,
so it allows enough steady output while preventing excessive windup.

#### `Kd = 5.0`

After adding the integral limit, representative tests gave:

| Kd | Overshoot | Settling time |
|---:|---:|---:|
| 0 | 3.03 C | 62 s |
| 2 | 1.59 C | 62 s |
| 5 | 0.41 C | 24 s |
| 10 | 0.24 C | 41 s |

`Kd = 5` gave the best balance between overshoot and settling time. A larger D
gain reduced overshoot slightly, but it settled more slowly and was more
sensitive to sensor noise.

### Result

For the fixed test seed used during tuning, the final controller had about
0.41 C overshoot, settled within +/-0.5 C in about 25 seconds, and had about
0.02 C steady-state error.

### Run

```bash
pip install matplotlib
python Controller.py
```

---

# 温度控制

## 中文版

### 项目目标

这是我对 SARP-UW **Temperature Control - Hard** 软件题目的解答。目标是只使用
植物当前温度和目标温度，输出一个能够加热或制冷的控制信号，将植物温度保持在
40 C。

### 仓库文件

| 文件 | 用途 |
|---|---|
| `.gitignore` | 防止本地 Python 缓存和编辑器文件被上传，不参与控制器运行。 |
| `Controller.py` | 我的解答，包含 PID 控制器和题目提供的主循环。 |
| `Plant_Box.py` | SARP-UW 提供的模拟模型，模拟逻辑没有被修改。 |
| `README.md` | 使用英文和中文解释控制器、参数选择以及稳态计算。 |
| `Temperature_Control_Hard_Problem.pdf` | SARP-UW 的原始题目。 |

### 控制器代码

这就是我最初完成的基础 PID 版本。这里只整理了格式和注释，控制逻辑没有改变。

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

题目提供的循环每秒运行一次，因此这个控制器按照一秒的时间间隔计算。

- **P** 根据当前误差作出反应。
- **I** 累积过去的误差，提供抵消房间散热所需的持续控制输出。
- **D** 根据误差变化的速度作出反应；当温度快速接近目标时，它会减少加热。

### 参数是如何选择的

这些参数不是一次猜出来的，而是在模拟中固定其他参数后逐个测试得到的。

#### `Kp = 1.0`

初始误差为 `40 - 20 = 20 C`，所以 `Kp = 1` 最初产生的比例输出是20。它能提供
合理的升温速度，同时避免较大的比例增益带来的严重超调。只使用P无法到达40 C，
因为接近目标时P输出会逐渐减小，但箱子仍在向房间散热。

#### `Ki = 0.05`

较大的 `Ki` 可以更快消除稳态误差，但也会造成更多积分饱和和超调。当
`Kp = 1` 且没有D项时，代表性测试结果如下：

| Ki | 超调 | 稳定时间 |
|---:|---:|---:|
| 0.01 | 0.09 C | 319秒 |
| 0.02 | 0.77 C | 123秒 |
| 0.05 | 4.91 C | 62秒 |
| 0.10 | 10.12 C | 118秒 |

`Ki = 0.05` 能较快建立系统所需的持续控制输出，之后再使用积分上限控制
windup。

#### 为什么稳态控制信号是 `6.25`

在稳态下，植物温度和加热器温度都不再变化。设 `H` 是加热器温度，`u` 是控制
信号。房间温度为20 C，植物目标温度为40 C。

根据植物温度方程：

```text
0 = 0.08(H - 40) + 0.02(20 - 40)
H = 45 C
```

因此加热器必须保持在45 C。再代入加热器方程：

```text
0 = 0.20u - 0.05(45 - 20)
u = 6.25
```

因此，控制器平均需要大约 `6.25` 的控制信号才能把植物保持在40 C。这个数是
模型中的控制信号，不代表6.25瓦。

接近稳态时，温度误差很小，所以P接近0；误差不再变化，所以D接近0。因此持续
输出主要由I项提供：

```text
Ki * integral = 6.25
0.05 * integral = 6.25
integral = 125
```

程序没有手动把integral设成125。它从0开始，随着误差的累积自然接近125。
`I_MAX = 130` 略高于125，既允许积分提供足够的稳态输出，又能防止积分过度
累积。

#### `Kd = 5.0`

加入积分上限之后，代表性测试结果如下：

| Kd | 超调 | 稳定时间 |
|---:|---:|---:|
| 0 | 3.03 C | 62秒 |
| 2 | 1.59 C | 62秒 |
| 5 | 0.41 C | 24秒 |
| 10 | 0.24 C | 41秒 |

`Kd = 5` 在超调和稳定时间之间取得了最好的平衡。更大的D增益虽然稍微降低了
超调，但稳定速度更慢，而且对传感器噪声更加敏感。

### 最终结果

在调参时使用的固定随机种子下，最终控制器的超调约为0.41 C，在大约25秒内
进入 +/-0.5 C 的范围，稳态误差约为0.02 C。

### 运行方法

```bash
pip install matplotlib
python Controller.py
```
