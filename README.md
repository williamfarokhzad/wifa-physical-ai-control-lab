# Physical AI Control Lab: Robot Joint Simulation and Model Mismatch

Build an offline one-joint control experiment with Python's standard library. Run three examples, inspect CSV output and test the implementation before trying more complex robot learning tools.

**Full illustrated tutorial:** https://wifarobotics.com/learn/physical-ai-control/

This is a classical feedback-control baseline for studying physical AI. It does not train a neural network, run a VLA model, or demonstrate transfer to real hardware. Every result in this project is simulated.

## Start from zero

1. Install Python 3.10 or newer from https://www.python.org/downloads/. On Windows, enable the Python launcher during installation.
2. Click the green Code button on this repository, choose Download ZIP, then extract it.
3. Open the extracted folder in a terminal. On Windows, right-click the folder and choose Open in Terminal.
4. Check the interpreter with `python --version`. If that command is missing on Windows use `py`; on Linux or macOS use `python3` in every command below.
5. Run `python main.py`. No pip install, camera, GPU, account, robot or paid API is needed.
6. Run `python -m unittest discover -s tests -v`. All eight test methods should pass.

## What the model means

The joint rotates in a vertical plane. Angle q is measured in radians from the downward position. The simplified equation is:

```text
J * acceleration = torque - b * velocity - G * sin(q)
command = Kp * (target - q) - Kd * velocity + G_nominal * sin(q)
torque = clip(command, -limit, +limit)
```

J is joint inertia in kg m². b is viscous damping in N m s/rad. G is the mass times gravity times center-of-mass distance in N m. Kp is 4 N m/rad and Kd is 0.8 N m s/rad by default. Time step is 0.001 s and duration is 4 s. Integration is semi-implicit Euler, not an exact solution.

## Three experiments

| Case | Change | Question |
| --- | --- | --- |
| nominal | J=0.05, G=0.5, torque limit=2 | Does matching gravity compensation remove static error? |
| model-mismatch | J=0.08, G=0.8, nominal G stays 0.5 | How does an incorrect plant model affect tracking? |
| torque-limited | Torque limit=0.12 | Can control gains overcome insufficient actuator torque? |

`results/report.json` contains final absolute tracking error, maximum commanded torque after clipping, and step count. Each CSV contains time, position, velocity and applied torque. Open it in a spreadsheet and plot position against time. These are numerical outputs, not measured hardware performance.

At the 0.5 rad target, gravity requires `0.5*sin(0.5)`, about 0.2397 N m. A 0.12 N m actuator cannot hold that target in this model. Its static gravity-only equilibrium is `asin(0.12/0.5)`, about 0.2424 rad. With increased plant gravity, equilibrium solves `4*(0.5-q) - 0.3*sin(q)=0`. This explains residual error without claiming a learning algorithm solves it.

## Inspect and change

Edit CASES in main.py to change one parameter at a time. Keep the original cases as a reference. Use `python main.py --output experiment2` to save a second run. Halve dt in a separate call to simulate and compare trajectories; the test suite checks the maximum sampled difference between 1 ms and 0.5 ms integration. A convergence check supports the numerical implementation, not the accuracy of the simplified physics.

## Test coverage and boundaries

Tests check the known first-step acceleration, zero equilibrium, nominal tracking, actuator clipping, saturation failure, parameter rejection, step convergence and model mismatch. They do not check real motors, friction beyond a viscous term, gearbox compliance, measurement noise, communication delays or physical safety. Do not connect this example to a moving machine.

## Troubleshooting

If Python is missing, reopen the terminal after installing it. If tests cannot import main, run the command from the repository root. If CSV files are locked by a spreadsheet, close it or select another output folder. If a changed configuration diverges, reduce the integration step and review gains, units and the plant assumptions.

## Continue learning

Compare this baseline with imitation learning only after defining observations, actions, data splits and a held-out evaluation. Useful background: [Modern Robotics Chapter 11](https://modernrobotics.northwestern.edu/chapters/chapter11/) and [WIFA's robot design path](https://wifarobotics.com/academy/robot-design/).

Original educational implementation by WIFA Robotics. No license grant is implied; contact the owner for reuse permission.
