"""One-joint physical AI baseline. Offline numerical model, not hardware validation."""
import argparse
import csv
import json
import math
from pathlib import Path


def simulate(inertia=0.05, damping=0.08, gravity=0.5, nominal_gravity=0.5,
             limit=2.0, target=0.5, dt=0.001, duration=4.0, kp=4.0, kd=0.8):
    values = (inertia, damping, gravity, nominal_gravity, limit, target, dt, duration, kp, kd)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('All parameters must be finite')
    if inertia <= 0 or dt <= 0 or duration <= 0 or limit <= 0:
        raise ValueError('Inertia, step, duration and torque limit must be positive')
    if damping < 0 or kp < 0 or kd < 0 or gravity < 0 or nominal_gravity < 0:
        raise ValueError('Damping, gains and gravity coefficients cannot be negative')
    steps = round(duration / dt)
    if steps < 1 or steps > 1000000:
        raise ValueError('Choose between 1 and 1000000 steps')
    q = velocity = 0.0
    rows = [(0.0, q, velocity, 0.0)]
    for step in range(steps):
        command = kp * (target - q) - kd * velocity + nominal_gravity * math.sin(q)
        torque = max(-limit, min(limit, command))
        acceleration = (torque - damping * velocity - gravity * math.sin(q)) / inertia
        # Semi-implicit Euler: update velocity before position.
        velocity += acceleration * dt
        q += velocity * dt
        if not math.isfinite(q) or not math.isfinite(velocity):
            raise ValueError('Numerical divergence: reduce step or gains')
        rows.append(((step + 1) * dt, q, velocity, torque))
    return rows


CASES = {
    'nominal': {},
    'model-mismatch': {'inertia': 0.08, 'gravity': 0.8},
    'torque-limited': {'limit': 0.12},
}


def run(output):
    output.mkdir(parents=True, exist_ok=True)
    report = {}
    for name, params in CASES.items():
        rows = simulate(**params)
        with (output / (name + '.csv')).open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time_s', 'position_rad', 'velocity_rad_s', 'torque_Nm'])
            writer.writerows(rows)
        report[name] = {
            'final_error_rad': abs(0.5 - rows[-1][1]),
            'maximum_abs_torque_Nm': max(abs(row[3]) for row in rows),
            'steps': len(rows) - 1,
        }
    (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('results'))
    run(parser.parse_args().output)
