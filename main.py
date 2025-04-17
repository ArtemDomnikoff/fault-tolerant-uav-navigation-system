from generator import SensorSimulator
from ekf_gsf_filter import GSF
from visualization import Plotter
from itertools import product
import numpy as np

dt = 0.1
total_time = 20.0

accs = ['acc1', 'acc2', 'acc3']
gyros = ['gyro1', 'gyro2', 'gyro3']
baros = ['baro1', 'baro2']
mag = 'mag'
sensor_combinations = [(*c, mag) for c in product(accs, gyros, baros)]

H_matrices = {
    'acc1': np.eye(3, 16, k=13),
    'acc2': np.eye(3, 16, k=13),
    'acc3': np.eye(3, 16, k=13),

    'gyro1': np.eye(3, 16, k=10),
    'gyro2': np.eye(3, 16, k=10),
    'gyro3': np.eye(3, 16, k=10),

    'baro1': np.eye(1, 16, k=2),
    'baro2': np.eye(1, 16, k=2),

    'mag': np.eye(1, 16, k=8)
}

R_matrices = {
    'acc1': np.diag([0.02 ** 2] * 3),
    'acc2': np.diag([0.02 ** 2] * 3),
    'acc3': np.diag([0.02 ** 2] * 3),

    'gyro1': np.diag([0.01 ** 2] * 3),
    'gyro2': np.diag([0.01 ** 2] * 3),
    'gyro3': np.diag([0.01 ** 2] * 3),

    'baro1': np.array([[0.5 ** 2]]),
    'baro2': np.array([[0.5 ** 2]]),

    'mag': np.array([[np.deg2rad(5.0) ** 2]])
}

gsf = GSF(sensor_combinations, H_matrices, R_matrices)

simulator = SensorSimulator()

simulator.sensors['accel_1'].sigma = 0.02
simulator.sensors['accel_1'].add_failure(
    failure_id='stuck_x',
    failure_type='stuck',
    parameters={'axes': ['x'], 'value': 0.0},
    start_time=5.0
)

plotter = Plotter()

t = 0.0

while t < total_time:
    sensor_data = {'timestamp': t}
    for sid, sensor in simulator.sensors.items():
        sensor_data[sid] = sensor.read(t)
    gsf.predict_all(dt)
    gsf.update_all(sensor_data)
    estimate = gsf.get_estimate()
    plotter.update(sensor_data, estimate)
    t += dt
plotter.plot()
