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
sensor_combinations = [(*c, 'mag') for c in product(accs, gyros, baros)]

H_matrices = {
    'acc_1': np.eye(3, 16, k=13),
    'acc_2': np.eye(3, 16, k=13),
    'acc_3': np.eye(3, 16, k=13),

    'gyro_1': np.eye(3, 16, k=10),
    'gyro_2': np.eye(3, 16, k=10),
    'gyro_3': np.eye(3, 16, k=10),

    'baro_1': np.eye(1, 16, k=2),
    'baro_2': np.eye(1, 16, k=2),

    'mag_1': np.eye(1, 16, k=8)
}

R_matrices = {
    'acc_1': np.diag([0.02 ** 2] * 3),
    'acc_2': np.diag([0.02 ** 2] * 3),
    'acc_3': np.diag([0.02 ** 2] * 3),

    'gyro_1': np.diag([0.01 ** 2] * 3),
    'gyro_2': np.diag([0.01 ** 2] * 3),
    'gyro_3': np.diag([0.01 ** 2] * 3),

    'baro_1': np.array([[0.5 ** 2]]),
    'baro_2': np.array([[0.5 ** 2]]),

    'mag_1': np.array([[np.deg2rad(5.0) ** 2]])
}

gsf = GSF(sensor_combinations, H_matrices, R_matrices)

simulator = SensorSimulator()

# simulator.sensors['accel_1'].sigma = 0.01
# simulator.sensors['accel_2'].sigma = 0.02
# simulator.sensors['accel_3'].sigma = 0.03
# simulator.sensors['gyro_1'].sigma = 0.001
# simulator.sensors['gyro_2'].sigma = 0.02
# simulator.sensors['gyro_3'].sigma = 0.03
# simulator.sensors['baro_1'].sigma = 0.01
# simulator.sensors['baro_2'].sigma = 0.02
# simulator.sensors['mag_1'].sigma = 0.01

# simulator.sensors['accel_1'].add_failure(
#     failure_id='stuck_x',
#     failure_type='stuck',
#     parameters={'axes': ['x']},
#     start_time=5.0
# )
# simulator.sensors['accel_1'].add_failure(
#     failure_id='stuck_y',
#     failure_type='stuck',
#     parameters={'axes': ['y']},
#     start_time=7.0
# )

plotter = Plotter()

t = 0.0

while t < total_time:
    sensor_data = {'timestamp': t}
    px = 0.0
    climb_rate = 0.0
    pz = 0.0
    altitude = 0.0
    pitch = 0.0

    if t <= 1.0:
        pass
    elif 1.0 < t <= 5.0:
        px = (t - 1.0) * 0.7
        climb_rate = (t - 1.0) * 1.5
        pz = (t - 1.0) * climb_rate
        altitude = pz
        pitch = -0.2
    elif 5.0 < t <= 8.0:
        px = prev_px
        climb_rate = max(prev_pz_acc, 0.0)
        pz = prev_pz
        altitude = prev_altitude if climb_rate == 0 else pz
        pitch = -0.1
    else:
        px = prev_px
        climb_rate = max(prev_pz_acc, 0.0)
        pz = prev_pz
        altitude = prev_altitude if climb_rate == 0 else pz
        pitch = -0.1

    if 'prev_px' in locals():
        px_acc = (px - prev_px) / dt
    else:
        px_acc = 0.0
    prev_px = px

    if 'prev_pz_acc' in locals():
        pz_acc = (climb_rate - prev_pz_acc) / dt
    else:
        pz_acc = 0.0
    prev_pz_acc = climb_rate

    prev_pz = pz

    if 'prev_altitude' not in locals():
        prev_altitude = altitude
    prev_altitude = altitude

    kwargs = {
        'px': px,
        'py': 0.0,
        'pz': pz,
        'roll': 0.0,
        'pitch': pitch,
        'yaw': 0.0,
        'altitude': altitude,
        'climb_rate': climb_rate,
        'heading': 0.0,
        'px_acc': px_acc,
        'pz_acc': pz_acc
    }

    for sid, sensor in simulator.sensors.items():
        sensor_data[sid] = sensor.read(t, **kwargs)

    gsf.predict_all(dt)
    gsf.update_all(sensor_data)
    estimate = gsf.get_estimate()
    plotter.update(sensor_data, estimate)
    t += dt
plotter.plot()