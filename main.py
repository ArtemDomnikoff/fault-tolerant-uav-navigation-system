from generator import SensorSimulator
from ekf_gsf_filter import GSF
from visualization import Plotter
from itertools import product
import numpy as np

dt = 0.1
total_time = 20.0

<<<<<<< Updated upstream
<<<<<<< Updated upstream
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
=======
# Corrected sensor names matching SensorSimulator
accs = ['accel_1', 'accel_2', 'accel_3']
gyros = ['gyro_1', 'gyro_2', 'gyro_3']
baros = ['baro_1', 'baro_2']
mag = ['mag_1']
sensor_combinations = [combo + tuple(mag) for combo in product(accs, gyros, baros)]

H_matrices = {
    'accel_1': np.eye(3, 15, k=12),
    'accel_2': np.eye(3, 15, k=12),
    'accel_3': np.eye(3, 15, k=12),

    'gyro_1': np.eye(3, 15, k=9),
    'gyro_2': np.eye(3, 15, k=9),
    'gyro_3': np.eye(3, 15, k=9),

    'baro_1': np.array([[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]),  # Measuring altitude
    'baro_2': np.array([[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]),

    'mag_1': np.array([[0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]])  # Measuring yaw
>>>>>>> Stashed changes
=======
# Corrected sensor names matching SensorSimulator
accs = ['accel_1', 'accel_2', 'accel_3']
gyros = ['gyro_1', 'gyro_2', 'gyro_3']
baros = ['baro_1', 'baro_2']
mag = ['mag_1']
sensor_combinations = [combo + tuple(mag) for combo in product(accs, gyros, baros)]

H_matrices = {
    'accel_1': np.eye(3, 15, k=12),
    'accel_2': np.eye(3, 15, k=12),
    'accel_3': np.eye(3, 15, k=12),

    'gyro_1': np.eye(3, 15, k=9),
    'gyro_2': np.eye(3, 15, k=9),
    'gyro_3': np.eye(3, 15, k=9),

    'baro_1': np.array([[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]),  # Measuring altitude
    'baro_2': np.array([[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]),

    'mag_1': np.array([[0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]])  # Measuring yaw
>>>>>>> Stashed changes
}

# Measurement noise matrices
R_matrices = {
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    'acc1': np.diag([0.02 ** 2] * 3),
    'acc2': np.diag([0.02 ** 2] * 3),
    'acc3': np.diag([0.02 ** 2] * 3),

    'gyro1': np.diag([0.01 ** 2] * 3),
    'gyro2': np.diag([0.01 ** 2] * 3),
    'gyro3': np.diag([0.01 ** 2] * 3),

    'baro1': np.array([[0.5 ** 2]]),
    'baro2': np.array([[0.5 ** 2]]),

    'mag': np.array([[np.deg2rad(5.0) ** 2]])
=======
    'accel_1': np.diag([0.1 ** 2, 0.1 ** 2, 0.1 ** 2]),
    'accel_2': np.diag([0.1 ** 2, 0.1 ** 2, 0.1 ** 2]),
    'accel_3': np.diag([0.1 ** 2, 0.1 ** 2, 0.1 ** 2]),

    'gyro_1': np.diag([0.05 ** 2, 0.05 ** 2, 0.05 ** 2]),
    'gyro_2': np.diag([0.05 ** 2, 0.05 ** 2, 0.05 ** 2]),
    'gyro_3': np.diag([0.05 ** 2, 0.05 ** 2, 0.05 ** 2]),

    'baro_1': np.array([[1.0 ** 2]]),
    'baro_2': np.array([[1.0 ** 2]]),

    'mag_1': np.array([[0.1 ** 2]])
>>>>>>> Stashed changes
=======
    'accel_1': np.diag([0.1 ** 2, 0.1 ** 2, 0.1 ** 2]),
    'accel_2': np.diag([0.1 ** 2, 0.1 ** 2, 0.1 ** 2]),
    'accel_3': np.diag([0.1 ** 2, 0.1 ** 2, 0.1 ** 2]),

    'gyro_1': np.diag([0.05 ** 2, 0.05 ** 2, 0.05 ** 2]),
    'gyro_2': np.diag([0.05 ** 2, 0.05 ** 2, 0.05 ** 2]),
    'gyro_3': np.diag([0.05 ** 2, 0.05 ** 2, 0.05 ** 2]),

    'baro_1': np.array([[1.0 ** 2]]),
    'baro_2': np.array([[1.0 ** 2]]),

    'mag_1': np.array([[0.1 ** 2]])
>>>>>>> Stashed changes
}

# Process noise matrix (Q)
Q = np.diag([
    0.1 ** 2, 0.1 ** 2, 0.1 ** 2,  # Position
    0.5 ** 2, 0.5 ** 2, 0.5 ** 2,  # Velocity
    0.01 ** 2, 0.01 ** 2, 0.01 ** 2,  # Attitude
    0.001 ** 2, 0.001 ** 2, 0.001 ** 2,  # Gyro bias
    0.005 ** 2, 0.005 ** 2, 0.005 ** 2  # Accel bias
])
estimate = [0.0]
# Initialize GSF
gsf = GSF(
    sensor_combinations=sensor_combinations,
    H_matrices=H_matrices,
    R_matrices=R_matrices,
    Q=Q,
    state_dim=15
)

simulator = SensorSimulator()
<<<<<<< Updated upstream
<<<<<<< Updated upstream

simulator.sensors['accel_1'].sigma = 0.02
simulator.sensors['accel_1'].add_failure(
    failure_id='stuck_x',
    failure_type='stuck',
    parameters={'axes': ['x'], 'value': 0.0},
    start_time=5.0
)

=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
plotter = Plotter()

# Configure sensor noise and failures if needed
# simulator.sensors['accel_1'].sigma = 0.1
# simulator.sensors['gyro_1'].sigma = 0.0175
# simulator.sensors['baro_1'].sigma = 0.1

t = 0.0
previous_position = np.zeros(3)

while t < total_time:
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    sensor_data = {'timestamp': t}
    for sid, sensor in simulator.sensors.items():
        sensor_data[sid] = sensor.read(t)
    gsf.predict_all(dt)
=======
=======
>>>>>>> Stashed changes
    # Generate true motion parameters
    altitude = 0.0
    climb_rate = 0.0
    pitch = 0.0
    if t <= 1.0:
        pass  # Hovering
    elif 1.0 < t <= 5.0:
        climb_rate = 2.0
        pz = (t - 1.0) * climb_rate
        pitch = np.deg2rad(-15)
    elif 5.0 < t <= 10.0:
        climb_rate = -1.0
        pitch = np.deg2rad(10)
    else:
        climb_rate = 0.0
        pitch = 0.0

    # Calculate derivatives
    current_position = previous_position + np.array([0, 0, climb_rate]) * dt
    acceleration = (current_position - previous_position) / dt ** 2
    sensor_kwargs = {
        'px_acc': acceleration[0],
        'py_acc': acceleration[1],
        'pz_acc': acceleration[2],
        'altitude': current_position[2],
        'climb_rate': climb_rate,
        'pitch': pitch,
        'heading': 0.0  # Assuming North
    }

    # Generate sensor data
    sensor_data = {'timestamp': t}
    for sensor_id, sensor in simulator.sensors.items():
        sensor_data[sensor_id] = sensor.read(t, **sensor_kwargs)

    # Get IMU data for prediction (using first set of sensors)
    imu_data = {
        'accel': [
            sensor_data['accel_1']['x'],
            sensor_data['accel_1']['y'],
            sensor_data['accel_1']['z']
        ],
        'gyro': [
            sensor_data['gyro_1']['roll'],
            sensor_data['gyro_1']['pitch'],
            sensor_data['gyro_1']['yaw']
        ]
    }

    # Filter steps
    gsf.predict_all(dt, imu_data)
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
    gsf.update_all(sensor_data)

    # Store results
    estimate = gsf.get_estimate()
    plotter.update(sensor_data, estimate)

    previous_position = current_position
    t += dt
<<<<<<< Updated upstream
<<<<<<< Updated upstream
plotter.plot()
=======

plotter.plot()
>>>>>>> Stashed changes
=======

plotter.plot()
>>>>>>> Stashed changes
