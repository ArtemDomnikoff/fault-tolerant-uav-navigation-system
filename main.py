from generator import SensorSimulator
from ekf_gsf_filter import GSF
from visualization import Plotter
from itertools import product
import numpy as np
import matplotlib as plt

dt = 0.1
total_time = 20.0

# Sensor combinations (corrected names)
accs = ['accel_1', 'accel_2', 'accel_3']
gyros = ['gyro_1', 'gyro_2', 'gyro_3']
baros = ['baro_1', 'baro_2']
mag = ['mag_1']
sensor_combinations = [combo + tuple(mag) for combo in product(accs, gyros, baros)]

# Measurement matrices (updated for 15-state vector)
H_matrices = {
    'accel_1': np.hstack([np.zeros((3, 13)), np.eye(3)]),
    'accel_2': np.hstack([np.zeros((3, 13)), np.eye(3)]),
    'accel_3': np.hstack([np.zeros((3, 13)), np.eye(3)]),

    'gyro_1': np.hstack([np.zeros((3, 10)), np.eye(3), np.zeros((3, 3))]),
    'gyro_2': np.hstack([np.zeros((3, 10)), np.eye(3), np.zeros((3, 3))]),
    'gyro_3': np.hstack([np.zeros((3, 10)), np.eye(3), np.zeros((3, 3))]),

    'baro_1': np.array([[0, 0, 1, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]),
    'baro_2': np.array([[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]),

    'mag_1': np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]])
}

# Measurement noise matrices
R_matrices = {
    'accel_1': np.diag([0.1 ** 2] * 3),
    'accel_2': np.diag([0.1 ** 2] * 3),
    'accel_3': np.diag([0.1 ** 2] * 3),

    'gyro_1': np.diag([0.05 ** 2] * 3),
    'gyro_2': np.diag([0.05 ** 2] * 3),
    'gyro_3': np.diag([0.05 ** 2] * 3),

    'baro_1': np.array([[0.5 ** 2]]),
    'baro_2': np.array([[0.5 ** 2]]),

    'mag_1': np.array([[np.deg2rad(5) ** 2]])
}

# Process noise matrix
Q = np.diag([
    0.0 ** 2, 0.0 ** 2, 0.0 ** 2,  # Position
    0.5 ** 2, 0.5 ** 2, 0.5 ** 2,  # Velocity
    0.005 ** 2, 0.005 ** 2, 0.005 ** 2, 0.005**2,  # Attitude
    0.001 ** 2, 0.001 ** 2, 0.001 ** 2,  # Gyro bias
    0.005 ** 2, 0.005 ** 2, 0.005 ** 2  # Accel bias
])

gsf = GSF(sensor_combinations, H_matrices, R_matrices, Q)
simulator = SensorSimulator()
plotter = Plotter()

# Physics variables
position = np.zeros(3)
velocity = np.zeros(3)
acceleration = np.zeros(3)
roll_angle = 0.0
roll_rate = 0.0
pitch_angle = 0.0
pitch_rate = 0.0
yaw_angle = 0.0
yaw_rate = 0.0

t = 0.0
while t < total_time:
    # Phase-based motion control
    if t <= 1.0:
        # Hover phase
        acceleration[:] = 0
        target_pitch = 0.0
        target_roll = 0.0
        target_yaw = 0.0
    elif 1.0 < t <= 5.0:
        # Acceleration phase
        acceleration[0] = 0.7  # Forward acceleration
        acceleration[2] = 1.5  # Upward acceleration
        target_pitch = -0.1  # Nose-down pitch
        target_roll = 0.0
        target_yaw = 0.0
    else:
        # Glide phase
        acceleration[:] = 0
        velocity[2] = 0
        target_pitch = -0.1
        target_roll = 0.0
        target_yaw = 0.0

    # Smooth pitch transitions
    if 1.0 < t <= 5.0:
        roll_rate = (target_roll - roll_angle) / (5.0 - t if t < 5.0 else dt)
        pitch_rate = (target_pitch - pitch_angle) / (5.0 - t if t < 5.0 else dt)
        yaw_rate = (target_roll - roll_angle) / (5.0 - t if t < 5.0 else dt)
    else:
        roll_rate = (target_roll - roll_angle) / dt
        pitch_rate = (target_pitch - pitch_angle) / dt
        yaw_rate = (target_yaw - yaw_angle) / dt

    pitch_angle += pitch_rate * dt
    roll_angle += roll_rate * dt
    yaw_angle += yaw_rate * dt

    # Physics integration
    velocity += acceleration * dt
    position += velocity * dt
    # Generate body-frame accelerations
    rot_matrix = np.array([
        [np.cos(pitch_angle), 0, -np.sin(pitch_angle)],
        [0, 1, 0],
        [np.sin(pitch_angle), 0, np.cos(pitch_angle)]
    ])

    body_acceleration = acceleration

    # Sensor data generation
    sensor_data = {'timestamp': t}
    # IMU data
    for i in [1, 2, 3]:
        sensor_data[f'accel_{i}'] = simulator.sensors[f'accel_{i}'].read(t,
                                                                         roll=roll_angle,
                                                                         pitch=pitch_angle,
                                                                         ax=body_acceleration[0],
                                                                         ay=body_acceleration[1],
                                                                         az=body_acceleration[2]
                                                                         )

        sensor_data[f'gyro_{i}'] = simulator.sensors[f'gyro_{i}'].read(t,
                                                                       roll=roll_angle,
                                                                       pitch=pitch_angle,
                                                                       yaw=yaw_angle
                                                                       )

    # Barometer and magnetometer
    sensor_data['baro_1'] = simulator.sensors['baro_1'].read(t, altitude=position[2])
    sensor_data['baro_2'] = simulator.sensors['baro_2'].read(t, altitude=position[2])
    sensor_data['mag_1'] = simulator.sensors['mag_1'].read(t, heading=0.0)

    # Filter processing
    gsf.predict_all(dt, sensor_data)
    gsf.update_all(sensor_data)
    estimate = gsf.get_estimate()

    plotter.update(sensor_data, estimate)
    t += dt
plotter.plot()
