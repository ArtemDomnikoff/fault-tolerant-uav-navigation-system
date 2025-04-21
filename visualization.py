import matplotlib.pyplot as plt
from collections import deque
import numpy as np


class Plotter:
    def __init__(self, max_len=1000):
        self.max_len = max_len
        self.data = {
<<<<<<< Updated upstream
<<<<<<< Updated upstream
            'accel': {'x': deque(maxlen=max_len), 'y': deque(maxlen=max_len), 'z': deque(maxlen=max_len)},
            'gyro': {'x': deque(maxlen=max_len), 'y': deque(maxlen=max_len), 'z': deque(maxlen=max_len)},
            'baro': {'altitude': deque(maxlen=max_len)},
            'mag': {'heading': deque(maxlen=max_len)},
            'estimate': {'x': deque(maxlen=max_len), 'y': deque(maxlen=max_len), 'z': deque(maxlen=max_len)},
=======
=======
>>>>>>> Stashed changes
            'sensors': {
                'accel': {'x': deque(maxlen=max_len), 'y': deque(maxlen=max_len), 'z': deque(maxlen=max_len)},
                'gyro': {'roll': deque(maxlen=max_len), 'pitch': deque(maxlen=max_len), 'yaw': deque(maxlen=max_len)},
                'baro': {'altitude': deque(maxlen=max_len)},
                'mag': {'heading': deque(maxlen=max_len)}
            },
            'estimate': {
                'position': {'x': deque(maxlen=max_len), 'y': deque(maxlen=max_len), 'z': deque(maxlen=max_len)},
                'velocity': {'x': deque(maxlen=max_len), 'y': deque(maxlen=max_len), 'z': deque(maxlen=max_len)},
                'attitude': {'roll': deque(maxlen=max_len), 'pitch': deque(maxlen=max_len),
                             'yaw': deque(maxlen=max_len)},
                'biases': {
                    'gyro': {'x': deque(maxlen=max_len), 'y': deque(maxlen=max_len), 'z': deque(maxlen=max_len)},
                    'accel': {'x': deque(maxlen=max_len), 'y': deque(maxlen=max_len), 'z': deque(maxlen=max_len)}
                }
            },
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
            't': deque(maxlen=max_len)
        }

        self.fig, self.axes = plt.subplots(4, 2, figsize=(14, 12))
        plt.subplots_adjust(hspace=0.5)
        plt.ion()
        plt.show()

    def update(self, sensor_data, estimate):
        t = sensor_data['timestamp']
        self.data['t'].append(t)

        # Store sensor data
        acc = sensor_data.get('accel_1', {})
        for axis in ['x', 'y', 'z']:
            self.data['sensors']['accel'][axis].append(acc.get(axis, 0))

        gyro = sensor_data.get('gyro_1', {})
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        for axis in ['x', 'y', 'z']:
            self.data['gyro'][axis].append(gyro.get(axis, 0))
        baro = sensor_data.get('baro_1', {})
        self.data['baro']['altitude'].append(baro.get('altitude', 0))
        mag = sensor_data.get('mag_1', {})
        self.data['mag']['heading'].append(mag.get('heading', 0))

        self.data['estimate']['x'].append(float(estimate[0]))
        self.data['estimate']['y'].append(float(estimate[1]))
        self.data['estimate']['z'].append(float(estimate[2]))
=======
        for axis in ['roll', 'pitch', 'yaw']:
            self.data['sensors']['gyro'][axis].append(gyro.get(axis, 0))

        baro = sensor_data.get('baro_1', {})
        self.data['sensors']['baro']['altitude'].append(baro.get('altitude', 0))

        mag = sensor_data.get('mag_1', {})
        self.data['sensors']['mag']['heading'].append(mag.get('heading', 0))

        # Store estimates
        state = estimate['state']
        self.data['estimate']['position']['x'].append(state[0])
        self.data['estimate']['position']['y'].append(state[1])
        self.data['estimate']['position']['z'].append(state[2])

        self.data['estimate']['velocity']['x'].append(state[3])
        self.data['estimate']['velocity']['y'].append(state[4])
        self.data['estimate']['velocity']['z'].append(state[5])

        self.data['estimate']['attitude']['roll'].append(np.rad2deg(state[6]))
        self.data['estimate']['attitude']['pitch'].append(np.rad2deg(state[7]))
        self.data['estimate']['attitude']['yaw'].append(np.rad2deg(state[8]))

        self.data['estimate']['biases']['gyro']['x'].append(state[9])
        self.data['estimate']['biases']['gyro']['y'].append(state[10])
        self.data['estimate']['biases']['gyro']['z'].append(state[11])

        self.data['estimate']['biases']['accel']['x'].append(state[12])
        self.data['estimate']['biases']['accel']['y'].append(state[13])
        self.data['estimate']['biases']['accel']['z'].append(state[14])

=======
        for axis in ['roll', 'pitch', 'yaw']:
            self.data['sensors']['gyro'][axis].append(gyro.get(axis, 0))

        baro = sensor_data.get('baro_1', {})
        self.data['sensors']['baro']['altitude'].append(baro.get('altitude', 0))

        mag = sensor_data.get('mag_1', {})
        self.data['sensors']['mag']['heading'].append(mag.get('heading', 0))

        # Store estimates
        state = estimate['state']
        self.data['estimate']['position']['x'].append(state[0])
        self.data['estimate']['position']['y'].append(state[1])
        self.data['estimate']['position']['z'].append(state[2])

        self.data['estimate']['velocity']['x'].append(state[3])
        self.data['estimate']['velocity']['y'].append(state[4])
        self.data['estimate']['velocity']['z'].append(state[5])

        self.data['estimate']['attitude']['roll'].append(np.rad2deg(state[6]))
        self.data['estimate']['attitude']['pitch'].append(np.rad2deg(state[7]))
        self.data['estimate']['attitude']['yaw'].append(np.rad2deg(state[8]))

        self.data['estimate']['biases']['gyro']['x'].append(state[9])
        self.data['estimate']['biases']['gyro']['y'].append(state[10])
        self.data['estimate']['biases']['gyro']['z'].append(state[11])

        self.data['estimate']['biases']['accel']['x'].append(state[12])
        self.data['estimate']['biases']['accel']['y'].append(state[13])
        self.data['estimate']['biases']['accel']['z'].append(state[14])

>>>>>>> Stashed changes
        self.plot()
>>>>>>> Stashed changes

    def plot(self):
        axes = self.axes
        data = self.data

<<<<<<< Updated upstream
<<<<<<< Updated upstream
        self.axes[0, 1].cla()
        self.axes[0, 1].set_title("Gyroscope")
        for axis in ['x', 'y', 'z']:
            self.axes[0, 1].plot(self.data['t'], self.data['gyro'][axis], label=axis)
        self.axes[0, 1].legend()
=======
        # Sensors
        axes[0, 0].cla()
        axes[0, 0].set_title("Accelerometer (m/s²)")
        for axis in ['x', 'y', 'z']:
            axes[0, 0].plot(data['t'], data['sensors']['accel'][axis], label=f'{axis}-axis')
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        axes[0, 1].cla()
        axes[0, 1].set_title("Gyroscope (rad/s)")
        for axis in ['roll', 'pitch', 'yaw']:
            axes[0, 1].plot(data['t'], data['sensors']['gyro'][axis], label=axis)
        axes[0, 1].legend()
        axes[0, 1].grid(True)
>>>>>>> Stashed changes

        axes[1, 0].cla()
        axes[1, 0].set_title("Barometric Altitude (m)")
        axes[1, 0].plot(data['t'], data['sensors']['baro']['altitude'], 'b', label='Altitude')
        axes[1, 0].legend()
        axes[1, 0].grid(True)

        axes[1, 1].cla()
        axes[1, 1].set_title("Magnetometer Heading (deg)")
        axes[1, 1].plot(data['t'], data['sensors']['mag']['heading'], 'g', label='Heading')
        axes[1, 1].legend()
        axes[1, 1].grid(True)

        # Estimates
        axes[2, 0].cla()
        axes[2, 0].set_title("Position Estimate")
        for axis in ['x', 'y', 'z']:
            axes[2, 0].plot(data['t'], data['estimate']['position'][axis], label=f'{axis}-position')
        axes[2, 0].legend()
        axes[2, 0].grid(True)

<<<<<<< Updated upstream
        self.axes[2, 1].axis('off')
        plt.ioff()
        plt.show()
=======
=======
        # Sensors
        axes[0, 0].cla()
        axes[0, 0].set_title("Accelerometer (m/s²)")
        for axis in ['x', 'y', 'z']:
            axes[0, 0].plot(data['t'], data['sensors']['accel'][axis], label=f'{axis}-axis')
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        axes[0, 1].cla()
        axes[0, 1].set_title("Gyroscope (rad/s)")
        for axis in ['roll', 'pitch', 'yaw']:
            axes[0, 1].plot(data['t'], data['sensors']['gyro'][axis], label=axis)
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        axes[1, 0].cla()
        axes[1, 0].set_title("Barometric Altitude (m)")
        axes[1, 0].plot(data['t'], data['sensors']['baro']['altitude'], 'b', label='Altitude')
        axes[1, 0].legend()
        axes[1, 0].grid(True)

        axes[1, 1].cla()
        axes[1, 1].set_title("Magnetometer Heading (deg)")
        axes[1, 1].plot(data['t'], data['sensors']['mag']['heading'], 'g', label='Heading')
        axes[1, 1].legend()
        axes[1, 1].grid(True)

        # Estimates
        axes[2, 0].cla()
        axes[2, 0].set_title("Position Estimate")
        for axis in ['x', 'y', 'z']:
            axes[2, 0].plot(data['t'], data['estimate']['position'][axis], label=f'{axis}-position')
        axes[2, 0].legend()
        axes[2, 0].grid(True)

>>>>>>> Stashed changes
        axes[2, 1].cla()
        axes[2, 1].set_title("Velocity Estimate")
        for axis in ['x', 'y', 'z']:
            axes[2, 1].plot(data['t'], data['estimate']['velocity'][axis], label=f'{axis}-velocity')
        axes[2, 1].legend()
        axes[2, 1].grid(True)

        axes[3, 0].cla()
        axes[3, 0].set_title("Attitude Estimate (deg)")
        for axis in ['roll', 'pitch', 'yaw']:
            axes[3, 0].plot(data['t'], data['estimate']['attitude'][axis], label=axis)
        axes[3, 0].legend()
        axes[3, 0].grid(True)

        axes[3, 1].cla()
        axes[3, 1].set_title("Sensor Biases")
        for sensor in ['gyro', 'accel']:
            for axis in ['x', 'y', 'z']:
                axes[3, 1].plot(data['t'], data['estimate']['biases'][sensor][axis],
                                linestyle='--' if sensor == 'accel' else '-',
                                label=f'{sensor} {axis}')
        axes[3, 1].legend()
        axes[3, 1].grid(True)
        plt.plot()
<<<<<<< Updated upstream
        plt.pause(0.1)
>>>>>>> Stashed changes
=======
        plt.pause(0.1)
>>>>>>> Stashed changes
