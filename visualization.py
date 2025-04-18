import matplotlib.pyplot as plt
from collections import deque


class Plotter:
    def __init__(self, max_len=1000):
        self.max_len = max_len

        self.data = {
            'accel': {'x': deque(maxlen=max_len), 'y': deque(maxlen=max_len), 'z': deque(maxlen=max_len)},
            'gyro': {'roll': deque(maxlen=max_len), 'pitch': deque(maxlen=max_len), 'yaw': deque(maxlen=max_len)},
            'baro': {'altitude': deque(maxlen=max_len)},
            'mag': {'heading': deque(maxlen=max_len)},
            'estimate': {'x': deque(maxlen=max_len), 'y': deque(maxlen=max_len), 'z': deque(maxlen=max_len)},
            't': deque(maxlen=max_len)
        }

        self.fig, self.axes = plt.subplots(3, 2, figsize=(12, 8))
        plt.ion()
        plt.show()

    def update(self, sensor_data, estimate):
        t = sensor_data['timestamp']
        self.data['t'].append(t)
        acc = sensor_data.get('accel_1', {})
        for axis in ['x', 'y', 'z']:
            self.data['accel'][axis].append(acc.get(axis, 0))
        gyro = sensor_data.get('gyro_1', {})
        for axis in ['roll', 'pitch', 'yaw']:
            self.data['gyro'][axis].append(gyro.get(axis, 0))
        baro = sensor_data.get('baro_1', {})
        self.data['baro']['altitude'].append(baro.get('altitude', 0))
        mag = sensor_data.get('mag_1', {})
        self.data['mag']['heading'].append(mag.get('heading', 0))

        self.data['estimate']['x'].append(float(estimate[0]))
        self.data['estimate']['y'].append(float(estimate[1]))
        self.data['estimate']['z'].append(float(estimate[2]))
        self.plot()

    def plot(self):
        self.axes[0, 0].cla()
        self.axes[0, 0].set_title("Accelerometer")
        for axis in ['x', 'y', 'z']:
            self.axes[0, 0].plot(self.data['t'], self.data['accel'][axis], label=axis)
        self.axes[0, 0].legend()

        self.axes[0, 1].cla()
        self.axes[0, 1].set_title("Gyroscope")
        for axis in ['roll', 'pitch', 'yaw']:
            self.axes[0, 1].plot(self.data['t'], self.data['gyro'][axis], label=axis)
        self.axes[0, 1].legend()

        self.axes[1, 0].cla()
        self.axes[1, 0].set_title("Barometer")
        self.axes[1, 0].plot(self.data['t'], self.data['baro']['altitude'], label='altitude')
        self.axes[1, 0].legend()

        self.axes[1, 1].cla()
        self.axes[1, 1].set_title("Magnetometer")
        self.axes[1, 1].plot(self.data['t'], self.data['mag']['heading'], label='heading')
        self.axes[1, 1].legend()

        self.axes[2, 0].cla()
        self.axes[2, 0].set_title("GSF Position Estimate")
        for axis in ['x', 'y', 'z']:
            self.axes[2, 0].plot(self.data['t'], self.data['estimate'][axis], label=axis)
        self.axes[2, 0].legend()

        self.axes[2, 1].axis('off')
        plt.pause(0.1)
