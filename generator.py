import numpy as np
from typing import Dict


class Sensor:
    def __init__(self, sigma: float = 0.0):
        self.sigma = sigma
        self.failures: Dict[str, dict] = {}

    def add_failure(self, failure_id: str, failure_type: str, parameters: dict, start_time: float):
        self.failures[failure_id] = {
            'type': failure_type,
            'parameters': parameters,
            'start_time': start_time
        }

    def remove_failure(self, failure_id: str):
        self.failures.pop(failure_id, None)

    def generate_true_signal(self, t: float) -> dict:
        raise NotImplementedError

    def apply_noise(self, signal: dict):
        return {k: v + np.random.normal(0, self.sigma) for k, v in signal.items()}

    def apply_failures(self, signal: dict, t: float):
        for failure in self.failures.values():
            if t >= failure['start_time']:
                signal = self._apply_failure(signal, failure, t)
        return signal

    def _apply_failure(self, signal: dict, failure: dict, t: float):
        return signal

    def read(self, t: float) -> dict:
        signal = self.generate_true_signal(t)
        signal = self.apply_noise(signal)
        return self.apply_failures(signal, t)


class Accelerometer(Sensor):
    def generate_true_signal(self, t: float) -> dict:
        return {
            'x': np.sin(0.1 * t),
            'y': np.cos(0.1 * t),
            'z': 9.81 + 0.5 * np.sin(0.05 * t)
        }

    def _apply_failure(self, signal: dict, failure: dict, t: float):
        typ, p = failure['type'], failure['parameters']
        if typ == 'bias':
            for axis in p.get('axes', signal):
                signal[axis] += p['value']
        elif typ == 'stuck':
            for axis in p.get('axes', signal):
                signal[axis] = p['value']
        elif typ == 'drift':
            drift = p['rate'] * (t - failure['start_time'])
            for axis in p.get('axes', signal):
                signal[axis] += drift
        return signal


class Gyroscope(Sensor):
    def generate_true_signal(self, t: float) -> dict:
        return {
            'x': 0.1 * np.sin(0.1 * t),
            'y': 0.1 * np.cos(0.1 * t),
            'z': 0.05 * t % 6.28
        }

    def _apply_failure(self, signal: dict, failure: dict, t: float):
        typ, p = failure['type'], failure['parameters']
        if typ == 'bias':
            for axis in p.get('axes', signal):
                signal[axis] += p['value']
        elif typ == 'stuck':
            for axis in p.get('axes', signal):
                signal[axis] = p['value']
        elif typ == 'drift':
            drift = p['rate'] * (t - failure['start_time'])
            for axis in p.get('axes', signal):
                signal[axis] += drift
        return signal


class Magnetometer(Sensor):
    def generate_true_signal(self, t: float) -> dict:
        return {'heading': (0.01 * t) % 360}

    def _apply_failure(self, signal: dict, failure: dict, t: float):
        typ, p = failure['type'], failure['parameters']
        if typ == 'stuck':
            signal['heading'] = p['value']
        return signal


class Barometer(Sensor):
    def generate_true_signal(self, t: float) -> dict:
        return {'altitude': 100 + 10 * np.sin(0.05 * t)}

    def _apply_failure(self, signal: dict, failure: dict, t: float):
        typ, p = failure['type'], failure['parameters']
        if typ == 'stuck':
            signal['altitude'] = p['value']
        elif typ == 'drift':
            signal['altitude'] += p['rate'] * (t - failure['start_time'])
        return signal


class SensorSimulator:
    def __init__(self):
        self.sensors = {
            'accel_1': Accelerometer(),
            'accel_2': Accelerometer(),
            'accel_3': Accelerometer(),
            'gyro_1': Gyroscope(),
            'gyro_2': Gyroscope(),
            'gyro_3': Gyroscope(),
            'baro_1': Barometer(),
            'baro_2': Barometer(),
            'mag_1': Magnetometer()
        }
