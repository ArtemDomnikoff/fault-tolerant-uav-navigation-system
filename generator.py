import numpy as np
from typing import Dict


class Sensor:
    def __init__(self, sigma: float = 0.0):
        self.sigma = sigma
        self.failures: Dict[str, dict] = {}
        self._allowed_params = {}

    def add_failure(self, failure_id: str, failure_type: str, parameters: dict, start_time: float):
        self.failures[failure_id] = {
            'type': failure_type,
            'parameters': parameters,
            'start_time': start_time
        }

    def remove_failure(self, failure_id: str):
        self.failures.pop(failure_id, None)

    def generate_true_signal(self, t: float, *args) -> dict:
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

    def read(self, t: float, **kwargs) -> dict:
        signal = self.generate_true_signal(t, **kwargs)
        signal = self.apply_noise(signal)
        return self.apply_failures(signal, t)


class Accelerometer(Sensor):
    def generate_true_signal(self, t: float, px_acc=0.0, py_acc=0.0, pz_acc=0.0, **kwargs) -> dict:
        return {
            'x': px_acc,
            'y': py_acc,
            'z': 9.81 + pz_acc
        }

    def _apply_failure(self, signal: dict, failure: dict, t: float):
        typ, p = failure['type'], failure['parameters']
        axes = p.get('axes', signal)

        if typ == 'bias':
            for axis in axes:
                signal[axis] += p['value']
        elif typ == 'stuck':
            if 'value' not in p:
                stuck_values = {axis: signal[axis] for axis in axes}
                p['value'] = stuck_values
            for axis in axes:
                signal[axis] = p['value'][axis]
        elif typ == 'drift':
            drift = p['rate'] * (t - failure['start_time'])
            for axis in axes:
                signal[axis] += drift
        return signal


class Gyroscope(Sensor):

    def generate_true_signal(self, t: float, roll=0.0, pitch=0.0, yaw=0.0, **kwargs) -> dict:
        return {
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }

    def _apply_failure(self, signal: dict, failure: dict, t: float):
        typ, p = failure['type'], failure['parameters']
        axes = p.get('axes', signal)

        if typ == 'bias':
            for axis in axes:
                signal[axis] += p['value']
        elif typ == 'stuck':
            if 'value' not in p:
                stuck_values = {axis: signal[axis] for axis in axes}
                p['value'] = stuck_values
            for axis in axes:
                signal[axis] = p['value'][axis]
        elif typ == 'drift':
            drift = p['rate'] * (t - failure['start_time'])
            for axis in axes:
                signal[axis] += drift
        return signal


class Magnetometer(Sensor):
    def generate_true_signal(self, t: float, heading=0.0, **kwargs) -> dict:
        return {'heading': heading%360}

    def _apply_failure(self, signal: dict, failure: dict, t: float):
        typ, p = failure['type'], failure['parameters']
        if typ == 'stuck':
            if 'value' not in p:
                p['value'] = signal['heading']
            signal['heading'] = p['value']
        return signal


class Barometer(Sensor):
    def generate_true_signal(self, t: float, altitude=0.0, climb_rate=0.0, **kwargs) -> dict:
        return {'altitude': altitude}

    def _apply_failure(self, signal: dict, failure: dict, t: float):
        typ, p = failure['type'], failure['parameters']
        if typ == 'stuck':
            if 'value' not in p:
                p['value'] = signal['altitude']
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
