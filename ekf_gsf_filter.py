import numpy as np
from scipy.linalg import block_diag
from scipy.stats import multivariate_normal
from scipy.spatial.transform import Rotation as R


class EKF:
    def __init__(self, state_dim, Q, H, R):
        self.x = np.zeros(state_dim)  # State: [pos(3), vel(3), quat(4), gyro_bias(3), acc_bias(3)]
        self.x[6:10] = np.array([0, 0, 0, 1])
        self.P = np.eye(state_dim)  # Initial covariance
        self.H = H  # Measurement matrix
        self.R = R  # Measurement noise
        self.Q = Q  # Process noise
        self.y = None  # Innovation
        self.S = None  # Innovation covariance

    def predict(self, dt, gyro_meas, accel_meas):
        gyro_bias = self.x[10:13]
        acc_bias = self.x[13:16]

        # Correct measurements with biases
        omega = gyro_meas - gyro_bias
        acc = accel_meas - acc_bias

        # Update attitude (Euler integration, not ideal for large angles)
        q = self.x[6:10]
        dq = self.quat_derivative(q, omega)
        q_new = q + dq * dt
        q_new /= np.linalg.norm(q_new)  # нормализация
        self.x[6:10] = q_new

        # Rotate acceleration to world frame (simplified)
        r = R.from_quat(q_new)
        acc_world = r.apply(acc) + np.array([0, 0, 9.81])

        # Update velocity and position
        self.x[3:6] += acc_world * dt
        self.x[0:3] += self.x[3:6] * dt

        # Process noise integration
        F = self.compute_jacobian(dt)
        self.P = F @ self.P @ F.T + self.Q

    def quat_derivative(self, q, omega):
        # Кватернион как [x, y, z, w]
        qx, qy, qz, qw, = q[0], q[1], q[2],q[3]
        ox, oy, oz = omega

        # Матрица умножения кватернионов
        Omega = 0.5 * np.array([
            [0, -ox, -oy, -oz],
            [ox, 0, oz, -oy],
            [oy, -oz, 0, ox],
            [oz, oy, -ox, 0]
        ])
        return Omega @ q

    def compute_jacobian(self, dt):
        # Basic Jacobian for constant velocity model
        F = np.eye(16)
        F[0:3, 3:6] = np.eye(3) * dt
        return F

    def update(self, z):
        H = self.H
        R = self.R
        x = self.x
        P = self.P

        z_pred = H @ x
        self.y = z - z_pred
        self.S = H @ P @ H.T + R

        try:
            K = P @ H.T @ np.linalg.inv(self.S)
        except np.linalg.LinAlgError:
            return  # Handle singular matrix

        self.x = x + K @ self.y
        self.P = (np.eye(16) - K @ H) @ P


class GSF:
    def __init__(self, sensor_combinations, H_matrices, R_matrices, Q, state_dim=16):
        self.filters = []
        self.weights = []
        self.combinations = sensor_combinations

        for combo in sensor_combinations:
            H_list = [H_matrices[s] for s in combo]
            R_blocks = [R_matrices[s] for s in combo]

            H_combined = np.vstack(H_list)
            R_combined = block_diag(*R_blocks)

            ekf = EKF(state_dim, Q, H_combined, R_combined)
            self.filters.append(ekf)

        self.weights = np.ones(len(sensor_combinations)) / len(sensor_combinations)

    def predict_all(self, dt, imu_data):
        for ekf, combo in zip(self.filters, self.combinations):
            accel = np.array([imu_data[combo[0]]['ax_acc'],
                              imu_data[combo[0]]['ay_acc'],
                              imu_data[combo[0]]['az_acc']])
            gyro = np.array([imu_data[combo[1]]['roll'],
                             imu_data[combo[1]]['pitch'],
                             imu_data[combo[1]]['yaw']])
            ekf.predict(dt, gyro, accel)

    def update_all(self, sensor_data):
        for i, (ekf, combo) in enumerate(zip(self.filters, self.combinations)):
            z = []
            valid = True
            for sensor in combo:
                if sensor not in sensor_data:
                    valid = False
                    break
                # Build measurement vector based on sensor type
                if 'accel' in sensor:
                    z.extend([sensor_data[sensor]['ax_acc'],
                              sensor_data[sensor]['ay_acc'],
                              sensor_data[sensor]['az_acc']])
                elif 'gyro' in sensor:
                    z.extend([sensor_data[sensor]['roll'],
                              sensor_data[sensor]['pitch'],
                              sensor_data[sensor]['yaw']])
                elif 'baro' in sensor:
                    z.append(sensor_data[sensor]['altitude'])
                elif 'mag' in sensor:
                    z.append(np.deg2rad(sensor_data[sensor]['heading']))

            if not valid:
                continue

            ekf.update(np.array(z))
            likelihood = multivariate_normal.pdf(ekf.y, cov=ekf.S)
            self.weights[i] *= likelihood

        # Normalize weights
        total = np.sum(self.weights)
        if total > 0:
            self.weights /= total
        else:
            self.weights = np.ones_like(self.weights) / len(self.weights)

    def get_estimate(self):
        state = np.zeros_like(self.filters[0].x)
        covariance = np.zeros_like(self.filters[0].P)

        for w, ekf in zip(self.weights, self.filters):
            state += w * ekf.x

        for w, ekf in zip(self.weights, self.filters):
            diff = ekf.x - state
            covariance += w * (ekf.P + np.outer(diff, diff))

        return {'state': state, 'covariance': covariance}
