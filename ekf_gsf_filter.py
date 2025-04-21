import numpy as np
from scipy.linalg import block_diag
from scipy.stats import multivariate_normal


class EKF:
    def __init__(self, state_dim, Q, H, R):
        self.x = np.zeros(state_dim)  # State: [pos(3), vel(3), euler(3), gyro_bias(3), acc_bias(3)]
        self.P = np.eye(state_dim)  # Initial covariance
        self.H = H  # Measurement matrix
        self.R = R  # Measurement noise
        self.Q = Q  # Process noise
        self.y = None  # Innovation
        self.S = None  # Innovation covariance

    def predict(self, dt, gyro_meas, accel_meas):
        # Simplified IMU-driven prediction (basic integration)
        # Extract biases
        gyro_bias = self.x[9:12]
        acc_bias = self.x[12:15]

        # Correct measurements with biases
        omega = gyro_meas - gyro_bias
        acc = accel_meas - acc_bias

        # Update attitude (Euler integration, not ideal for large angles)
        phi, theta, psi = self.x[6:9]
        self.x[6] += omega[0] * dt
        self.x[7] += omega[1] * dt
        self.x[8] += omega[2] * dt

        # Rotate acceleration to world frame (simplified)
        R = self.euler_to_rot(phi, theta)
        acc_world = R @ acc + np.array([0, 0, 9.81])

        # Update velocity and position
        self.x[3:6] += acc_world * dt
        self.x[0:3] += self.x[3:6] * dt

        # Process noise integration
        F = self.compute_jacobian(dt)
        self.P = F @ self.P @ F.T + self.Q

    def euler_to_rot(self, phi, theta):
        # Simplified rotation matrix (roll, pitch, yaw)
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi), np.cos(phi)]])

        R_y = np.array([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])

        return R_y @ R_x

    def compute_jacobian(self, dt):
        # Basic Jacobian for constant velocity model
        F = np.eye(15)
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
        self.P = (np.eye(15) - K @ H) @ P


class GSF:
    def __init__(self, sensor_combinations, H_matrices, R_matrices, Q, state_dim=15):
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
        for ekf in self.filters:
            gyro = np.array([imu_data['gyro'][0],
                             imu_data['gyro'][1],
                             imu_data['gyro'][2]])
            accel = np.array([imu_data['accel'][0],
                              imu_data['accel'][1],
                              imu_data['accel'][2]])
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
                    z.extend([sensor_data[sensor]['x'],
                              sensor_data[sensor]['y'],
                              sensor_data[sensor]['z']])
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

            try:
                ekf.update(np.array(z))
                likelihood = multivariate_normal.pdf(ekf.y, cov=ekf.S)
                self.weights[i] *= likelihood
            except:
                self.weights[i] = 0

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
