import numpy as np


class EKF:
    def __init__(self):
        self.x = np.zeros((16, 1))  # [x y z vx vy vz qx qy qz qw bgx bgy bgz bax bay baz]
        self.P = np.eye(16) * 0.1
        self.F = np.eye(16)
        self.Q = np.eye(16) * 0.01
        self.H = None

    def predict(self, dt):
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z, R):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(16) - K @ self.H) @ self.P

    def get_state(self):
        return self.x


class GSF:
    def __init__(self, sensor_combinations, H_matrices, R_matrices):
        self.sensor_combinations = sensor_combinations
        self.H_matrices = H_matrices
        self.R_matrices = R_matrices
        self.ekf_filters = [EKF() for _ in sensor_combinations]
        self.weights = np.ones(len(sensor_combinations)) / len(sensor_combinations)

    def predict_all(self, dt):
        for f in self.ekf_filters:
            f.predict(dt)

    def update_all(self, sensor_data):
        likelihoods = []

        for i, (ekf, combination) in enumerate(zip(self.ekf_filters, self.sensor_combinations)):
            z_parts = []
            H_rows = []
            R_blocks = []

            for sensor_id in combination:
                measurement = sensor_data.get(sensor_id)
                if measurement is None:
                    continue
                z_vec = np.array(list(measurement.values())).reshape(-1, 1)
                H = self.H_matrices[sensor_id]
                R = self.R_matrices[sensor_id]

                z_parts.append(z_vec)
                H_rows.append(H)
                R_blocks.append(R)

            if not z_parts:
                likelihoods.append(1e-10)
                continue

            z = np.vstack(z_parts)
            ekf.H = np.vstack(H_rows)
            R = np.block([[R if i == j else np.zeros_like(R) for j, R in enumerate(R_blocks)]
                          for i, R in enumerate(R_blocks)])

            try:
                y = z - ekf.H @ ekf.x
                S = ekf.H @ ekf.P @ ekf.H.T + R
                inv_S = np.linalg.inv(S)
                det_S = np.linalg.det(S)
                exp_term = np.exp(-0.5 * y.T @ inv_S @ y)
                likelihood = (1.0 / np.sqrt((2 * np.pi) ** len(z) * det_S)) * exp_term
                likelihoods.append(float(likelihood))
                ekf.update(z, R)
            except np.linalg.LinAlgError:
                likelihoods.append(1e-10)

        likelihoods = np.array(likelihoods)
        self.weights *= likelihoods
        self.weights += 1e-10
        self.weights /= np.sum(self.weights)

    def get_estimate(self):
        weighted = sum(w * f.x for w, f in zip(self.weights, self.ekf_filters))
        return weighted
