import csv
import utm
import numpy as np
from scipy import interpolate


class Kalman3d:
    def __init__(self, imutime, an, ae, ad, gpstime, lat, lon, alt) -> None:
        # Arrays of data

        # IMU
        self.imutime = np.array(imutime)
        self.an = np.array(an)  # accel north
        self.ae = np.array(ae)  # accel east
        self.ad = np.array(ad)  # accel down

        # GPS
        self.gpstime = np.array(gpstime)
        self.lat = np.array(lat)
        self.lon = np.array(lon)
        self.alt = np.array(alt)
        self.eastings = []
        self.northings = []
        self.alt_relative = []

    def llm2utm(self):
        (eastings, northings, _, _) = utm.from_latlon(self.latitudes, self.longitudes)
        altitudes = np.array(self.alt)
        eastings = np.array(eastings)
        northings = np.array(northings)
        self.alt_relative = (
            -(altitudes - altitudes[0]) * -1.0
        )  # convert to relative, down is positive
        self.eastings = eastings - eastings[0]  # convert to relative
        self.northings = northings - northings[0]  # convert to relative

    """Interpolate the GPS to the same timescales as the imu data"""

    def interpolate_gps(self):
        self.eastings = interpolate.interp1d(self.gpstime, self.eastings)(self.imutime)
        self.northings = interpolate.interp1d(self.gpstime, self.northings)(
            self.imutime
        )
        self.alt_relative = interpolate.interp1d(self.gpstime, self.alt_relative)(
            self.imutime
        )

    def process(self):
        self.llm2utm()
        self.interpolate_gps()
        self.filter()

    def filter(self):
        # Process the kalman filter
        n_steps = len(self.imutime)

        # State vector: [s_n, s_e, s_d, v_n, v_e, v_d, a_n, a_e, a_d]
        x = np.zero(9, 1)
        P = 1e-3 * np.identity(9)

        # Quick and hacky estimation of timestep
        dt = self.imutime[1] - self.imutime[0]

        # Process noise
        Q = np.diag([1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-3, 1e-3, 1e-3])
        # Measurement noise
        gps_var = 1e-6
        imu_var = 1e-3
        R = np.diag([gps_var, gps_var, gps_var * 10, imu_var, imu_var, imu_var])

        # Discrete time model
        # s_n(k+1) = s_n(k) + v_n(k) * dt
        # v_n(k+1) = v_n(k) + a_n(k) * dt
        # a_n(k+1) = a_n(k)
        F = np.array(
            [
                [1, 0, 0, dt, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, dt, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, dt, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, dt, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, dt, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, dt],
                [0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1],
            ]
        )

        H = np.array(
            [
                [1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1],
            ]
        )

        # Remove gravity from NED accel, leaving linear accels in NED frame
        an = an - 1
        # Convert to ms^-2
        an = an * 9.81
        ae = ae * 9.81
        ad = ad * 9.81

        sensors = np.stack(self.northings, self.eastings, self.alt_relative, an, ae, ad)

        kal_x_stor = np.zero(9, n_steps)

        for i in range(0, n_steps - 1):
            # Predict
            xp = F * x  # no inputs
            Pp = F * P * np.conj(F) + Q

            # Update
            y = sensors(i) - H * x
            S = H * P * np.conj(H) + R
            K = Pp * np.conj(H) * np.linalg.inv(S)
            x = xp + K * y
            P = (np.identity(9) - K * H) * Pp

            # Store
            kal_x_stor[:, i] = x
