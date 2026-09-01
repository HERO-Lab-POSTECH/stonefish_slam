import numpy as np


class OculusProperty(object):
    def __init__(self, tilt_angle_deg: float = 0.0):
        # model
        self.model = ""

        # range bins: [r1, ..., rn]
        self.num_ranges = None
        self.ranges = None
        # r[i] - r[i - 1]
        self.range_resolution = None
        # n * resolution (set from sonar data during initialization)
        self.range_max = None

        # bearings: [b1, ..., bm]
        self.num_beams = None
        # rad
        self.bearings = None
        # b[m] - b[1]
        self.horizontal_fov = np.radians(130.)
        # mean: (b[m] - b[1]) / m
        self.angular_resolution = None
        # rad
        self.vertical_fov = None

        # Tilt angle for FLS (Forward Looking Sonar)
        self.tilt_angle_deg = tilt_angle_deg
        self.tilt_angle_rad = np.deg2rad(tilt_angle_deg)

        ##################################################
        # polar <-> Cartesian
        ##################################################
        # functions to transform between bearings and cols
        # col -> bearing
        self.c2b = None
        # col <- bearing
        self.b2c = None
        # row -> range
        self.ro2ra = None
        # row <- range
        self.ra2ro = None

        # parameters for remapping from polar to Cartesian
        self.remap_x = None
        self.remap_y = None

    def __str__(self):
        d = dict(self.__dict__)
        d["angular_resolution"] = np.degrees(d["angular_resolution"])
        d["horizontal_fov"] = np.degrees(d["horizontal_fov"])
        d["vertical_fov"] = np.degrees(d["vertical_fov"])
        return (
            "\n===============================\n"
            "         Oculus Property\n"
            "===============================\n"
            "Model: {model:>24}\n"
            "#Ranges: {num_ranges:>22.0f}\n"
            "Range resolution: {range_resolution:>12.2f}m\n"
            "#Bearings: {num_beams:>21}\n"
            "Angular resolution: {angular_resolution:>8.1f}deg\n"
            "Horizontal aperture: {horizontal_fov:>7.1f}deg\n"
            "Vertical aperture: {vertical_fov:>9.1f}deg\n"
            "===============================\n".format(**d)
        )
