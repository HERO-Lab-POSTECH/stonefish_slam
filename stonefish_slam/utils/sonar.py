import numpy as np
from scipy.interpolate import interp1d
import cv2
# import rospy  # ROS2: Not needed

from stonefish_slam.utils.conversions import r2n


class OculusProperty(object):
    OCULUS_VERTICAL_APERTURE = {1: np.deg2rad(20), 2: np.deg2rad(12)}
    OCULUS_PART_NUMBER = {1042: "M1200d", 1032: "M750d"}

    noise = 0.01
    # fmt: off
    psf = np.array([[0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.0005, 0.0005, 0.0005, 0.0005, 0.    , 0.0005, 0.0005, 0.0005,
                     0.0005, 0.    , 0.    , 0.0005, 0.0005, 0.    , 0.    , 0.    ,
                     0.001 , 0.001 , 0.001 , 0.001 , 0.    , 0.    , 0.001 , 0.001 ,
                     0.001 , 0.    , 0.    , 0.001 , 0.0015, 0.002 , 0.0015, 0.0005,
                     0.    , 0.001 , 0.002 , 0.0025, 0.002 , 0.001 , 0.001 , 0.002 ,
                     0.003 , 0.003 , 0.0015, 0.    , 0.0025, 0.005 , 0.005 , 0.0035,
                     0.002 , 0.0105, 0.022 , 0.0355, 0.049 , 0.0615, 0.071 , 0.076 ,
                     0.076 , 0.071 , 0.0615, 0.049 , 0.0355, 0.022 , 0.0105, 0.002 ,
                     0.0035, 0.005 , 0.005 , 0.0025, 0.    , 0.0015, 0.003 , 0.003 ,
                     0.002 , 0.001 , 0.001 , 0.002 , 0.0025, 0.002 , 0.001 , 0.    ,
                     0.0005, 0.0015, 0.002 , 0.0015, 0.001 , 0.    , 0.    , 0.001 ,
                     0.001 , 0.001 , 0.    , 0.    , 0.001 , 0.001 , 0.001 , 0.001 ,
                     0.    , 0.    , 0.    , 0.0005, 0.0005, 0.    , 0.    , 0.0005,
                     0.0005, 0.0005, 0.0005, 0.    , 0.0005, 0.0005, 0.0005, 0.0005,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ,
                     0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    , 0.    ]])
    # fmt: on

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

    def configure(self, ping):
        if "part_number" not in ping.__slots__:
            # backward compatibility
            self.model = "M750d"
        else:
            self.model = OculusProperty.OCULUS_PART_NUMBER[ping.part_number]

        changed = False
        if (
            ping.num_ranges != self.num_ranges
            or ping.range_resolution != self.range_resolution
        ):
            self.num_ranges = ping.num_ranges
            self.range_resolution = ping.range_resolution
            self.ranges = self.range_resolution * (1 + np.arange(self.num_ranges))
            self.range_max = self.ranges[-1]

            self.ro2ra = lambda ro: (ro + 1) * self.range_resolution
            self.ra2ro = lambda ra: np.round(ra / self.range_resolution - 1)
            changed = True

        if len(ping.bearings) != self.num_beams:
            self.num_beams = len(ping.bearings)
            self.bearings = np.deg2rad(np.array(ping.bearings, np.float32) / 100)
            self.horizontal_fov = abs(self.bearings[-1] - self.bearings[0])
            self.angular_resolution = self.horizontal_fov / self.num_beams
            # Use default vertical aperture (mode 1 = 20 degrees)
            self.vertical_fov = OculusProperty.OCULUS_VERTICAL_APERTURE.get(1, np.deg2rad(20))

            self.b2c = interp1d(
                self.bearings,
                np.arange(self.num_beams),
                kind="cubic",
                bounds_error=False,
                fill_value=-1,
                assume_sorted=True,
            )
            self.c2b = interp1d(
                np.arange(self.num_beams),
                self.bearings,
                kind="cubic",
                bounds_error=False,
                fill_value=-1,
                assume_sorted=True,
            )
            changed = True

        if changed:
            height = self.range_max
            rows = self.num_ranges
            width = np.sin((self.bearings[-1] - self.bearings[0]) / 2) * height * 2
            cols = int(np.ceil(width / self.range_resolution))

            XX, YY = np.meshgrid(range(cols), range(rows))
            x = self.range_resolution * (rows - YY)
            y = self.range_resolution * (-cols / 2.0 + XX + 0.5)
            b = np.arctan2(y, x)
            r = np.sqrt(x ** 2 + y ** 2)
            self.remap_y = np.asarray(self.ra2ro(r), dtype=np.float32)
            self.remap_x = np.asarray(self.b2c(b), dtype=np.float32)

        return changed

    def remap(self, ping=None, img=None):
        if img is None:
            img = r2n(ping)
        img = np.array(img, dtype=img.dtype, order="F")

        if self.remap_x.shape[1] > img.shape[1]:
            img.resize(*self.remap_x.shape)
        # Not too much difference between cubic and nearest
        img = cv2.remap(img, self.remap_x, self.remap_y, cv2.INTER_NEAREST)
        return img

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
