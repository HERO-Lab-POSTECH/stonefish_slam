#!/usr/bin/env python3
"""
중앙 dip 문제 진단 스크립트
bearing=0° vs bearing=65°에서 3D 좌표 분포를 비교
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge


class CenterDipDiagnostic(Node):
    def __init__(self):
        super().__init__('center_dip_diagnostic')

        # Sonar parameters (from config)
        self.horizontal_fov = np.radians(130.0)  # 130°
        self.vertical_fov = np.radians(20.0)     # 20°
        self.num_beams = 512
        self.num_bins = 500
        self.range_min = 0.5
        self.range_max = 20.0
        self.sonar_tilt = np.radians(45.0)       # 45° down

        self.bridge = CvBridge()
        self.sonar_image = None
        self.collected_data = []

        # Subscribers
        self.sonar_sub = self.create_subscription(
            Image, '/bluerov2/fls/image', self.sonar_callback, 10)

        self.get_logger().info('Center dip diagnostic started. Waiting for sonar data...')

        # Timer for analysis
        self.timer = self.create_timer(5.0, self.analyze_callback)

    def sonar_callback(self, msg):
        self.sonar_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

    def analyze_callback(self):
        if self.sonar_image is None:
            self.get_logger().warn('No sonar image received yet')
            return

        self.get_logger().info('Analyzing sonar image...')

        # Bearings to compare
        bearings_deg = [0, 15, 30, 45, 60, 65]

        results = {}

        for bearing_deg in bearings_deg:
            bearing_rad = np.radians(bearing_deg)

            # Find column index for this bearing
            # np.linspace(-fov/2, +fov/2, num_beams)
            col_idx = int((bearing_deg + 65) / 130 * (self.num_beams - 1))
            col_idx = np.clip(col_idx, 0, self.num_beams - 1)

            # Get intensity column
            intensity_col = self.sonar_image[:, col_idx].astype(np.float32)

            # Find first hit (threshold)
            threshold = 30
            hit_indices = np.where(intensity_col > threshold)[0]

            if len(hit_indices) == 0:
                self.get_logger().warn(f'No hit found at bearing={bearing_deg}°')
                continue

            first_hit_row = hit_indices[0]

            # Convert row to range (FLS image: row 0 = far, row max = near)
            range_m = self.range_max - (first_hit_row / self.num_bins) * (self.range_max - self.range_min)

            # Compute 3D points for different elevation samples
            half_aperture = self.vertical_fov / 2
            num_v_samples = 5  # Sample 5 elevations

            points_3d = []
            for v_idx in range(num_v_samples):
                # Elevation relative to tilt axis
                v_ratio = (v_idx - (num_v_samples - 1) / 2) / ((num_v_samples - 1) / 2)
                elevation_rel = v_ratio * half_aperture

                # Actual elevation from horizontal (tilt + relative)
                elevation_abs = self.sonar_tilt + elevation_rel

                # 3D point in sonar frame (before tilt)
                # x = r * cos(elev_rel) * cos(bearing)
                # y = r * cos(elev_rel) * sin(bearing)
                # z = r * sin(elev_rel)
                x_sonar = range_m * np.cos(elevation_rel) * np.cos(bearing_rad)
                y_sonar = range_m * np.cos(elevation_rel) * np.sin(bearing_rad)
                z_sonar = range_m * np.sin(elevation_rel)

                # Apply sonar tilt (rotate around Y axis by -tilt)
                # After tilt, the sonar X points forward-down at 45°
                cos_tilt = np.cos(self.sonar_tilt)
                sin_tilt = np.sin(self.sonar_tilt)

                x_body = cos_tilt * x_sonar + sin_tilt * z_sonar
                y_body = y_sonar
                z_body = -sin_tilt * x_sonar + cos_tilt * z_sonar

                points_3d.append({
                    'elevation_rel_deg': np.degrees(elevation_rel),
                    'elevation_abs_deg': np.degrees(elevation_abs),
                    'x': x_body,
                    'y': y_body,
                    'z': z_body
                })

            results[bearing_deg] = {
                'col_idx': col_idx,
                'first_hit_row': first_hit_row,
                'range_m': range_m,
                'intensity': intensity_col[first_hit_row],
                'points_3d': points_3d
            }

        # Print comparison
        self.get_logger().info('\n' + '='*80)
        self.get_logger().info('CENTER DIP DIAGNOSTIC RESULTS')
        self.get_logger().info('='*80)

        for bearing_deg, data in results.items():
            self.get_logger().info(f'\nBearing = {bearing_deg}°:')
            self.get_logger().info(f'  Column index: {data["col_idx"]}')
            self.get_logger().info(f'  First hit row: {data["first_hit_row"]}')
            self.get_logger().info(f'  Range: {data["range_m"]:.2f} m')
            self.get_logger().info(f'  Intensity: {data["intensity"]:.0f}')
            self.get_logger().info(f'  3D Points (body frame):')
            for pt in data['points_3d']:
                self.get_logger().info(
                    f'    elev_rel={pt["elevation_rel_deg"]:+6.1f}° → '
                    f'X={pt["x"]:+6.2f}, Y={pt["y"]:+6.2f}, Z={pt["z"]:+6.2f}'
                )

        # Compare Z values at center vs edges
        if 0 in results and 65 in results:
            self.get_logger().info('\n' + '-'*80)
            self.get_logger().info('COMPARISON: Center (0°) vs Edge (65°)')
            self.get_logger().info('-'*80)

            center_z = [pt['z'] for pt in results[0]['points_3d']]
            edge_z = [pt['z'] for pt in results[65]['points_3d']]

            self.get_logger().info(f'Center (0°) Z range: {min(center_z):.2f} to {max(center_z):.2f}')
            self.get_logger().info(f'Edge (65°) Z range: {min(edge_z):.2f} to {max(edge_z):.2f}')
            self.get_logger().info(f'Center range: {results[0]["range_m"]:.2f} m')
            self.get_logger().info(f'Edge range: {results[65]["range_m"]:.2f} m')

            # Check if ranges are different
            range_diff = results[0]["range_m"] - results[65]["range_m"]
            self.get_logger().info(f'\nRange difference (center - edge): {range_diff:+.3f} m')

            if abs(range_diff) > 0.1:
                self.get_logger().warn('Range values differ significantly!')
                self.get_logger().warn('This suggests the floor is NOT flat, OR')
                self.get_logger().warn('Stonefish is returning different ranges for different bearings.')

        self.get_logger().info('\nDiagnostic complete. Shutting down...')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = CenterDipDiagnostic()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
