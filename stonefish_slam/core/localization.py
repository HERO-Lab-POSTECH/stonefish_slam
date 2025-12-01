"""
Localization Module

This module handles ICP-based scan matching, sequential scan matching (SSM),
non-sequential scan matching (NSSM) initialization, and keyframe detection.

Extracted from slam_legacy.py during refactoring (2025-11-30).
"""

from __future__ import annotations

import cv2
import gtsam
import numpy as np
import time as time_pkg
from typing import Tuple, List, Union as TypingUnion
from scipy.optimize import shgo
from sklearn.covariance import MinCovDet

from stonefish_slam.core.types import (
    STATUS,
    Keyframe,
    InitializationResult,
    ICPResult,
    SMParams,
)
from stonefish_slam.core.factor_graph import FactorGraph
from stonefish_slam.cpp import pcl
from stonefish_slam.utils.sonar import OculusProperty
from stonefish_slam.utils.conversions import g2n, n2g, pose322, X
from stonefish_slam.utils.io import CodeTimer


class Localization:
    """Handles ICP-based localization and scan matching."""

    def __init__(self, factor_graph: FactorGraph):
        """Initialize localization module.

        Args:
            factor_graph: Reference to FactorGraph instance
        """
        # Dependency injection
        self.fg = factor_graph

        # Sonar configuration
        self.oculus = OculusProperty()

        # ICP instances
        self.icp = pcl.ICP()
        self.icp_ssm = pcl.ICP()

        # Keyframe criteria
        self.keyframe_duration = None
        self.keyframe_translation = None
        self.keyframe_rotation = None

        # Current (non-key) frame
        self.current_frame: Keyframe = None

        # Point cloud parameters
        self.point_resolution = 0.5
        self.point_noise = 0.5

        # Scan matching parameters
        self.ssm_params = SMParams()
        self.ssm_params.initialization = True
        self.ssm_params.initialization_params = 50, 1, 0.01
        self.ssm_params.min_st_sep = 1
        self.ssm_params.min_points = 50
        self.ssm_params.max_translation = 2.0
        self.ssm_params.max_rotation = np.pi / 6
        self.ssm_params.target_frames = 3
        self.ssm_params.cov_samples = 0
        self.ssm_params.enable = True

        self.nssm_params = SMParams()
        self.nssm_params.initialization = True
        self.nssm_params.initialization_params = 100, 5, 0.01
        self.nssm_params.min_st_sep = 10
        self.nssm_params.min_points = 100
        self.nssm_params.max_translation = 6.0
        self.nssm_params.max_rotation = np.pi / 2
        self.nssm_params.source_frames = 5
        self.nssm_params.cov_samples = 30

        # Noise model sigmas (set externally)
        self.odom_sigmas = None
        self.icp_odom_sigmas = None

        # Debug flags
        self.save_fig = False
        self.save_data = False

    def is_keyframe(self, frame: Keyframe) -> bool:
        """Determine if a frame should be a keyframe.

        Criteria:
        - First frame is always keyframe
        - Reject if angular velocity too high (motion blur)
        - Must exceed minimum duration
        - Must exceed translation OR rotation threshold

        Args:
            frame: Candidate keyframe

        Returns:
            True if frame should be added as keyframe
        """
        # First frame is always a keyframe
        if not self.fg.keyframes:
            return True

        # Reject if angular velocity too high (motion blur prevention)
        if frame.twist is not None:
            angular_vel_z = abs(frame.twist.angular.z)
            max_angular_vel = 0.1  # rad/s (~6°/s)
            if angular_vel_z > max_angular_vel:
                return False

        # Check time duration (ROS2: Time has sec and nanosec fields)
        frame_time_ns = frame.time.sec * 1e9 + frame.time.nanosec
        current_time_ns = self.fg.current_keyframe.time.sec * 1e9 + self.fg.current_keyframe.time.nanosec
        duration_ns = frame_time_ns - current_time_ns
        keyframe_duration_ns = self.keyframe_duration.nanoseconds
        if duration_ns < keyframe_duration_ns:
            return False

        # Check translation and rotation
        dr_odom = self.fg.keyframes[-1].dr_pose.between(frame.dr_pose)
        translation = np.linalg.norm(dr_odom.translation())
        rotation = abs(dr_odom.theta())

        return (
            translation > self.keyframe_translation
            or rotation > self.keyframe_rotation
        )

    def compute_icp(
        self,
        source_points: np.ndarray,
        target_points: np.ndarray,
        guess: gtsam.Pose2 = gtsam.Pose2(),
    ) -> Tuple[str, gtsam.Pose2]:
        """Compute standard ICP alignment.

        Args:
            source_points: Source point cloud [N x 2]
            target_points: Target point cloud [M x 2]
            guess: Initial transformation guess

        Returns:
            Tuple of (status_message, estimated_transform)
        """
        source_points = np.array(source_points, np.float32)
        target_points = np.array(target_points, np.float32)

        guess_matrix = guess.matrix()
        message, T = self.icp.compute(source_points, target_points, guess_matrix)

        # Parse result
        x, y = T[:2, 2]
        theta = np.arctan2(T[1, 0], T[0, 0])

        return message, gtsam.Pose2(x, y, theta)

    def compute_icp_with_cov(
        self,
        source_points: np.ndarray,
        target_points: np.ndarray,
        guesses: List[gtsam.Pose2],
    ) -> Tuple[str, gtsam.Pose2, np.ndarray, np.ndarray]:
        """Compute ICP with covariance estimation.

        Uses multiple initial guesses and robust covariance estimation.

        Args:
            source_points: Source point cloud
            target_points: Target point cloud
            guesses: List of initial transformation guesses

        Returns:
            Tuple of (message, transform, covariance, sample_transforms)
        """
        source_points = np.array(source_points, np.float32)
        target_points = np.array(target_points, np.float32)

        sample_transforms = []
        start = time_pkg.time()

        for g in guesses:
            g_matrix = g.matrix()
            message, T = self.icp.compute(source_points, target_points, g_matrix)

            if message == "success":
                x, y = T[:2, 2]
                theta = np.arctan2(T[1, 0], T[0, 0])
                sample_transforms.append((x, y, theta))

            # Enforce maximum runtime (2 seconds)
            if time_pkg.time() - start >= 2.0:
                break

        sample_transforms = np.array(sample_transforms)
        if len(sample_transforms) < 5:
            return "Too few samples for covariance computation", None, None, None

        # Robust covariance estimation (handles outliers)
        try:
            fcov = MinCovDet(store_precision=False, support_fraction=0.8).fit(
                sample_transforms
            )
        except ValueError:
            return "Failed to calculate covariance", None, None, None

        # Extract mean and covariance
        m = n2g(fcov.location_, "Pose2")
        cov = fcov.covariance_

        # Unrotate covariance to local frame
        R = m.rotation().matrix()
        cov[:2, :] = R.T.dot(cov[:2, :])
        cov[:, :2] = cov[:, :2].dot(R)

        # Ensure covariance not smaller than default
        default_cov = np.diag(self.icp_odom_sigmas) ** 2
        if np.linalg.det(cov) < np.linalg.det(default_cov):
            cov = default_cov

        return "success", m, cov, sample_transforms

    def get_overlap(
        self,
        source_points: np.ndarray,
        target_points: np.ndarray,
        source_pose: gtsam.Pose2 = None,
        target_pose: gtsam.Pose2 = None,
        return_indices: bool = False,
    ) -> TypingUnion[int, Tuple[int, np.ndarray]]:
        """Compute overlap between point clouds.

        Args:
            source_points: Source point cloud
            target_points: Target point cloud
            source_pose: Optional source transformation
            target_pose: Optional target transformation
            return_indices: If True, return matched indices

        Returns:
            Number of overlapping points, or (count, indices) if return_indices=True
        """
        if source_pose:
            source_points = Keyframe.transform_points(source_points, source_pose)
        if target_pose:
            target_points = Keyframe.transform_points(target_points, target_pose)

        indices, dists = pcl.match(target_points, source_points, 1, self.point_noise)

        if return_indices:
            return np.sum(indices != -1), indices
        else:
            return np.sum(indices != -1)

    def get_matching_cost_subroutine1(
        self,
        source_points: np.ndarray,
        source_pose: gtsam.Pose2,
        target_points: np.ndarray,
        target_pose: gtsam.Pose2,
        source_pose_cov: np.ndarray = None,
    ) -> Tuple[callable, list]:
        """Create cost function for global scan matching.

        Grid-based matching cost computation for initialization.

        Args:
            source_points: Source point cloud
            source_pose: Source pose
            target_points: Target point cloud
            target_pose: Target pose
            source_pose_cov: Source covariance

        Returns:
            Tuple of (optimization_function, pose_samples_list)
        """
        pose_samples = []

        # Create grid for target points
        xmin, ymin = np.min(target_points, axis=0) - 2 * self.point_noise
        xmax, ymax = np.max(target_points, axis=0) + 2 * self.point_noise
        resolution = self.point_noise / 10.0
        xs = np.arange(xmin, xmax, resolution)
        ys = np.arange(ymin, ymax, resolution)
        target_grids = np.zeros((len(ys), len(xs)), np.uint8)

        # Populate grid
        r = np.int32(np.round((target_points[:, 1] - ymin) / resolution))
        c = np.int32(np.round((target_points[:, 0] - xmin) / resolution))
        r = np.clip(r, 0, target_grids.shape[0] - 1)
        c = np.clip(c, 0, target_grids.shape[1] - 1)
        target_grids[r, c] = 255

        # Dilate grid to account for point noise
        dilate_hs = int(np.ceil(self.point_noise / resolution))
        dilate_size = 2 * dilate_hs + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (dilate_size, dilate_size), (dilate_hs, dilate_hs)
        )
        target_grids = cv2.dilate(target_grids, kernel)

        source_pose_info = np.linalg.inv(source_pose_cov) if source_pose_cov is not None else None

        def subroutine(x: np.ndarray) -> float:
            """Optimization subroutine for scipy.shgo.

            Args:
                x: Pose delta [dx, dy, dtheta]

            Returns:
                Matching cost (negative overlap count)
            """
            delta = n2g(x, "Pose2")
            sample_source_pose = source_pose.compose(delta)
            sample_transform = target_pose.between(sample_source_pose)

            points = Keyframe.transform_points(source_points, sample_transform)
            r = np.int32(np.round((points[:, 1] - ymin) / resolution))
            c = np.int32(np.round((points[:, 0] - xmin) / resolution))
            inside = (
                (0 <= r)
                & (r < target_grids.shape[0])
                & (0 <= c)
                & (c < target_grids.shape[1])
            )

            cost = -np.sum(target_grids[r[inside], c[inside]] > 0)
            pose_samples.append(np.r_[g2n(sample_source_pose), cost])

            return cost

        return subroutine, pose_samples

    def initialize_sequential_scan_matching(
        self, keyframe: Keyframe
    ) -> InitializationResult:
        """Initialize sequential scan matching with global optimization.

        Args:
            keyframe: Current keyframe to match

        Returns:
            Initialization result with global ICP estimate
        """
        ret = InitializationResult()
        ret.status = STATUS.SUCCESS
        ret.status.description = None

        # Setup source and target
        ret.source_key = self.fg.current_key
        ret.target_key = self.fg.current_key - 1
        ret.source_pose = keyframe.pose
        ret.target_pose = self.fg.current_keyframe.pose

        # Accumulate points from previous k frames
        ret.source_points = keyframe.points
        target_frames = range(self.fg.current_key)[-self.ssm_params.target_frames:]
        ret.target_points = self.get_points(target_frames, ret.target_key)
        ret.cov = np.diag(self.odom_sigmas)

        # Check if SSM is enabled
        if not self.ssm_params.enable:
            ret.status = STATUS.NOT_ENOUGH_POINTS
            ret.status.description = f"SSM disabled"
            return ret

        # Validate source points
        if len(ret.source_points) < self.ssm_params.min_points:
            ret.status = STATUS.NOT_ENOUGH_POINTS
            ret.status.description = f"source points {len(ret.source_points)}"
            return ret

        # Validate target points
        if len(ret.target_points) < self.ssm_params.min_points:
            ret.status = STATUS.NOT_ENOUGH_POINTS
            ret.status.description = f"target points {len(ret.target_points)}"
            return ret

        # Global initialization
        if not self.ssm_params.initialization:
            return ret

        with CodeTimer("SLAM - sequential scan matching - sampling"):
            # Define search bounds
            pose_stds = np.array([self.odom_sigmas]).T
            pose_bounds = 5.0 * np.c_[-pose_stds, pose_stds]

            # Create cost function
            subroutine, pose_samples = self.get_matching_cost_subroutine1(
                ret.source_points,
                ret.source_pose,
                ret.target_points,
                ret.target_pose,
                ret.cov,
            )

            # Optimize with scipy.shgo
            result = shgo(
                func=subroutine,
                bounds=pose_bounds,
                n=self.ssm_params.initialization_params[0],
                iters=self.ssm_params.initialization_params[1],
                sampling_method="sobol",
                minimizer_kwargs={
                    "options": {"ftol": self.ssm_params.initialization_params[2]}
                },
            )

        if result.success:
            ret.source_pose_samples = np.array(pose_samples)
            ret.estimated_source_pose = ret.source_pose.compose(n2g(result.x, "Pose2"))
            ret.status.description = f"matching cost {result.fun:.2f}"

            if self.save_data:
                ret.save(f"step-{self.fg.current_key}-ssm-sampling.npz")
        else:
            ret.status = STATUS.INITIALIZATION_FAILURE
            ret.status.description = result.message

        return ret

    def initialize_nonsequential_scan_matching(self) -> InitializationResult:
        """Initialize non-sequential scan matching (loop closure detection).

        Returns:
            Initialization result with potential loop closure
        """
        ret = InitializationResult()
        ret.status = STATUS.SUCCESS
        ret.status.description = None

        # Setup source
        ret.source_key = self.fg.current_key - 1
        ret.source_pose = self.current_frame.pose
        ret.estimated_source_pose = ret.source_pose

        # Aggregate source cloud from k recent frames
        source_frames = range(
            ret.source_key, ret.source_key - self.nssm_params.source_frames, -1
        )
        ret.source_points = self.get_points(source_frames, ret.source_key)

        # Validate source points
        if len(ret.source_points) < self.nssm_params.min_points:
            ret.status = STATUS.NOT_ENOUGH_POINTS
            ret.status.description = f"source points {len(ret.source_points)}"
            return ret

        # Find target points (all frames except recent k)
        target_frames = range(self.fg.current_key - self.nssm_params.min_st_sep)
        target_points, target_keys = self.get_points(target_frames, None, True)

        # Filter by field of view
        sel = np.zeros(len(target_points), bool)
        for source_frame in source_frames:
            pose = self.fg.keyframes[source_frame].pose
            cov = self.fg.keyframes[source_frame].cov

            translation_std = np.sqrt(np.max(np.linalg.eigvals(cov[:2, :2])))
            rotation_std = np.sqrt(cov[2, 2])
            range_bound = translation_std * 5.0 + self.oculus.range_max
            bearing_bound = rotation_std * 5.0 + self.oculus.horizontal_fov * 0.5

            local_points = Keyframe.transform_points(target_points, pose.inverse())
            ranges = np.linalg.norm(local_points, axis=1)
            bearings = np.arctan2(local_points[:, 1], local_points[:, 0])
            sel_i = (ranges < range_bound) & (abs(bearings) < bearing_bound)
            sel |= sel_i

        target_points = target_points[sel]
        target_keys = target_keys[sel]

        # Find frame with most overlapping points
        target_frames, counts = np.unique(np.int32(target_keys), return_counts=True)
        target_frames = target_frames[counts > 10]
        counts = counts[counts > 10]

        if len(target_frames) == 0 or len(target_points) < self.nssm_params.min_points:
            ret.status = STATUS.NOT_ENOUGH_POINTS
            ret.status.description = f"target points {len(target_points)}"
            return ret

        # Select target with maximum overlap
        ret.target_key = target_frames[np.argmax(counts)]
        ret.target_pose = self.fg.keyframes[ret.target_key].pose
        ret.target_points = Keyframe.transform_points(
            target_points, ret.target_pose.inverse()
        )
        ret.cov = self.fg.keyframes[ret.source_key].cov

        # Global initialization
        if not self.nssm_params.initialization:
            return ret

        with CodeTimer("SLAM - nonsequential scan matching - sampling"):
            translation_std = np.sqrt(np.max(np.linalg.eigvals(cov[:2, :2])))
            rotation_std = np.sqrt(cov[2, 2])
            pose_stds = np.array([[translation_std, translation_std, rotation_std]]).T
            pose_bounds = 5.0 * np.c_[-pose_stds, pose_stds]

            subroutine, pose_samples = self.get_matching_cost_subroutine1(
                ret.source_points,
                ret.source_pose,
                ret.target_points,
                ret.target_pose,
                ret.cov,
            )

            result = shgo(
                func=subroutine,
                bounds=pose_bounds,
                n=self.nssm_params.initialization_params[0],
                iters=self.nssm_params.initialization_params[1],
                sampling_method="sobol",
                minimizer_kwargs={
                    "options": {"ftol": self.nssm_params.initialization_params[2]}
                },
            )

        if not result.success:
            ret.status = STATUS.INITIALIZATION_FAILURE
            ret.status.description = result.message
            return ret

        delta = n2g(result.x, "Pose2")
        ret.estimated_source_pose = ret.source_pose.compose(delta)
        ret.source_pose_samples = np.array(pose_samples)
        ret.status.description = f"matching cost {result.fun:.2f}"

        # Refine target key by maximum overlap
        estimated_source_points = Keyframe.transform_points(
            ret.source_points, ret.estimated_source_pose
        )
        overlap, indices = self.get_overlap(
            estimated_source_points, target_points, return_indices=True
        )
        target_frames1, counts1 = np.unique(
            np.int32(target_keys[indices[indices != -1]]), return_counts=True
        )
        if len(counts1) == 0:
            ret.status = STATUS.NOT_ENOUGH_OVERLAP
            ret.status.description = "0"
            return ret

        if self.save_data:
            ret.save(f"step-{self.fg.current_key - 1}-nssm-sampling.npz")

        # Update target key with refined estimate
        ret.target_key = target_frames1[np.argmax(counts1)]
        ret.target_pose = self.fg.keyframes[ret.target_key].pose
        ret.target_points = self.get_points(target_frames, ret.target_key)

        return ret

    def get_points(
        self, frames: list = None, ref_frame: TypingUnion[int, gtsam.Pose2] = None, return_keys: bool = False
    ) -> np.ndarray:
        """Get accumulated point cloud from keyframes.

        Args:
            frames: List of keyframe indices (None = all)
            ref_frame: Reference frame (int index or gtsam.Pose2)
            return_keys: If True, append keyframe index to each point

        Returns:
            Point cloud array [N x 2] or [N x 3] if return_keys=True
        """
        if frames is None:
            frames = range(self.fg.current_key)

        if ref_frame is not None:
            if isinstance(ref_frame, gtsam.Pose2):
                ref_pose = ref_frame
            else:
                ref_pose = self.fg.keyframes[ref_frame].pose

        if return_keys:
            all_points = [np.zeros((0, 3), np.float32)]
        else:
            all_points = [np.zeros((0, 2), np.float32)]

        for key in frames:
            if ref_frame is not None:
                points = self.fg.keyframes[key].points
                pose = self.fg.keyframes[key].pose
                transf = ref_pose.between(pose)
                transf_points = Keyframe.transform_points(points, transf)
            else:
                transf_points = self.fg.keyframes[key].transf_points

            if return_keys:
                transf_points = np.c_[
                    transf_points, key * np.ones((len(transf_points), 1))
                ]
            all_points.append(transf_points)

        all_points = np.concatenate(all_points)

        if return_keys:
            return pcl.downsample(
                all_points[:, :2], all_points[:, (2,)], self.point_resolution
            )
        else:
            return pcl.downsample(all_points, self.point_resolution)
