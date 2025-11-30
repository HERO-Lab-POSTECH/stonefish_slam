"""
Factor Graph Management Module

This module handles GTSAM factor graph operations, keyframe management,
and Non-Sequential Scan Matching (NSSM) for loop closure detection.

Extracted from slam_legacy.py during refactoring (2025-11-30).
"""

from __future__ import annotations

import gtsam
import numpy as np
from typing import List, Tuple
from itertools import combinations
from collections import defaultdict

from stonefish_slam.core.types import Keyframe, ICPResult
from stonefish_slam.utils.conversions import X


class FactorGraph:
    """Manages GTSAM factor graph, keyframes, and loop closure verification."""

    def __init__(self):
        """Initialize the factor graph manager."""

        # GTSAM components
        self.isam_params = gtsam.ISAM2Params()
        self.isam = gtsam.ISAM2(self.isam_params)
        self.graph = gtsam.NonlinearFactorGraph()
        self.values = gtsam.Values()

        # Keyframe storage (single source of truth)
        self.keyframes: List[Keyframe] = []

        # Loop closure queue for PCM verification
        self.nssm_queue: List[ICPResult] = []

        # Noise models (initialized externally via configure())
        self.prior_model = None
        self.odom_model = None
        self.icp_odom_model = None

        # PCM parameters
        self.pcm_queue_size = 5
        self.min_pcm = 3

    @property
    def current_key(self) -> int:
        """Get the number of keyframes in the graph.

        Returns:
            int: Number of keyframes
        """
        return len(self.keyframes)

    @property
    def current_keyframe(self) -> Keyframe:
        """Get the most recent keyframe.

        Returns:
            Keyframe: The last keyframe in the list
        """
        return self.keyframes[-1]

    def add_keyframe(self, keyframe: Keyframe) -> int:
        """Add a keyframe to the list.

        Args:
            keyframe: Keyframe to add

        Returns:
            int: Index of the added keyframe
        """
        self.keyframes.append(keyframe)
        return len(self.keyframes) - 1

    def get_keyframe(self, key: int) -> Keyframe:
        """Get keyframe by index.

        Args:
            key: Keyframe index

        Returns:
            Keyframe: The requested keyframe
        """
        return self.keyframes[key]

    def set_noise_models(self, prior_model, odom_model, icp_odom_model):
        """Set noise models for factors.

        Args:
            prior_model: Prior factor noise model
            odom_model: Odometry factor noise model
            icp_odom_model: ICP odometry factor noise model
        """
        self.prior_model = prior_model
        self.odom_model = odom_model
        self.icp_odom_model = icp_odom_model

    def add_prior_factor(self, keyframe: Keyframe) -> None:
        """Add prior factor for the first keyframe.

        Args:
            keyframe: Initial keyframe
        """
        pose = keyframe.pose
        factor = gtsam.PriorFactorPose2(X(0), pose, self.prior_model)
        self.graph.add(factor)
        self.values.insert(X(0), pose)

    def add_odometry_factor(self, keyframe: Keyframe) -> None:
        """Add odometry factor between current and previous keyframe.

        Args:
            keyframe: Current keyframe
        """
        # Compute time delta (ROS2: Time message has sec and nanosec fields)
        current_sec = keyframe.time.sec + keyframe.time.nanosec / 1e9
        last_sec = self.keyframes[-1].time.sec + self.keyframes[-1].time.nanosec / 1e9
        dt = current_sec - last_sec

        # Compute odometry from dead reckoning
        dr_odom = self.keyframes[-1].pose.between(keyframe.pose)

        # Add factor
        factor = gtsam.BetweenFactorPose2(
            X(self.current_key - 1),
            X(self.current_key),
            dr_odom,
            self.odom_model
        )
        self.graph.add(factor)
        self.values.insert(X(self.current_key), keyframe.pose)

    def add_icp_factor(
        self,
        source_key: int,
        target_key: int,
        transform: gtsam.Pose2,
        cov: np.ndarray = None
    ) -> None:
        """Add ICP-based constraint factor.

        Args:
            source_key: Source keyframe index
            target_key: Target keyframe index
            transform: Relative transform from ICP
            cov: Covariance matrix (if None, uses default)
        """
        # Select noise model
        if cov is not None:
            noise_model = self.create_full_noise_model(cov)
        else:
            noise_model = self.icp_odom_model

        # Add factor
        factor = gtsam.BetweenFactorPose2(
            X(target_key),
            X(source_key),
            transform,
            noise_model
        )
        self.graph.add(factor)

    def update_graph(self, keyframe: Keyframe = None) -> None:
        """Update ISAM2 with new factors and optimize.

        Args:
            keyframe: Optional keyframe to add before optimization
        """
        # Add new keyframe if provided
        if keyframe:
            self.keyframes.append(keyframe)

        # Push factors to ISAM2
        self.isam.update(self.graph, self.values)
        self.graph.resize(0)
        self.values.clear()

        # Update all keyframe poses
        values = self.isam.calculateEstimate()
        for x in range(values.size()):
            pose = values.atPose2(X(x))
            self.keyframes[x].update(pose)

        # Update latest covariance
        cov = self.isam.marginalCovariance(X(values.size() - 1))
        self.keyframes[-1].update(pose, cov)

        # Update poses in pending loop closures for PCM
        for ret in self.nssm_queue:
            ret.source_pose = self.keyframes[ret.source_key].pose
            ret.target_pose = self.keyframes[ret.target_key].pose
            if ret.inserted:
                ret.estimated_transform = ret.target_pose.between(ret.source_pose)

    def add_loop_closure(self, icp_result: ICPResult) -> None:
        """Add loop closure candidate to NSSM queue.

        Args:
            icp_result: ICP result containing loop closure
        """
        # Update queue (remove old entries)
        while (
            self.nssm_queue
            and icp_result.source_key - self.nssm_queue[0].source_key > self.pcm_queue_size
        ):
            self.nssm_queue.pop(0)

        # Add new candidate
        self.nssm_queue.append(icp_result)

        # Verify PCM
        pcm_indices = self.verify_pcm(self.nssm_queue, self.min_pcm)

        # Add verified loop closures to graph
        for idx in pcm_indices:
            ret = self.nssm_queue[idx]
            if not ret.inserted:
                # Add factor
                self.add_icp_factor(
                    ret.source_key,
                    ret.target_key,
                    ret.estimated_transform,
                    ret.cov
                )

                # Log constraint in keyframe
                self.keyframes[ret.source_key].constraints.append(
                    (ret.target_key, ret.estimated_transform)
                )

                ret.inserted = True

    def verify_pcm(self, queue: List[ICPResult], min_pcm_value: int) -> List[int]:
        """Verify Pairwise Consistent Measurements (PCM).

        Geometric verification for loop closures using consistency graph.

        Args:
            queue: List of loop closure candidates
            min_pcm_value: Minimum number of consistent measurements

        Returns:
            List of indices in queue that form maximum clique
        """
        if len(queue) < min_pcm_value:
            return []

        # Build consistency graph
        G = defaultdict(list)
        for (a, ret_il), (b, ret_jk) in combinations(zip(range(len(queue)), queue), 2):
            pi = ret_il.target_pose
            pj = ret_jk.target_pose
            pil = ret_il.estimated_transform
            plk = ret_il.source_pose.between(ret_jk.source_pose)
            pjk1 = ret_jk.estimated_transform
            pjk2 = pj.between(pi.compose(pil).compose(plk))

            # Compute Mahalanobis distance
            error = gtsam.Pose2.Logmap(pjk1.between(pjk2))
            md = error.dot(np.linalg.inv(ret_jk.cov)).dot(error)

            # chi2.ppf(0.99, 3) = 11.34
            if md < 11.34:
                G[a].append(b)
                G[b].append(a)

        # Find maximal cliques
        maximal_cliques = list(self.find_cliques(G))

        if not maximal_cliques:
            return []

        # Return largest clique if it meets minimum size
        maximum_clique = sorted(maximal_cliques, key=len, reverse=True)[0]
        if len(maximum_clique) < min_pcm_value:
            return []

        return maximum_clique

    def find_cliques(self, G: defaultdict):
        """Find all maximal cliques in undirected graph.

        Bron-Kerbosch algorithm implementation.

        Args:
            G: Adjacency list representation of graph

        Yields:
            List of nodes forming a maximal clique
        """
        if len(G) == 0:
            return

        adj = {u: {v for v in G[u] if v != u} for u in G}
        Q = [None]

        subg = set(G)
        cand = set(G)
        u = max(subg, key=lambda u: len(cand & adj[u]))
        ext_u = cand - adj[u]
        stack = []

        try:
            while True:
                if ext_u:
                    q = ext_u.pop()
                    cand.remove(q)
                    Q[-1] = q
                    adj_q = adj[q]
                    subg_q = subg & adj_q
                    if not subg_q:
                        yield Q[:]
                    else:
                        cand_q = cand & adj_q
                        if cand_q:
                            stack.append((subg, cand, ext_u))
                            Q.append(None)
                            subg = subg_q
                            cand = cand_q
                            u = max(subg, key=lambda u: len(cand & adj[u]))
                            ext_u = cand - adj[u]
                else:
                    Q.pop()
                    subg, cand, ext_u = stack.pop()
        except IndexError:
            pass

    def get_states(self) -> np.ndarray:
        """Retrieve all states as structured array.

        Returns:
            Structured numpy array with fields:
            - time: timestamp
            - pose: [x, y, yaw]
            - dr_pose3: [x, y, z, roll, pitch, yaw]
            - cov: flattened 3x3 covariance
        """
        states = np.zeros(
            self.current_key,
            dtype=[
                ("time", np.float64),
                ("pose", np.float32, 3),
                ("dr_pose3", np.float32, 6),
                ("cov", np.float32, 9),
            ],
        )

        # Update all keyframes with latest estimates
        values = self.isam.calculateEstimate()
        for key in range(self.current_key):
            pose = values.atPose2(X(key))
            cov = self.isam.marginalCovariance(X(key))
            self.keyframes[key].update(pose, cov)

        # Extract state information
        from stonefish_slam.utils.conversions import g2n
        t_zero = self.keyframes[0].time
        for key in range(self.current_key):
            keyframe = self.keyframes[key]
            t_zero_sec = t_zero.sec + t_zero.nanosec / 1e9
            keyframe_sec = keyframe.time.sec + keyframe.time.nanosec / 1e9
            states[key]["time"] = keyframe_sec - t_zero_sec
            states[key]["pose"] = g2n(keyframe.pose)
            states[key]["dr_pose3"] = g2n(keyframe.dr_pose3)
            states[key]["cov"] = keyframe.transf_cov.ravel()

        return states

    @staticmethod
    def sample_pose(pose: gtsam.Pose2, covariance: np.ndarray) -> gtsam.Pose2:
        """Generate random pose sample from covariance.

        Args:
            pose: Mean pose
            covariance: 3x3 covariance matrix

        Returns:
            Sampled pose
        """
        from stonefish_slam.utils.conversions import n2g
        delta = np.random.multivariate_normal(np.zeros(3), covariance)
        return pose.compose(n2g(delta, "Pose2"))

    @staticmethod
    def create_full_noise_model(cov: np.ndarray) -> gtsam.noiseModel.Gaussian:
        """Create GTSAM noise model from covariance matrix.

        Args:
            cov: Covariance matrix

        Returns:
            GTSAM Gaussian noise model
        """
        return gtsam.noiseModel.Gaussian.Covariance(cov)

    @staticmethod
    def create_robust_full_noise_model(cov: np.ndarray) -> gtsam.noiseModel.Robust:
        """Create robust GTSAM noise model from covariance matrix.

        Args:
            cov: Covariance matrix

        Returns:
            GTSAM robust noise model with Cauchy kernel
        """
        model = gtsam.noiseModel.Gaussian.Covariance(cov)
        robust = gtsam.noiseModel.mEstimator.Cauchy.Create(1.0)
        return gtsam.noiseModel.Robust.Create(robust, model)
