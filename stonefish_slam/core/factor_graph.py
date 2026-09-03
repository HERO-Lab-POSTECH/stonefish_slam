"""
Factor Graph Management Module

This module handles GTSAM factor graph operations, keyframe management,
and Non-Sequential Scan Matching (NSSM) for loop closure detection.

Extracted from slam_legacy.py during refactoring (2025-11-30).
"""

from __future__ import annotations

import logging

import gtsam
import numpy as np
from typing import List
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

        # How many trailing keyframes get their pose refreshed from ISAM2 each
        # tick. <= 0 refreshes the whole history (O(N) per update).
        self.pose_refresh_window = 10

        # 계측 (I7) — PCM 이 실제로 그래프에 넣은 루프 팩터 수. 판정에 쓰이지 않는다.
        self.pcm_inserted_count = 0

        # Robust cost parameter for loop closure (NSSM) factors only.
        # Cauchy kernel: c=3.0 means "down-weight at ~3σ" (conservative).
        self.robust_loop_c: float = 3.0

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
        cov: np.ndarray = None,
        robust: bool = False,
    ) -> None:
        """Add ICP-based constraint factor.

        Args:
            source_key: Source keyframe index
            target_key: Target keyframe index
            transform: Relative transform from ICP
            cov: Covariance matrix (if None, uses default)
            robust: If True, wrap noise model with Cauchy robust kernel.
                    Should be True only for loop closure (NSSM) factors.
        """
        # Select noise model
        if cov is not None:
            if robust:
                noise_model = self.create_robust_full_noise_model(cov)
            else:
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

    def _trim_nssm_queue(self, current_key: int) -> None:
        """Drop loop-closure candidates older than the PCM window.

        Args:
            current_key: Keyframe index the window is measured back from
        """
        while (
            self.nssm_queue
            and current_key - self.nssm_queue[0].source_key > self.pcm_queue_size
        ):
            self.nssm_queue.pop(0)

    def _default_covariance(self) -> np.ndarray:
        """Covariance to stand in for a marginal GTSAM could not compute.

        Returns:
            np.ndarray: 3x3 covariance from the odometry noise model, or a
            conservative identity when noise models have not been set yet.
        """
        if self.odom_model is not None:
            return np.asarray(self.odom_model.covariance())
        return np.eye(3)

    def update_graph(self, keyframe: Keyframe = None) -> None:
        """Update ISAM2 with new factors and optimize.

        Args:
            keyframe: Optional keyframe to add before optimization
        """
        # Add new keyframe if provided
        if keyframe:
            self.keyframes.append(keyframe)

        # Push factors to ISAM2.
        #
        # A degenerate linearization makes GTSAM raise (IndeterminantLinear-
        # SystemException surfaces as RuntimeError through the pybind layer),
        # which would otherwise kill the node mid-mission. Catching it is only
        # half the fix: the pending graph/values MUST still be cleared, or the
        # offending factors stay queued and every later keyframe re-pushes them,
        # leaving ISAM2 permanently failing for the rest of the node's life.
        # Hence the finally — the clear happens on both paths.
        try:
            self.isam.update(self.graph, self.values)
        except RuntimeError as e:
            logging.error("[FactorGraph] ISAM2 update failed, skipping this tick "
                          "(pending factors dropped): %s", e)
            # The keyframe was appended above and keeps its dead-reckoning pose.
            # Give it a covariance too — verify_pcm and add_icp_factor both have
            # None guards, but leaving None here would silently degrade every
            # loop closure the frame takes part in.
            if self.keyframes and self.keyframes[-1].cov is None:
                self.keyframes[-1].cov = self._default_covariance()
            return
        finally:
            self.graph.resize(0)
            self.values.clear()

        # Drop loop-closure candidates that have aged out of the PCM window.
        # Trimming used to happen only when a NEW candidate arrived, so once the
        # vehicle left a loopy area the queue never emptied and the pose refresh
        # below stayed pinned to the O(N) full-history path for the rest of the
        # mission — exactly the cost the window was introduced to avoid.
        self._trim_nssm_queue(self.current_key)

        # Update keyframe poses — only the most recent window each tick, unless
        # a pending loop closure (nssm_queue) may have moved older poses.
        # Full-history refresh is O(N) per update and was measured dominating
        # the callback on long real-data runs; ISAM2 rarely moves old poses
        # without loop closures. pose_refresh_window <= 0 restores the
        # full-history refresh for consumers that need always-fresh poses.
        values = self.isam.calculateEstimate()
        n = values.size()
        window = self.pose_refresh_window
        start_idx = 0 if (window <= 0 or self.nssm_queue) else max(0, n - window)
        for x in range(start_idx, n):
            pose = values.atPose2(X(x))
            self.keyframes[x].update(pose)

        # Update latest covariance. A marginal can be indeterminate even when
        # the update above succeeded; leaving cov as None then crashes
        # verify_pcm's np.linalg.solve later, so substitute the odometry noise
        # model's own covariance rather than propagating a None.
        try:
            cov = self.isam.marginalCovariance(X(n - 1))
        except (RuntimeError, IndexError) as e:
            # IndexError is not redundant: after a failed update the variable is
            # in theta_ but may never have been eliminated, and GTSAM then
            # raises "Requested the BayesTree clique for a key that is not in
            # the BayesTree" rather than the indeterminate-system RuntimeError.
            cov = self._default_covariance()
            logging.warning("[FactorGraph] marginalCovariance(X(%d)) failed, using the "
                            "odometry noise model instead: %s", n - 1, e)
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
        self._trim_nssm_queue(icp_result.source_key)

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
                    ret.cov,
                    robust=True,
                )

                # Log constraint in keyframe
                self.keyframes[ret.source_key].constraints.append(
                    (ret.target_key, ret.estimated_transform)
                )

                ret.inserted = True
                self.pcm_inserted_count += 1

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
            # Same guard add_icp_factor already has: cov is None whenever the
            # ICP path was configured to skip covariance sampling
            # (nssm.cov_samples == 0). Without an uncertainty there is no
            # Mahalanobis distance to compare, so claim no consistency for this
            # pair rather than letting np.linalg.solve raise LinAlgError.
            if ret_jk.cov is None:
                continue

            pi = ret_il.target_pose
            pj = ret_jk.target_pose
            pil = ret_il.estimated_transform
            plk = ret_il.source_pose.between(ret_jk.source_pose)
            pjk1 = ret_jk.estimated_transform
            pjk2 = pj.between(pi.compose(pil).compose(plk))

            # Squared Mahalanobis distance via solve (numerically stabler than
            # forming the explicit inverse; identical on well-conditioned cov).
            error = gtsam.Pose2.Logmap(pjk1.between(pjk2))
            md = error.dot(np.linalg.solve(ret_jk.cov, error))

            # PCM consistency gate. md is the SQUARED Mahalanobis distance, so it
            # is compared against a chi2 quantile directly:
            #   chi2.ppf(0.99, 3) = 11.34  (3 dof = Pose2 x,y,theta; 99% confidence)
            # 0.99 is the community-standard operating point (Mangelson et al.,
            # ICRA 2018; AEROS, Antonante et al. 2022 uses chi2_inv(0.99,3)=11.35).
            # Known limitation (inherited from the canonical PCM implementation):
            # only ret_jk.cov is used — the joint covariance of both measurements
            # and the odometry path (plk) is omitted, making the gate marginally
            # stricter than the nominal 0.99. Correcting this needs odometry-cov
            # propagation (research-grade), deferred beyond P4.
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

    @staticmethod
    def create_full_noise_model(cov: np.ndarray) -> gtsam.noiseModel.Gaussian:
        """Create GTSAM noise model from covariance matrix.

        Args:
            cov: Covariance matrix

        Returns:
            GTSAM Gaussian noise model
        """
        return gtsam.noiseModel.Gaussian.Covariance(cov)

    def create_robust_full_noise_model(self, cov: np.ndarray) -> gtsam.noiseModel.Robust:
        """Create robust GTSAM noise model from covariance matrix.

        Uses self.robust_loop_c as the Cauchy kernel parameter (default 3.0).
        c controls at how many sigma the weight starts dropping:
        Cauchy(c=3.0).weight(3.0) ≈ 0.5 (conservative down-weighting).

        Args:
            cov: Covariance matrix

        Returns:
            GTSAM robust noise model with Cauchy kernel
        """
        model = gtsam.noiseModel.Gaussian.Covariance(cov)
        robust = gtsam.noiseModel.mEstimator.Cauchy.Create(self.robust_loop_c)
        return gtsam.noiseModel.Robust.Create(robust, model)
