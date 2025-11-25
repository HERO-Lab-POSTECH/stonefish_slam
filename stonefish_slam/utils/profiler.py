"""
Mapping profiling utilities for performance monitoring.
"""
import csv
import time
from pathlib import Path


class MappingProfiler:
    """CSV-based profiler for 3D mapping performance tracking"""

    def __init__(self, csv_path: str, sample_interval: int = 10):
        """
        Args:
            csv_path: Path to CSV output file
            sample_interval: Record every N frames
        """
        self.csv_path = Path(csv_path)
        self.sample_interval = sample_interval
        self.csv_file = None
        self.csv_writer = None
        self.current_frame = 0
        self.metrics = {}

    def start(self):
        """Initialize CSV file with headers"""
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write header
        self.csv_writer.writerow([
            'frame_id', 'timestamp_sec', 'total_ms', 'ray_ms', 'octree_ms',
            'exp_calls', 'exp_ms', 'map_voxels', 'memory_mb', 'dedup_count'
        ])
        self.csv_file.flush()

    def record_frame(self, frame_id: int, **metrics):
        """Record metrics for current frame"""
        self.current_frame = frame_id
        self.metrics = metrics

        # Write to CSV if sampling interval matches
        if frame_id % self.sample_interval == 0:
            self.write_csv_row()

    def write_csv_row(self):
        """Write current metrics to CSV"""
        if self.csv_writer is None:
            return

        row = [
            self.current_frame,
            self.metrics.get('timestamp', time.time()),
            self.metrics.get('total_ms', 0),
            self.metrics.get('ray_ms', 0),
            self.metrics.get('octree_ms', 0),
            self.metrics.get('exp_calls', 0),
            self.metrics.get('exp_ms', 0),
            self.metrics.get('map_voxels', 0),
            self.metrics.get('memory_mb', 0),
            self.metrics.get('dedup_count', 0)
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def close(self):
        """Close CSV file"""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
