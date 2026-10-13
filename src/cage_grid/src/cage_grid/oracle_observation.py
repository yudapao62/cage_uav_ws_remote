"""ROS-free causal observation synthesis from current ground-truth cells."""
import math
import random


def cells_in_window(cells, row_min=None, row_max=None, col_min=None, col_max=None):
    selected = []
    for cell in cells:
        row, col = int(cell['row']), int(cell['col'])
        if not cell.get('exists', True):
            continue
        if row_min is not None and row < int(row_min):
            continue
        if row_max is not None and row > int(row_max):
            continue
        if col_min is not None and col < int(col_min):
            continue
        if col_max is not None and col > int(col_max):
            continue
        selected.append(cell)
    return selected


def cells_in_oriented_footprint(cells, center_xy, yaw, width, height):
    if width <= 0.0 or height <= 0.0:
        raise ValueError('footprint width and height must be positive')
    cosine, sine = math.cos(yaw), math.sin(yaw)
    selected = []
    for cell in cells:
        if not cell.get('exists', True):
            continue
        x, y = cell['center_enu'][:2]
        dx, dy = float(x) - center_xy[0], float(y) - center_xy[1]
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        if abs(local_x) <= width / 2.0 and abs(local_y) <= height / 2.0:
            selected.append(cell)
    return selected


class OracleObservationGenerator:
    """Create one observation using only a current visible-cell set."""

    def __init__(self, random_seed=42, position_noise_std_m=0.0,
                 dropout_probability=0.0, edge_observed_probability=1.0):
        self.random_seed = int(random_seed)
        self.position_noise_std_m = float(position_noise_std_m)
        self.dropout_probability = float(dropout_probability)
        self.edge_observed_probability = float(edge_observed_probability)
        if self.position_noise_std_m < 0.0:
            raise ValueError('position noise must be non-negative')
        if not 0.0 <= self.dropout_probability <= 1.0:
            raise ValueError('dropout probability must be in [0,1]')
        if not 0.0 <= self.edge_observed_probability <= 1.0:
            raise ValueError('edge observed probability must be in [0,1]')

    def generate(self, visible_cells, frame_seq, forced_drop_ids=None):
        visible = [cell for cell in visible_cells if cell.get('exists', True)]
        if not visible:
            return {'frame_seq': int(frame_seq), 'local_rows': 0, 'local_cols': 0,
                    'cells': [], 'expected_cell_ids': []}
        row_min = min(int(cell['row']) for cell in visible)
        row_max = max(int(cell['row']) for cell in visible)
        col_min = min(int(cell['col']) for cell in visible)
        col_max = max(int(cell['col']) for cell in visible)
        forced = {str(value).upper() for value in (forced_drop_ids or [])}
        rng = random.Random(self.random_seed + 1000003 * int(frame_seq))
        observed = []
        for cell in sorted(visible, key=lambda item: (int(item['row']), int(item['col']))):
            if str(cell['cell_id']).upper() in forced or rng.random() < self.dropout_probability:
                continue
            center = cell['center_enu']
            noisy_center = [
                float(center[0]) + rng.gauss(0.0, self.position_noise_std_m),
                float(center[1]) + rng.gauss(0.0, self.position_noise_std_m),
                float(center[2]) if len(center) > 2 else 0.0,
            ]
            polygon = cell.get('polygon_enu', [])
            quad_uv = [[float(point[0]), float(point[1]), 0.0] for point in polygon]
            edge_state = [1 if rng.random() < self.edge_observed_probability else 0
                          for _ in range(4)]
            confidence = 0.25 + 0.75 * sum(edge_state) / 4.0
            observed.append({
                'local_i': int(cell['row']) - row_min,
                'local_j': int(cell['col']) - col_min,
                'quad_uv': quad_uv,
                'projected_center': noisy_center,
                'projected_covariance_xy': [self.position_noise_std_m ** 2, 0.0,
                                            0.0, self.position_noise_std_m ** 2],
                'edge_state': edge_state,
                'confidence': confidence,
                'expected_visible': True,
                # Debug-only bookkeeping for tests/evaluator.  ROS conversion deliberately drops it.
                '_ground_truth_cell_id': cell['cell_id'],
            })
        return {
            'frame_seq': int(frame_seq),
            'local_rows': row_max - row_min + 1,
            'local_cols': col_max - col_min + 1,
            'cells': observed,
            'expected_cell_ids': [cell['cell_id'] for cell in visible],
        }

