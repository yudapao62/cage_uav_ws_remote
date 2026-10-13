import unittest
import os
import tempfile

from cage_grid.association import AssociationDecision
from cage_grid.persistent_map import (
    IDENTITY_CONFIRMED,
    VISIBILITY_EXPECTED_BUT_UNOBSERVED,
    VISIBILITY_OUT_OF_VIEW,
    VISIBILITY_VISIBLE,
    PersistentLatticeMap,
)


def nominal(rows, cols):
    return {(row, col): ((col - 0.5) * 10.0, (row - 0.5) * 10.0, 0.0)
            for row in range(1, rows + 1) for col in range(1, cols + 1)}


def observed(local_cells, local_rows, local_cols):
    return {
        'local_rows': local_rows,
        'local_cols': local_cols,
        'cells': [{
            'local_i': local_i,
            'local_j': local_j,
            'projected_center': ((local_j + 0.5) * 10.0,
                                 (local_i + 0.5) * 10.0, 0.0),
            'projected_covariance_xy': (0.25, 0.0, 0.0, 0.25),
            'confidence': 0.9,
        } for local_i, local_j in local_cells],
    }


COMMIT = AssociationDecision(True, 1, 1, 'COMMITTED', tuple())


class PersistentMapTest(unittest.TestCase):
    def test_three_observations_confirm_and_identity_persists(self):
        lattice = PersistentLatticeMap(1, 3, nominal(1, 3), min_confirmations=3)
        frame = observed([(0, 0), (0, 1), (0, 2)], 1, 3)
        for timestamp in (1.0, 2.0, 3.0):
            lattice.apply_observation(frame, COMMIT, timestamp)
        self.assertEqual(len(lattice.cells), 3)
        self.assertTrue(all(cell.identity_state == IDENTITY_CONFIRMED
                            for cell in lattice.cells.values()))
        original_ids = {index: cell.cell_id for index, cell in lattice.cells.items()}

        missed = observed([(0, 0), (0, 2)], 1, 3)
        lattice.apply_observation(missed, COMMIT, 4.0)
        self.assertEqual(lattice.cells[(1, 2)].visibility_state,
                         VISIBILITY_EXPECTED_BUT_UNOBSERVED)
        self.assertEqual({index: cell.cell_id for index, cell in lattice.cells.items()},
                         original_ids)

        recovery_events = lattice.apply_observation(frame, COMMIT, 4.5)
        self.assertEqual(lattice.cells[(1, 2)].visibility_state,
                         VISIBILITY_VISIBLE)
        self.assertTrue(any(event['type'] == 'RECOVER'
                            and event['cell_id'] == 'R01C02'
                            for event in recovery_events))

        narrow = observed([(0, 0)], 1, 1)
        lattice.apply_observation(narrow, COMMIT, 5.0)
        self.assertEqual(lattice.cells[(1, 2)].visibility_state,
                         VISIBILITY_OUT_OF_VIEW)
        self.assertEqual(lattice.cells[(1, 2)].identity_state, IDENTITY_CONFIRMED)

    def test_ambiguous_decision_freezes_map(self):
        lattice = PersistentLatticeMap(1, 2, nominal(1, 2))
        ambiguous = AssociationDecision(False, 0, 0, 'TOP1_TOP2_MARGIN_TOO_SMALL', tuple())
        lattice.apply_observation(observed([(0, 0)], 1, 1), ambiguous, 1.0)
        self.assertEqual(lattice.map_version, 0)
        self.assertEqual(lattice.cells, {})
        self.assertEqual(lattice.events[-1]['type'], 'FREEZE')

    def test_versioned_snapshot_requires_explicit_matching_metadata(self):
        lattice = PersistentLatticeMap(1, 2, nominal(1, 2), map_uuid='map-test')
        lattice.apply_observation(observed([(0, 0)], 1, 1), COMMIT, 1.0)
        metadata = {'geometry_sha256': 'abc', 'anchor_definition': 'vertex_0_0'}
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'map.json')
            lattice.save_json(path, metadata)
            restored = PersistentLatticeMap.load_json(
                path, nominal(1, 2), expected_metadata=metadata)
            self.assertEqual(restored.map_uuid, 'map-test')
            self.assertEqual(restored.map_version, lattice.map_version)
            self.assertEqual(restored.cells[(1, 1)].cell_id, 'R01C01')
            with self.assertRaises(ValueError):
                PersistentLatticeMap.load_json(
                    path, nominal(1, 2),
                    expected_metadata={'geometry_sha256': 'different'})

    def test_unseen_hole_geometry_is_inferred_from_confirmed_neighbor(self):
        lattice = PersistentLatticeMap(1, 3, nominal(1, 3))
        lattice.apply_observation(observed([(0, 0)], 1, 1), COMMIT, 1.0)
        lattice.cells[(1, 1)].estimated_position = (7.0, 8.0, 0.0)
        candidate = lattice.candidate_cells(3.0)[(1, 2)]
        self.assertEqual(candidate['position'], (17.0, 8.0, 0.0))
        self.assertFalse(candidate['established'])

    def test_fresh_session_changes_uuid_and_clears_task_state(self):
        lattice = PersistentLatticeMap(
            1, 2, nominal(1, 2), map_uuid='old-map',
            task_id='old-task', cage_id=0)
        lattice.apply_observation(observed([(0, 0)], 1, 1), COMMIT, 1.0)
        reset = lattice.fresh_session('new-task', cage_id=0, created_at=2.0)
        self.assertNotEqual(reset.map_uuid, lattice.map_uuid)
        self.assertEqual(reset.task_id, 'new-task')
        self.assertEqual(reset.cage_id, 0)
        self.assertEqual(reset.map_version, 0)
        self.assertEqual(reset.cells, {})

    def test_finalize_snapshot_is_detached_atomic_and_reloadable(self):
        polygons = {
            (1, 1): ((0.0, 0.0, 0.0), (10.0, 0.0, 0.0),
                     (10.0, 10.0, 0.0), (0.0, 10.0, 0.0)),
            (1, 2): ((10.0, 0.0, 0.0), (20.0, 0.0, 0.0),
                     (20.0, 10.0, 0.0), (10.0, 10.0, 0.0)),
        }
        lattice = PersistentLatticeMap(
            1, 2, nominal(1, 2), min_confirmations=1,
            map_uuid='final-map', task_id='task-1', cage_id=0,
            created_at=1.0, nominal_polygons=polygons)
        lattice.apply_observation(
            observed([(0, 0), (0, 1)], 1, 2), COMMIT, 2.0)
        metadata = {
            'geometry_sha256': 'geometry',
            'configuration_sha256': 'configuration',
            'anchor_definition': 'vertex_0_0',
            'anchor_enu': [0.0, 0.0, 0.0],
            'frame_id': 'map',
            'observation_source': 'oracle_noisy',
        }
        payload = lattice.finalize_snapshot(metadata, timestamp=3.0)
        self.assertTrue(payload['frozen'])
        self.assertEqual(payload['task_id'], 'task-1')
        self.assertEqual(payload['cage_id'], 0)
        self.assertEqual(payload['finalized_at'], 3.0)
        self.assertEqual(len(payload['cells']), 2)
        self.assertEqual(len(payload['edges']), 1)
        self.assertEqual(len(payload['cells'][0]['polygon']), 4)
        with self.assertRaises(RuntimeError):
            lattice.apply_observation(
                observed([(0, 0)], 1, 1), COMMIT, 4.0)

        payload['cells'][0]['cell_id'] = 'MUTATED'
        self.assertEqual(
            lattice.to_json_dict(metadata)['cells'][0]['cell_id'], 'R01C01')
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'map.json')
            stable_payload = lattice.finalize_snapshot(metadata, timestamp=5.0)
            PersistentLatticeMap.save_json_payload(path, stable_payload)
            self.assertFalse(any(name.startswith('.persistent_lattice_')
                                 for name in os.listdir(directory)))
            restored = PersistentLatticeMap.load_json(
                path, nominal(1, 2), expected_metadata=metadata,
                nominal_polygons=polygons)
            self.assertTrue(restored.frozen)
            self.assertEqual(restored.task_id, 'task-1')
            self.assertEqual(restored.map_uuid, 'final-map')
            self.assertEqual(restored.map_version, lattice.map_version)


if __name__ == '__main__':
    unittest.main()
