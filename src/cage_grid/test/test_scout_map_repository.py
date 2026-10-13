import hashlib
import json
import os
import tempfile
import time
import unittest

from cage_grid.association import AssociationDecision
from cage_grid.persistent_map import PersistentLatticeMap
from cage_grid.scout_map_repository import (
    ArtifactValidationError,
    ScoutMapRepository,
    reference_sha256,
)


class ScoutMapRepositoryTest(unittest.TestCase):
    TASK_ID = 'scout-test-1'
    GEOMETRY_SHA = 'a' * 64
    ENVIRONMENT_SHA = 'b' * 64
    MAPPING_SHA = 'c' * 64
    SOFTWARE_SHA = 'd' * 64
    RUNTIME_SHA = 'e' * 64
    REFERENCE = (-35.363261, 149.165230, 584.0)

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = self.temporary.name
        self.directory = os.path.join(self.root, self.TASK_ID)
        os.makedirs(self.directory)
        self.nominal = {
            (1, 1): (5.0, 5.0, 0.0),
            (1, 2): (15.0, 5.0, 0.0),
        }
        self.polygons = {
            (1, 1): ((0.0, 0.0, 0.0), (10.0, 0.0, 0.0),
                     (10.0, 10.0, 0.0), (0.0, 10.0, 0.0)),
            (1, 2): ((10.0, 0.0, 0.0), (20.0, 0.0, 0.0),
                     (20.0, 10.0, 0.0), (10.0, 10.0, 0.0)),
        }
        self.started = time.time() - 10.0
        self.finished = time.time() - 5.0
        self._write_valid_bundle()
        self.repository = ScoutMapRepository(
            self.root, self.nominal, self.polygons, cage_id=0,
            anchor_enu=(0.0, 0.0, 0.0),
            geometry_sha256=self.GEOMETRY_SHA,
            reference_wgs84=self.REFERENCE,
            environment_geometry_sha256=self.ENVIRONMENT_SHA,
            environment_id='test-environment', minimum_coverage=1.0)

    def tearDown(self):
        self.temporary.cleanup()

    @staticmethod
    def _write_json(path, value):
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write('\n')

    @staticmethod
    def _hash(path):
        with open(path, 'rb') as stream:
            return hashlib.sha256(stream.read()).hexdigest()

    def _load(self, name):
        with open(os.path.join(self.directory, name), encoding='utf-8') as stream:
            return json.load(stream)

    def _rewrite_and_remanifest(self, name, payload):
        path = os.path.join(self.directory, name)
        self._write_json(path, payload)
        manifest = self._load('manifest.json')
        manifest['files'][name] = self._hash(path)
        self._write_json(os.path.join(self.directory, 'manifest.json'), manifest)

    def _write_valid_bundle(self):
        lattice = PersistentLatticeMap(
            1, 2, self.nominal, min_confirmations=1,
            map_uuid='map-uuid-1', task_id=self.TASK_ID, cage_id=0,
            created_at=10.0, nominal_polygons=self.polygons)
        observation = {
            'local_rows': 1,
            'local_cols': 2,
            'cells': [
                {'local_i': 0, 'local_j': 0, 'projected_center': (5.0, 5.0, 0.0),
                 'projected_covariance_xy': (0.25, 0.0, 0.0, 0.25),
                 'confidence': 0.95},
                {'local_i': 0, 'local_j': 1, 'projected_center': (15.0, 5.0, 0.0),
                 'projected_covariance_xy': (0.25, 0.0, 0.0, 0.25),
                 'confidence': 0.95},
            ],
        }
        lattice.apply_observation(
            observation,
            AssociationDecision(True, 1, 1, 'COMMITTED', tuple()), 11.0)
        metadata = {
            'task_id': self.TASK_ID,
            'cage_id': 0,
            'frame_id': 'map',
            'anchor_definition': 'vertex_0_0',
            'anchor_enu': [0.0, 0.0, 0.0],
            'geometry_sha256': self.GEOMETRY_SHA,
            'nominal_geometry_sha256': self.GEOMETRY_SHA,
            'mapping_config_sha256': self.MAPPING_SHA,
            'software_source_sha256': self.SOFTWARE_SHA,
            'observation_source': 'oracle_noisy',
            'reference_lat': self.REFERENCE[0],
            'reference_lon': self.REFERENCE[1],
            'reference_alt': self.REFERENCE[2],
            'reference_sha256': reference_sha256(self.REFERENCE),
            'environment_geometry_sha256': self.ENVIRONMENT_SHA,
            'environment_id': 'test-environment',
            'source_created_at_unix_sec': self.started,
            'source_completed_at_unix_sec': self.finished,
        }
        map_path = os.path.join(self.directory, 'map.json')
        PersistentLatticeMap.save_json_payload(
            map_path, lattice.finalize_snapshot(metadata, timestamp=12.0))
        request = {'schema_version': 2, 'task_id': self.TASK_ID, 'cage_id': 0}
        quality = {
            'schema_version': 2, 'task_id': self.TASK_ID, 'cage_id': 0,
            'map_uuid': lattice.map_uuid, 'map_version': lattice.map_version,
            'expected_cells': 2, 'confirmed_cells': 2, 'coverage': 1.0,
        }
        result = {
            'schema_version': 2, 'task_id': self.TASK_ID, 'cage_id': 0,
            'started_at': self.started, 'finished_at': self.finished,
            'terminal_status': 'COMPLETED', 'mapping_outcome': 'SUCCESS',
            'map': {
                'map_uuid': lattice.map_uuid, 'map_version': lattice.map_version,
                'frozen': True, 'geometry_sha256': self.GEOMETRY_SHA,
                'expected_cells': 2, 'confirmed_cells': 2, 'coverage': 1.0,
            },
        }
        for name, payload in (('request.json', request), ('quality.json', quality),
                              ('result.json', result)):
            self._write_json(os.path.join(self.directory, name), payload)
        files = {name: self._hash(os.path.join(self.directory, name))
                 for name in ('request.json', 'map.json', 'quality.json', 'result.json')}
        manifest = {
            'schema_version': 2,
            'artifact_bundle_schema_version': 2,
            'map_schema_version': 2,
            'result_schema_version': 2,
            'quality_schema_version': 2,
            'task_id': self.TASK_ID, 'cage_id': 0,
            'map_uuid': lattice.map_uuid, 'map_version': lattice.map_version,
            'reference_sha256': reference_sha256(self.REFERENCE),
            'nominal_geometry_sha256': self.GEOMETRY_SHA,
            'environment_geometry_sha256': self.ENVIRONMENT_SHA,
            'environment_id': 'test-environment',
            'mapping_config_sha256': self.MAPPING_SHA,
            'software_source_sha256': self.SOFTWARE_SHA,
            'runtime_config_sha256': self.RUNTIME_SHA,
            'created_at': self.started, 'finalized_at': self.finished,
            'created_at_unix_sec': self.started,
            'finalized_at_unix_sec': self.finished,
            'files': files,
        }
        self._write_json(os.path.join(self.directory, 'manifest.json'), manifest)

    def test_valid_v2_bundle_is_accepted(self):
        value = self.repository.validate(self.TASK_ID)
        self.assertEqual(value.map_uuid, 'map-uuid-1')
        self.assertEqual(value.confirmed_cells, 2)
        self.assertFalse(value.legacy_reference_assumed)

    def test_changed_artifact_is_rejected_by_manifest_hash(self):
        with open(os.path.join(self.directory, 'map.json'), 'a', encoding='utf-8') as stream:
            stream.write(' ')
        with self.assertRaises(ArtifactValidationError) as context:
            self.repository.validate(self.TASK_ID)
        self.assertEqual(context.exception.code, 'ARTIFACT_HASH_MISMATCH')

    def test_cross_file_map_version_mismatch_is_rejected(self):
        quality = self._load('quality.json')
        quality['map_version'] += 1
        self._rewrite_and_remanifest('quality.json', quality)
        with self.assertRaises(ArtifactValidationError) as context:
            self.repository.validate(self.TASK_ID)
        self.assertEqual(context.exception.code, 'MAP_VERSION_MISMATCH')

    def test_unfrozen_snapshot_is_rejected(self):
        payload = self._load('map.json')
        payload['frozen'] = False
        self._rewrite_and_remanifest('map.json', payload)
        with self.assertRaises(ArtifactValidationError) as context:
            self.repository.validate(self.TASK_ID)
        self.assertEqual(context.exception.code, 'MAP_NOT_FROZEN')

    def test_path_traversal_task_id_is_rejected(self):
        with self.assertRaises(ArtifactValidationError) as context:
            self.repository.validate('../outside')
        self.assertEqual(context.exception.code, 'INVALID_TASK_ID')


if __name__ == '__main__':
    unittest.main()
