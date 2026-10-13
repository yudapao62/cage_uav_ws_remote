"""Validated repository for immutable Scout artifact bundles."""
from dataclasses import dataclass
import hashlib
import hmac
import json
import math
import os
import re
import stat
import time

from cage_grid.persistent_map import IDENTITY_CONFIRMED, PersistentLatticeMap


TASK_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$')
SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')
ALLOWED_ARTIFACTS = frozenset((
    'request.json', 'route.json', 'map.json', 'events.jsonl', 'mission.bag',
    'quality.json', 'result.json'))
REQUIRED_ARTIFACTS = frozenset((
    'request.json', 'map.json', 'quality.json', 'result.json'))


class ArtifactValidationError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)
        self.message = str(message)


@dataclass(frozen=True)
class ValidatedScoutMap:
    task_id: str
    cage_id: int
    map_uuid: str
    map_version: int
    map_sha256: str
    manifest_sha256: str
    frozen: bool
    source_created_at_unix_sec: float
    source_completed_at_unix_sec: float
    confirmed_cells: int
    expected_cells: int
    coverage: float
    frame_id: str
    reference_wgs84: tuple
    reference_sha256: str
    geometry_sha256: str
    environment_geometry_sha256: str
    environment_id: str
    observation_source: str
    mapping_config_sha256: str
    software_source_sha256: str
    legacy_reference_assumed: bool
    legacy_environment_assumed: bool
    last_validated_at_unix_sec: float
    lattice: PersistentLatticeMap
    map_payload: dict
    manifest_payload: dict


def canonical_sha256(value):
    encoded = json.dumps(value, sort_keys=True, separators=(',', ':'),
                         ensure_ascii=True, allow_nan=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def reference_sha256(reference):
    return canonical_sha256({
        'reference_lat': float(reference[0]),
        'reference_lon': float(reference[1]),
        'reference_alt': float(reference[2]),
    })


def _strict_json_bytes(raw, name):
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise ArtifactValidationError(
                    'MANIFEST_INVALID' if name == 'manifest.json' else 'MAP_STRUCTURE_INVALID',
                    '{} contains duplicate key {}'.format(name, key))
            result[key] = value
        return result

    def reject_constant(value):
        raise ArtifactValidationError(
            'MANIFEST_INVALID' if name == 'manifest.json' else 'MAP_STRUCTURE_INVALID',
            '{} contains non-finite constant {}'.format(name, value))

    try:
        return json.loads(raw.decode('utf-8'), object_pairs_hook=pairs,
                          parse_constant=reject_constant)
    except ArtifactValidationError:
        raise
    except (UnicodeDecodeError, ValueError, TypeError) as exc:
        raise ArtifactValidationError(
            'MANIFEST_INVALID' if name == 'manifest.json' else 'MAP_STRUCTURE_INVALID',
            '{} is invalid JSON: {}'.format(name, exc))


def _sha256_bytes(raw):
    return hashlib.sha256(raw).hexdigest()


class ScoutMapRepository:
    def __init__(self, output_root, nominal_cells, nominal_polygons,
                 cage_id, frame_id='map', anchor_definition='vertex_0_0',
                 anchor_enu=(0.0, 0.0, 0.0), geometry_sha256='',
                 reference_wgs84=None, environment_geometry_sha256='',
                 environment_id='', minimum_coverage=0.95,
                 max_snapshot_age_sec=0.0, allow_legacy_v1=False,
                 legacy_profiles=None, current_preset='', current_seed=None,
                 max_json_bytes=16 * 1024 * 1024,
                 max_artifact_bytes=1024 * 1024 * 1024,
                 max_artifact_count=32):
        self.output_root = os.path.realpath(os.path.abspath(
            os.path.expanduser(str(output_root))))
        self.nominal_cells = dict(nominal_cells)
        self.nominal_polygons = dict(nominal_polygons or {})
        self.cage_id = int(cage_id)
        self.frame_id = str(frame_id)
        self.anchor_definition = str(anchor_definition)
        self.anchor_enu = tuple(float(value) for value in anchor_enu)
        self.geometry_sha256 = str(geometry_sha256)
        self.reference_wgs84 = (None if reference_wgs84 is None else
                                tuple(float(value) for value in reference_wgs84))
        self.environment_geometry_sha256 = str(environment_geometry_sha256 or '')
        self.environment_id = str(environment_id or '')
        self.minimum_coverage = float(minimum_coverage)
        self.max_snapshot_age_sec = float(max_snapshot_age_sec)
        self.allow_legacy_v1 = bool(allow_legacy_v1)
        self.legacy_profiles = dict(legacy_profiles or {})
        self.current_preset = str(current_preset or '')
        self.current_seed = None if current_seed is None else int(current_seed)
        self.max_json_bytes = int(max_json_bytes)
        self.max_artifact_bytes = int(max_artifact_bytes)
        self.max_artifact_count = int(max_artifact_count)
        if not 0.0 <= self.minimum_coverage <= 1.0:
            raise ValueError('minimum_coverage must be in [0,1]')

    def _task_directory(self, task_id):
        task_id = str(task_id or '')
        if not TASK_ID_PATTERN.fullmatch(task_id):
            raise ArtifactValidationError('INVALID_TASK_ID', 'invalid Scout task_id')
        path = os.path.realpath(os.path.join(self.output_root, task_id))
        if os.path.commonpath((self.output_root, path)) != self.output_root:
            raise ArtifactValidationError('PATH_OUTSIDE_OUTPUT_ROOT',
                                          'task directory escapes output root')
        try:
            info = os.lstat(path)
        except OSError:
            raise ArtifactValidationError('ARTIFACT_NOT_FOUND',
                                          'Scout artifact directory not found')
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise ArtifactValidationError('PATH_OUTSIDE_OUTPUT_ROOT',
                                          'Scout task path is not a regular directory')
        return path

    def _artifact_path(self, directory, name):
        if (not isinstance(name, str) or name not in ALLOWED_ARTIFACTS or
                os.path.basename(name) != name or '/' in name or '\\' in name):
            raise ArtifactValidationError('MANIFEST_INVALID',
                                          'manifest contains unsafe artifact name')
        path = os.path.join(directory, name)
        resolved = os.path.realpath(path)
        if os.path.commonpath((directory, resolved)) != directory:
            raise ArtifactValidationError('PATH_OUTSIDE_OUTPUT_ROOT',
                                          'artifact escapes task directory')
        try:
            info = os.lstat(path)
        except OSError:
            raise ArtifactValidationError('ARTIFACT_NOT_FOUND',
                                          'artifact is missing: {}'.format(name))
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
            raise ArtifactValidationError('PATH_OUTSIDE_OUTPUT_ROOT',
                                          'artifact is not a regular file: {}'.format(name))
        if info.st_size < 0 or info.st_size > self.max_artifact_bytes:
            raise ArtifactValidationError('ARTIFACT_TOO_LARGE',
                                          'artifact size exceeds limit: {}'.format(name))
        if name.endswith('.json') and info.st_size > self.max_json_bytes:
            raise ArtifactValidationError('ARTIFACT_TOO_LARGE',
                                          'JSON artifact exceeds limit: {}'.format(name))
        return path

    @staticmethod
    def _read_bytes(path):
        with open(path, 'rb') as stream:
            return stream.read()

    def _read_bundle(self, task_id):
        directory = self._task_directory(task_id)
        manifest_path = os.path.join(directory, 'manifest.json')
        try:
            info = os.lstat(manifest_path)
        except OSError:
            raise ArtifactValidationError('MANIFEST_MISSING', 'manifest.json is missing')
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
            raise ArtifactValidationError('MANIFEST_INVALID',
                                          'manifest.json is not a regular file')
        if info.st_size > self.max_json_bytes:
            raise ArtifactValidationError('ARTIFACT_TOO_LARGE',
                                          'manifest.json exceeds size limit')
        manifest_raw = self._read_bytes(manifest_path)
        manifest = _strict_json_bytes(manifest_raw, 'manifest.json')
        files = manifest.get('files')
        if not isinstance(files, dict) or len(files) > self.max_artifact_count:
            raise ArtifactValidationError('MANIFEST_INVALID', 'manifest files is invalid')
        if not REQUIRED_ARTIFACTS.issubset(files):
            missing = sorted(REQUIRED_ARTIFACTS - set(files))
            raise ArtifactValidationError('MANIFEST_INVALID',
                                          'required artifact not registered: {}'.format(','.join(missing)))
        payloads = {'manifest.json': manifest}
        hashes = {'manifest.json': _sha256_bytes(manifest_raw)}
        for name, expected_hash in files.items():
            expected_hash = str(expected_hash or '').lower()
            if not SHA256_PATTERN.fullmatch(expected_hash):
                raise ArtifactValidationError('MANIFEST_INVALID',
                                              'invalid hash for {}'.format(name))
            path = self._artifact_path(directory, name)
            raw = self._read_bytes(path)
            actual_hash = _sha256_bytes(raw)
            if not hmac.compare_digest(actual_hash, expected_hash):
                raise ArtifactValidationError('ARTIFACT_HASH_MISMATCH',
                                              '{} hash mismatch'.format(name))
            hashes[name] = actual_hash
            if name.endswith('.json'):
                payloads[name] = _strict_json_bytes(raw, name)
        return directory, manifest, payloads, hashes

    @staticmethod
    def _equal_float(left, right, tolerance=1e-6):
        return math.isfinite(float(left)) and math.isfinite(float(right)) and abs(
            float(left) - float(right)) <= tolerance

    @staticmethod
    def _require(condition, code, message):
        if not condition:
            raise ArtifactValidationError(code, message)

    def validate(self, task_id, now_unix_sec=None):
        _, manifest, payloads, hashes = self._read_bundle(task_id)
        request = payloads['request.json']
        map_payload = payloads['map.json']
        quality = payloads['quality.json']
        result = payloads['result.json']
        task_id = str(task_id)

        for name, payload in (('request', request), ('map', map_payload),
                              ('quality', quality), ('result', result),
                              ('manifest', manifest)):
            self._require(str(payload.get('task_id', '')) == task_id,
                          'TASK_ID_MISMATCH', '{} task_id mismatch'.format(name))
        cage_id = int(map_payload.get('cage_id', -1))
        self._require(int(request.get('cage_id', -1)) == cage_id and
                      int(result.get('cage_id', -1)) == cage_id,
                      'CAGE_ID_MISMATCH', 'artifact cage_id mismatch')
        self._require(cage_id == self.cage_id, 'CAGE_ID_MISMATCH',
                      'map does not belong to active cage')

        map_uuid = str(map_payload.get('map_uuid', ''))
        map_version = int(map_payload.get('map_version', 0))
        self._require(bool(map_uuid) and map_version > 0, 'MAP_STRUCTURE_INVALID',
                      'invalid map identity')
        for name, value in (('quality', quality),
                            ('result', result.get('map') or {}),
                            ('manifest', manifest)):
            self._require(str(value.get('map_uuid', '')) == map_uuid,
                          'MAP_UUID_MISMATCH', '{} map_uuid mismatch'.format(name))
            self._require(int(value.get('map_version', -1)) == map_version,
                          'MAP_VERSION_MISMATCH', '{} map_version mismatch'.format(name))

        map_result = result.get('map') or {}
        metadata = dict(map_payload.get('metadata') or {})
        geometry_sha = str(map_payload.get('geometry_sha256', ''))
        self._require(bool(geometry_sha), 'GEOMETRY_MISMATCH',
                      'map geometry hash is missing')
        self._require(str(metadata.get('geometry_sha256', geometry_sha)) == geometry_sha and
                      str(map_result.get('geometry_sha256', '')) == geometry_sha,
                      'GEOMETRY_MISMATCH', 'artifact geometry hash mismatch')
        if self.geometry_sha256:
            self._require(geometry_sha == self.geometry_sha256, 'GEOMETRY_MISMATCH',
                          'map nominal geometry does not match runtime')
        self._require(str(map_payload.get('frame_id', metadata.get('frame_id', ''))) == self.frame_id,
                      'FRAME_MISMATCH', 'map frame mismatch')
        self._require(str(map_payload.get('anchor_definition', metadata.get('anchor_definition', ''))) ==
                      self.anchor_definition, 'ANCHOR_MISMATCH', 'map anchor definition mismatch')
        anchor = map_payload.get('anchor_enu', metadata.get('anchor_enu', []))
        self._require(isinstance(anchor, list) and len(anchor) == 3 and
                      all(math.isfinite(float(value)) for value in anchor) and
                      all(abs(float(anchor[index]) - self.anchor_enu[index]) <= 1e-6
                          for index in range(3)),
                      'ANCHOR_MISMATCH', 'map anchor position mismatch')

        try:
            lattice = PersistentLatticeMap.from_json_payload(
                map_payload, self.nominal_cells,
                expected_metadata={
                    'geometry_sha256': geometry_sha,
                    'anchor_definition': self.anchor_definition,
                    'frame_id': self.frame_id,
                    'cage_id': self.cage_id,
                }, nominal_polygons=self.nominal_polygons)
        except (KeyError, TypeError, ValueError) as exc:
            raise ArtifactValidationError('MAP_STRUCTURE_INVALID', str(exc))
        self._require(bool(map_payload.get('frozen')) and bool(map_result.get('frozen')) and
                      lattice.frozen, 'MAP_NOT_FROZEN', 'map is not frozen')
        self._require(str(result.get('terminal_status', '')) == 'COMPLETED',
                      'MAP_QUALITY_REJECTED', 'Scout task was not completed')
        self._require(str(result.get('mapping_outcome', '')) == 'SUCCESS',
                      'MAP_QUALITY_REJECTED', 'Scout mapping did not succeed')

        expected = int(map_payload['rows']) * int(map_payload['cols'])
        confirmed = sum(1 for cell in lattice.cells.values()
                        if cell.identity_state == IDENTITY_CONFIRMED)
        coverage = float(confirmed) / expected if expected else 0.0
        for name, value in (('quality', quality), ('result', map_result)):
            self._require(int(value.get('expected_cells', -1)) == expected and
                          int(value.get('confirmed_cells', -1)) == confirmed and
                          self._equal_float(value.get('coverage', -1.0), coverage),
                          'MAP_QUALITY_REJECTED', '{} map counts mismatch'.format(name))
        self._require(coverage >= self.minimum_coverage,
                      'MAP_QUALITY_REJECTED', 'map coverage below minimum')

        manifest_created = float(manifest.get('created_at', result.get('started_at', 0.0)))
        manifest_finalized = float(manifest.get('finalized_at', result.get('finished_at', 0.0)))
        self._require(all(math.isfinite(value) for value in
                          (manifest_created, manifest_finalized)) and
                      manifest_finalized >= manifest_created,
                      'CLOCK_INVALID', 'invalid artifact wall timestamps')
        result_started = float(result.get('started_at', manifest_created))
        result_finished = float(result.get('finished_at', manifest_finalized))
        self._require(result_started <= result_finished and
                      abs(result_finished - manifest_finalized) <= 300.0,
                      'CLOCK_INVALID', 'result/manifest wall timestamps disagree')

        map_sha = hashes['map.json']
        manifest_sha = hashes['manifest.json']
        self._require(str(map_result.get('map_sha256', map_sha)) == map_sha,
                      'ARTIFACT_HASH_MISMATCH',
                      'result map_sha256 does not match map.json')
        schema_v2 = all(key in map_payload for key in (
            'reference_lat', 'reference_lon', 'reference_alt', 'reference_sha256',
            'environment_geometry_sha256', 'environment_id'))
        legacy_reference = legacy_environment = False
        if schema_v2:
            self._require(int(manifest.get('artifact_bundle_schema_version', 0)) == 2 and
                          int(manifest.get('map_schema_version', 0)) == 2 and
                          int(manifest.get('result_schema_version', 0)) == 2 and
                          int(manifest.get('quality_schema_version', 0)) == 2,
                          'MANIFEST_INVALID', 'v2 artifact schema versions are invalid')
            reference = (float(map_payload['reference_lat']),
                         float(map_payload['reference_lon']),
                         float(map_payload['reference_alt']))
            ref_sha = str(map_payload['reference_sha256'])
            self._require(ref_sha == reference_sha256(reference),
                          'REFERENCE_MISMATCH', 'reference hash mismatch')
            environment_sha = str(map_payload['environment_geometry_sha256'])
            environment_id = str(map_payload['environment_id'])
            self._require(str(map_payload.get('nominal_geometry_sha256', '')) ==
                          geometry_sha, 'GEOMETRY_MISMATCH',
                          'v2 nominal geometry hash mismatch')
            for key in ('reference_sha256', 'nominal_geometry_sha256',
                        'environment_geometry_sha256', 'environment_id',
                        'mapping_config_sha256', 'software_source_sha256'):
                self._require(str(manifest.get(key, '')) == str(map_payload.get(key, '')),
                              'MANIFEST_INVALID',
                              'v2 manifest/map {} mismatch'.format(key))
            for key in ('runtime_config_sha256', 'mapping_config_sha256',
                        'software_source_sha256'):
                self._require(bool(SHA256_PATTERN.fullmatch(
                    str(manifest.get(key, '')).lower())),
                    'MANIFEST_INVALID', '{} is not SHA-256'.format(key))
            created_v2 = float(manifest.get('created_at_unix_sec', float('nan')))
            finalized_v2 = float(manifest.get('finalized_at_unix_sec', float('nan')))
            self._require(self._equal_float(created_v2, manifest_created) and
                          self._equal_float(finalized_v2, manifest_finalized),
                          'CLOCK_INVALID', 'v2 manifest timestamps disagree')
            self._require(self._equal_float(
                map_payload.get('source_created_at_unix_sec', float('nan')),
                manifest_created, tolerance=300.0) and
                self._equal_float(
                    map_payload.get('source_completed_at_unix_sec', float('nan')),
                    manifest_finalized, tolerance=300.0),
                'CLOCK_INVALID', 'v2 map/manifest source timestamps disagree')
        else:
            self._require(self.allow_legacy_v1, 'LEGACY_REFERENCE_NOT_ALLOWED',
                          'legacy Scout maps are disabled')
            profile = self.legacy_profiles.get(map_sha)
            self._require(isinstance(profile, dict), 'LEGACY_PROFILE_MISMATCH',
                          'no legacy compatibility profile for map hash')
            reference = tuple(float(profile[key]) for key in
                              ('reference_lat', 'reference_lon', 'reference_alt'))
            ref_sha = reference_sha256(reference)
            environment_sha = str(profile.get('environment_geometry_sha256', ''))
            environment_id = str(profile.get('environment_id', ''))
            self._require(bool(environment_sha) and bool(environment_id),
                          'LEGACY_PROFILE_MISMATCH',
                          'legacy profile must bind environment id and hash')
            self._require(str(profile.get('preset', '')) == str(manifest.get('preset', '')) and
                          int(profile.get('seed', -1)) == int(manifest.get('seed', -2)),
                          'LEGACY_PROFILE_MISMATCH', 'legacy preset/seed mismatch')
            if self.current_preset:
                self._require(str(manifest.get('preset', '')) == self.current_preset,
                              'LEGACY_PROFILE_MISMATCH', 'legacy preset differs from runtime')
            if self.current_seed is not None:
                self._require(int(manifest.get('seed', -1)) == self.current_seed,
                              'LEGACY_PROFILE_MISMATCH', 'legacy seed differs from runtime')
            legacy_reference = legacy_environment = True

        self._require(-90.0 <= reference[0] <= 90.0 and
                      -180.0 <= reference[1] <= 180.0 and
                      all(math.isfinite(value) for value in reference),
                      'REFERENCE_MISMATCH', 'invalid WGS84 reference')
        if self.reference_wgs84 is not None:
            self._require(all(abs(reference[index] - self.reference_wgs84[index]) <= 1e-9
                              for index in range(3)),
                          'REFERENCE_MISMATCH', 'map reference differs from runtime')
        if self.environment_geometry_sha256:
            self._require(environment_sha == self.environment_geometry_sha256,
                          'GEOMETRY_MISMATCH', 'environment geometry hash mismatch')
        if self.environment_id:
            self._require(environment_id == self.environment_id,
                          'GEOMETRY_MISMATCH', 'environment id mismatch')

        now = float(time.time() if now_unix_sec is None else now_unix_sec)
        self._require(math.isfinite(now) and manifest_finalized <= now + 1e-6,
                      'CLOCK_INVALID', 'artifact completion time is in the future')
        if self.max_snapshot_age_sec > 0.0:
            self._require(now - manifest_finalized <= self.max_snapshot_age_sec,
                          'SOURCE_MAP_EXPIRED', 'Scout map exceeded source age policy')

        observation_source = str(manifest.get(
            'observation_source', map_payload.get('observation_source', '')))
        return ValidatedScoutMap(
            task_id=task_id, cage_id=cage_id, map_uuid=map_uuid,
            map_version=map_version, map_sha256=map_sha,
            manifest_sha256=manifest_sha, frozen=True,
            source_created_at_unix_sec=manifest_created,
            source_completed_at_unix_sec=manifest_finalized,
            confirmed_cells=confirmed, expected_cells=expected,
            coverage=coverage, frame_id=self.frame_id,
            reference_wgs84=reference, reference_sha256=ref_sha,
            geometry_sha256=geometry_sha,
            environment_geometry_sha256=environment_sha,
            environment_id=environment_id,
            observation_source=observation_source,
            mapping_config_sha256=str(map_payload.get(
                'mapping_config_sha256', map_payload.get('configuration_sha256', ''))),
            software_source_sha256=str(manifest.get(
                'software_source_sha256', manifest.get('source_sha256', ''))),
            legacy_reference_assumed=legacy_reference,
            legacy_environment_assumed=legacy_environment,
            last_validated_at_unix_sec=now, lattice=lattice,
            map_payload=map_payload, manifest_payload=manifest)

    def list_task_ids(self):
        try:
            entries = os.scandir(self.output_root)
        except OSError:
            return []
        result = []
        with entries:
            for entry in entries:
                if (TASK_ID_PATTERN.fullmatch(entry.name) and
                        entry.is_dir(follow_symlinks=False)):
                    result.append(entry.name)
        return sorted(result)
