"""Pure fail-closed feeder identity safety policy."""


STATUS_CONFIRMED = 0
STATUS_AMBIGUOUS = 1
STATUS_LOST = 2
STATUS_STALE_MAP = 3
STATUS_MAP_MISMATCH = 4


def identity_gate_decision(required, status, identity_cell_id,
                           expected_cell_id, age_sec, timeout_sec,
                           active_map_pin=None, identity_task_map_pin=None,
                           expected_task_map_pin=None):
    if not required:
        return True, 'DISABLED'
    if age_sec is None or age_sec > float(timeout_sec):
        return False, 'STALE_IDENTITY'
    if int(status) != STATUS_CONFIRMED:
        names = {
            STATUS_AMBIGUOUS: 'AMBIGUOUS',
            STATUS_LOST: 'LOST',
            STATUS_STALE_MAP: 'STALE_MAP',
            STATUS_MAP_MISMATCH: 'TASK_MAP_MISMATCH',
        }
        return False, names.get(int(status), 'UNCONFIRMED')
    if expected_cell_id and identity_cell_id != expected_cell_id:
        return False, 'CELL_MISMATCH'
    if not identity_cell_id:
        return False, 'EMPTY_CELL_ID'
    if expected_task_map_pin:
        expected = tuple(expected_task_map_pin)
        if not all(expected):
            return False, 'EMPTY_TASK_MAP_PIN'
        if tuple(active_map_pin or ()) != expected:
            return False, 'ACTIVE_MAP_MISMATCH'
        if tuple(identity_task_map_pin or ()) != expected:
            return False, 'TASK_MAP_MISMATCH'
    return True, 'CONFIRMED'
