#!/usr/bin/env python3
import csv
import os
from pathlib import Path

import yaml


def experiment_dir():
    return Path(__file__).resolve().parent.parent


def load_config(path=None):
    path = Path(path) if path else experiment_dir() / 'config' / 'experiment_topics.yaml'
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream), path


def resolve_result(value):
    path = Path(value).expanduser().resolve()
    if path.is_file():
        return path.parent, path
    return path, path / 'raw.bag'


def load_metadata(result_dir):
    path = result_dir / 'metadata.yaml'
    if not path.exists():
        return {}
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def write_metrics(path, rows):
    with Path(path).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['category', 'metric', 'value', 'unit'])
        writer.writerows(rows)


def metric(rows, category, name, value, unit=''):
    if value is None:
        rows.append((category, name, '', unit))
    else:
        rows.append((category, name, '{:.9g}'.format(float(value)), unit))


def ensure_bag(path):
    if not path.exists():
        raise SystemExit('Bag not found: {}'.format(path))
    if path.name.endswith('.active'):
        raise SystemExit('Bag is still active; stop rosbag cleanly before analysis')


def ros_time(value):
    return float(value.to_sec())


def yaml_topic(config, key):
    return config.get('topics', {}).get(key)


def finite(*values):
    import math
    return all(math.isfinite(float(v)) for v in values)
