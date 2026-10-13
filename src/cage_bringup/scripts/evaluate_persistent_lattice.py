#!/usr/bin/env python3
"""Aggregate Scout/Feeder JSONL acceptance records into a stable report."""
import argparse
import json
import math
import os
from collections import defaultdict


def safe_ratio(numerator, denominator):
    return float(numerator) / denominator if denominator else 0.0


def evaluate(records):
    associations = [item for item in records if item.get('type') == 'association']
    coordinates = [item for item in records if item.get('type') == 'coordinate']
    feeder = [item for item in records if item.get('type') == 'feeder']
    safety = [item for item in records if item.get('type') == 'safety']
    ablations = [item for item in records if item.get('type') == 'ablation']
    committed = [item for item in associations if item.get('committed')]
    correct = [item for item in committed if item.get('correct')]
    slips = [item for item in committed if item.get('slip')]
    nominal_squared = [float(item['nominal_error_m']) ** 2 for item in coordinates]
    estimated_squared = [float(item['estimated_error_m']) ** 2 for item in coordinates]
    nominal_rmse = math.sqrt(sum(nominal_squared) / len(nominal_squared)) if nominal_squared else 0.0
    estimated_rmse = math.sqrt(sum(estimated_squared) / len(estimated_squared)) if estimated_squared else 0.0
    feeder_commits = [item for item in feeder if item.get('status') == 'CONFIRMED']
    wrong_commits = [item for item in feeder_commits if not item.get('correct')]
    ambiguous_cases = [item for item in feeder if item.get('expected_ambiguous')]
    ambiguous_rejected = [item for item in ambiguous_cases if item.get('status') != 'CONFIRMED']
    result = {
        'schema_version': 1,
        'scout': {
            'association_count': len(associations),
            'committed_count': len(committed),
            'identity_accuracy': safe_ratio(len(correct), len(committed)),
            'slip_count': len(slips),
            'slip_rate': safe_ratio(len(slips), len(associations)),
            'coverage': safe_ratio(len(committed), len(associations)),
            'nominal_rmse_m': nominal_rmse,
            'estimated_rmse_m': estimated_rmse,
            'nominal_improvement': (1.0 - estimated_rmse / nominal_rmse
                                    if nominal_rmse > 0.0 else 0.0),
        },
        'feeder': {
            'sample_count': len(feeder),
            'candidate_recall': safe_ratio(
                sum(bool(item.get('truth_in_candidates')) for item in feeder), len(feeder)),
            'commit_count': len(feeder_commits),
            'wrong_commit_count': len(wrong_commits),
            'wrong_commit_rate': safe_ratio(len(wrong_commits), len(feeder_commits)),
            'ambiguous_recall': safe_ratio(len(ambiguous_rejected), len(ambiguous_cases)),
        },
        'safety': {
            'forbidden_feed_trigger_count': sum(
                int(item.get('feed_triggered', False))
                for item in safety
                if item.get('identity_status') in ('AMBIGUOUS', 'LOST', 'STALE_MAP')),
        },
    }
    grouped = defaultdict(list)
    for item in ablations:
        grouped[item['name']].append(float(item['value']))
    result['ablations'] = {
        name: {
            'metric': next(item['metric'] for item in ablations
                           if item['name'] == name),
            'mean': sum(values) / len(values),
            'sample_count': len(values),
        }
        for name, values in sorted(grouped.items())
    }
    scout = result['scout']
    feeder_result = result['feeder']
    result['thresholds'] = {
        'identity_accuracy_ge_0_995': scout['identity_accuracy'] >= 0.995,
        'slip_count_eq_0': scout['slip_count'] == 0,
        'nominal_improvement_ge_0_30': scout['nominal_improvement'] >= 0.30,
        'candidate_recall_ge_0_99': feeder_result['candidate_recall'] >= 0.99,
        'wrong_commit_rate_le_0_01': feeder_result['wrong_commit_rate'] <= 0.01,
        'forbidden_feed_trigger_count_eq_0':
            result['safety']['forbidden_feed_trigger_count'] == 0,
    }
    result['passed'] = all(result['thresholds'].values())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, help='JSONL record path')
    parser.add_argument('--output', required=True, help='report JSON path')
    args = parser.parse_args()
    with open(args.input, encoding='utf-8') as stream:
        records = [json.loads(line) for line in stream if line.strip()]
    report = evaluate(records)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(json.dumps(report, indent=2, sort_keys=True))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
