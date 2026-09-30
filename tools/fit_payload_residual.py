#!/usr/bin/env python3
"""Offline exploratory fit of paired breakaway thresholds; never commands hardware.

Symmetric friction is an assumption, not an observed ground truth. Each pair uses
both measured start poses. Failed probes are excluded. Output is NOT deployable.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from compare_urdf_gravity import Model


def regressor(model, q):
    """Payload gravity per [kg, kg*m, kg*m, kg*m] at gripper_base."""
    original = model.links['gripper_base']
    try:
        columns = []
        model.links['gripper_base'] = (1., np.zeros(3))
        base = model.evaluate(q)[2]['gripper_base']
        columns.append(base)
        for axis in np.eye(3):
            model.links['gripper_base'] = (1., axis)
            columns.append(model.evaluate(q)[2]['gripper_base'] - base)
        return np.column_stack(columns)
    finally:
        model.links['gripper_base'] = original


def collect(path, model_hash):
    pairs, pending, positive = [], None, None
    group, valid = 0, False
    with Path(path).open('rb') as stream:
        for line in stream:
            if b'"type":"friction_' not in line:
                continue
            r = json.loads(line)
            kind = r['type']
            if kind == 'friction_batch_start':
                group += 1
                valid = r.get('gravity_source') == 'urdf' and r.get('model_sha256') == model_hash
                pending = positive = None
            elif kind == 'friction_probe_start':
                pending = r if valid and r.get('profile') == 'wrist_accumulated_v3' else None
            elif kind == 'friction_probe_result':
                if pending is None or r.get('outcome') != 'onset_candidate':
                    pending = positive = None
                    continue
                assert (pending['joint'], pending['direction']) == (r['joint'], r['direction'])
                sample = dict(group=group, joint=r['joint']-1, q=pending['q_start'],
                              tau=r['candidate_threshold_nm'])
                if r['direction'] == 1:
                    positive = sample
                elif positive and positive['joint'] == sample['joint']:
                    pairs.append((positive, sample))
                    positive = None
                pending = None
            elif kind in ('friction_batch_aborted', 'friction_batch_complete'):
                valid = False
                pending = positive = None
    return pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model')
    parser.add_argument('log')
    args = parser.parse_args()
    model = Model(args.model)
    digest = hashlib.sha256(Path(args.model).read_bytes()).hexdigest()
    pairs = collect(args.log, digest)
    if not pairs:
        raise ValueError('No compatible successful pairs')
    rows, target, groups, joints = [], [], [], []
    for plus, minus in pairs:
        j = plus['joint']
        rows.append((regressor(model, plus['q'])[j] + regressor(model, minus['q'])[j])/2)
        target.append((plus['tau'] + minus['tau'])/2)
        groups.append(plus['group']); joints.append(j)
    A, y, groups, joints = map(np.asarray, (rows, target, groups, joints))
    # Aggregate repeated pairs to give each posture/joint equal weight.
    keys = sorted(set(zip(groups.tolist(), joints.tolist())))
    X = np.array([np.mean(A[(groups==g)&(joints==j)], axis=0) for g,j in keys])
    z = np.array([np.median(y[(groups==g)&(joints==j)]) for g,j in keys])
    gidx = np.array([g for g,j in keys]); jidx = np.array([j for g,j in keys])
    delta, _, rank, singular = np.linalg.lstsq(X, z, rcond=None)
    mass, com = model.links['gripper_base']
    nominal = np.r_[mass, mass*com]
    fitted = nominal + delta
    rmse = lambda a: float(np.sqrt(np.mean(np.asarray(a)**2)))
    validation = []
    for g in sorted(set(gidx)):
        train, test = gidx != g, gidx == g
        d = np.linalg.lstsq(X[train], z[train], rcond=None)[0]
        # Simple extra compensation comparator; never applied automatically.
        offset = np.array([np.mean(z[train & (jidx==j)]) for j in range(6)])
        validation.append(dict(held_out_group=int(g), baseline_rmse=rmse(z[test]),
            payload_rmse=rmse(z[test]-X[test]@d),
            constant_offset_rmse=rmse(z[test]-offset[jidx[test]])))
    print(json.dumps(dict(model_sha256=digest, successful_pairs=len(pairs), rank=int(rank),
        singular_values=singular.tolist(), delta_mass_first_moments=delta.tolist(),
        fitted_mass_kg=float(fitted[0]),
        fitted_com_m=(fitted[1:]/fitted[0]).tolist() if abs(fitted[0])>1e-9 else None,
        baseline_rmse_nm=rmse(z), payload_training_rmse_nm=rmse(z-X@delta),
        validation=validation,
        posture_joint_rows=[dict(group=g,joint=j+1,midpoint_nm=float(z[i]),
            predicted_payload_correction_nm=float(X[i]@delta)) for i,(g,j) in enumerate(keys)],
        warning='Exploratory only. Symmetric friction, start-pose approximation; no hardware/config writes.'), indent=2))


if __name__ == '__main__':
    main()
