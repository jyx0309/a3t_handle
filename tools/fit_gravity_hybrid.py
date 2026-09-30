#!/usr/bin/env python3
"""Offline constrained physical-parameter fit, followed by residual shrinkage.

Bounds are exploratory priors, NOT verified CAD tolerances. Never writes a URDF
or commands hardware. Breakaway midpoint is only a gravity-residual proxy.
"""
import argparse
import hashlib
import json
import numpy as np
from scipy.optimize import lsq_linear
from compare_urdf_gravity import Model
from fit_payload_residual import collect

LINKS = ['link2', 'link3', 'link4', 'gripper_base']


def basis(model, q):
    columns = []
    for name in LINKS:
        mass, com = model.links[name]
        original = model.links[name]
        try:
            base = model.evaluate(q)[2][name]
            # Fixed mass, COM shifts. Payload additionally has mass freedom at
            # its nominal COM; this is a linear first-moment parameterization.
            if name == 'gripper_base':
                columns.append(base / mass)
            for axis in np.eye(3):
                model.links[name] = (mass, com + axis)
                columns.append(model.evaluate(q)[2][name] - base)
        finally:
            model.links[name] = original
    return np.column_stack(columns)


def fit(X, y, joints):
    # 20 mm arm COM prior; payload +/-100g and 30 mm equivalent COM prior.
    scales = np.array([.02]*9 + [.1] + [.03]*3)
    matrix = X * scales
    result = lsq_linear(np.vstack([matrix, np.sqrt(.1)*np.eye(13)]),
                        np.r_[y, np.zeros(13)], bounds=(-1, 1), tol=1e-12)
    if not result.success:
        raise RuntimeError(result.message)
    theta = result.x * scales
    remaining = y-X@theta
    residual = np.zeros(6)
    # Only J2/J3/J4, after physical correction, half strength, bounded.
    for j in (1, 2, 3):
        residual[j] = np.clip(.5*np.median(remaining[joints==j]), -.5, .5)
    return theta, residual, np.flatnonzero(np.abs(result.x)>.99).tolist()


def analyze(path, log):
    model = Model(path)
    with open(path, 'rb') as stream:
        digest = hashlib.sha256(stream.read()).hexdigest()
    pairs = collect(log, digest)
    rows = {}
    for a,b in pairs:
        j = a['joint']; key = (a['group'],j)
        rows.setdefault(key, []).append(((basis(model,a['q'])[j]+basis(model,b['q'])[j])/2,
                                        (a['tau']+b['tau'])/2))
    keys = sorted(rows)
    X = np.array([np.mean([v[0] for v in rows[k]],axis=0) for k in keys])
    y = np.array([np.median([v[1] for v in rows[k]]) for k in keys])
    groups = np.array([k[0] for k in keys]); joints = np.array([k[1] for k in keys])
    theta,residual,bounds = fit(X,y,joints)
    rmse = lambda e: float(np.sqrt(np.mean(e**2)))
    validation=[]
    for group in sorted(set(groups)):
        train,test=groups!=group,groups==group
        t,r,hit=fit(X[train],y[train],joints[train])
        validation.append(dict(group=int(group),baseline=rmse(y[test]),
            physical=rmse(y[test]-X[test]@t),
            hybrid=rmse(y[test]-X[test]@t-r[joints[test]]),bound_hits=hit))
    contributions=[]
    for group in sorted(set(groups)):
        a=next(a for a,b in pairs if a['group']==group and a['joint']==1)
        gravity,_,parts,_=model.evaluate(a['q'])
        contributions.append(dict(group=int(group),q=a['q'],J2_total=float(gravity[1]),
            J2_parts={name:float(v[1]) for name,v in parts.items()}))
    return dict(deployable=False,model_sha256=digest,successful_pairs=len(pairs),
        parameter_order='link2/3/4 COM xyz shifts; payload dm then equivalent COM xyz shifts',
        parameter_delta=theta.tolist(),bound_hits=bounds,rank=int(np.linalg.matrix_rank(X)),
        residual_nm=residual.tolist(),baseline_rmse=rmse(y),
        physical_rmse=rmse(y-X@theta),hybrid_rmse=rmse(y-X@theta-residual[joints]),
        leave_one_pose_out=validation,contributions=contributions,
        warnings=['Only four postures, fourth incomplete; parameters not uniquely identifiable.',
                  'Midpoints assume symmetric friction. Limits are priors, not measured geometry.',
                  'Cross-validation reused for analysis; independent validation is still required.'])


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model');parser.add_argument('log')
    args=parser.parse_args()
    print(json.dumps(analyze(args.model,args.log),indent=2))
